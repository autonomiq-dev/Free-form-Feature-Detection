import numpy as np
import trimesh
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.gp import gp_Pnt


def bounding_box(feature_faces, faces_list=None) -> dict:
    faces = _resolve_feature_faces(feature_faces, faces_list=faces_list)

    # sew the feature faces into a single shell
    sewing = BRepBuilderAPI_Sewing()
    for face in faces:
        sewing.Add(face)
    sewing.Perform()
    feature_shape = sewing.SewedShape()

    bbox = Bnd_Box()
    BRepBndLib.Add_s(feature_shape, bbox)

    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return {
        "x": (xmin, xmax),
        "y": (ymin, ymax),
        "z": (zmin, zmax),
    }


def build_feature_box_from_bbox(
    feature_bbox,
    stock_shape,
    output_path: str = "feature_bbox_solid.stl",
):
    xmin, xmax = feature_bbox["x"]
    ymin, ymax = feature_bbox["y"]
    zmin_feature, _ = feature_bbox["z"]

    stock_bbox = Bnd_Box()
    BRepBndLib.Add_s(stock_shape, stock_bbox)
    _, _, _, _, _, zmax_stock = stock_bbox.Get()

    dx = xmax - xmin
    dy = ymax - ymin
    dz = zmax_stock - zmin_feature

    center = np.array([
        (xmin + xmax) / 2.0,
        (ymin + ymax) / 2.0,
        (zmin_feature + zmax_stock) / 2.0,
    ])

    transform = trimesh.transformations.translation_matrix(center)
    box = trimesh.creation.box(extents=[dx, dy, dz], transform=transform)
    box.export(output_path)
    return box


def build_feature_bbox_solid_occ(feature_bbox: dict, stock_shape: TopoDS_Shape) -> TopoDS_Shape:
    """Return an OCC box solid spanning the XY footprint of the feature and
    the full Z range from the feature bottom to the top of the stock.

    This is always a valid closed solid, making it safe to pass directly to
    OCC boolean operations (BRepAlgoAPI_Cut / BRepAlgoAPI_Common).
    """
    xmin, xmax = feature_bbox["x"]
    ymin, ymax = feature_bbox["y"]
    zmin_feat, _ = feature_bbox["z"]

    stock_box = Bnd_Box()
    BRepBndLib.Add_s(stock_shape, stock_box)
    _, _, _, _, _, zmax_stock = stock_box.Get()

    return BRepPrimAPI_MakeBox(
        gp_Pnt(xmin, ymin, zmin_feat),
        gp_Pnt(xmax, ymax, zmax_stock),
    ).Shape()


def bounding_box_from_stl(stl_path: str) -> dict:
    mesh = trimesh.load(stl_path, force="mesh")
    (xmin, ymin, zmin), (xmax, ymax, zmax) = mesh.bounds
    return {
        "x": (float(xmin), float(xmax)),
        "y": (float(ymin), float(ymax)),
        "z": (float(zmin), float(zmax)),
    }


def feature_faces_to_mesh(
    feature_faces,
    faces_list=None,
    linear_deflection: float = 0.1,
    angular_deflection: float = 0.5,
) -> trimesh.Trimesh:
    """
    Sew BRep feature faces into a shell, triangulate it, and return a trimesh.

    Parameters
    ----------
    feature_faces : Iterable[TopoDS_Face | int]
        OCC faces or face indices that define the feature.
    faces_list : list[TopoDS_Face] | None
        Full face list used to resolve indices in feature_faces.
    linear_deflection : float
        Maximum linear deviation for tessellation (smaller = finer mesh).
    angular_deflection : float
        Maximum angular deviation in radians for tessellation.

    Returns
    -------
    trimesh.Trimesh
        Triangle mesh of the sewn feature shell.
    """
    faces = _resolve_feature_faces(feature_faces, faces_list=faces_list)

    sewing = BRepBuilderAPI_Sewing()
    for face in faces:
        sewing.Add(face)
    sewing.Perform()
    shell = sewing.SewedShape()

    mesh_algo = BRepMesh_IncrementalMesh(shell, linear_deflection, False, angular_deflection)
    mesh_algo.Perform()

    vertices = []
    triangles = []
    vertex_offset = 0

    explorer = TopExp_Explorer(shell, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = face.Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)

        if triangulation is not None:
            n_nodes = triangulation.NbNodes()
            n_tris = triangulation.NbTriangles()

            face_verts = np.array([
                [triangulation.Node(i).X(),
                 triangulation.Node(i).Y(),
                 triangulation.Node(i).Z()]
                for i in range(1, n_nodes + 1)
            ])
            vertices.append(face_verts)

            for i in range(1, n_tris + 1):
                n1, n2, n3 = triangulation.Triangle(i).Get()
                triangles.append([
                    vertex_offset + n1 - 1,
                    vertex_offset + n2 - 1,
                    vertex_offset + n3 - 1,
                ])
            vertex_offset += n_nodes

        explorer.Next()

    if not vertices:
        raise RuntimeError("No triangulation could be extracted from feature faces")

    return trimesh.Trimesh(
        vertices=np.vstack(vertices),
        faces=np.array(triangles, dtype=np.int64),
        process=True,
    )


def build_material_removal_mesh(
    feature_box: trimesh.Trimesh,
    feature_faces,
    faces_list=None,
    output_path: str = "material_to_be_removed.stl",
    linear_deflection: float = 0.1,
    angular_deflection: float = 0.5,
) -> trimesh.Trimesh:
   
    # --- 1. Convert BRep feature faces → mesh shell ---
    feature_shell = feature_faces_to_mesh(
        feature_faces,
        faces_list=faces_list,
        linear_deflection=linear_deflection,
        angular_deflection=angular_deflection,
    )

    # --- 2. Drop the bottom face of the feature_box ---
    # The bottom sits at the minimum Z of the box (zmin_feature).
    z_bottom = float(feature_box.bounds[0][2])
    tol = 1e-6 * (feature_box.bounds[1][2] - z_bottom)  # relative tolerance

    face_centers_z = feature_box.triangles_center[:, 2]
    keep_mask = face_centers_z > (z_bottom + tol)        # True = keep
    open_box = trimesh.Trimesh(
        vertices=feature_box.vertices,
        faces=feature_box.faces[keep_mask],
        process=False,
    )

    # --- 3. Flip feature shell normals to point outward from the volume ---
    # The sewn shell normals point away from the solid; reverse them so they
    # point into the removal volume (i.e. downward / into the part).
    feature_shell_flipped = feature_shell.copy()
    feature_shell_flipped.faces = feature_shell_flipped.faces[:, ::-1]

    # --- 4. Combine and export ---
    removal_mesh = trimesh.util.concatenate([open_box, feature_shell_flipped])
    removal_mesh.export(output_path)
    return removal_mesh


def _resolve_feature_faces(feature_faces, faces_list=None):
    items = list(feature_faces)
    if all(isinstance(item, int) for item in items):
        return [faces_list[i] for i in items]
    return items
