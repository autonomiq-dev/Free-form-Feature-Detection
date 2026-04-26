import numpy as np
import pyvista as pv

from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS_Shape, TopoDS_Face, TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.StlAPI import StlAPI_Writer


def load_step(path: str) -> TopoDS_Shape:
    reader = STEPControl_Reader()
    reader.ReadFile(path)
    reader.TransferRoots()
    return reader.OneShape()


def read_step_from_user(step_path: str):
    shape = load_step(step_path)
    faces_list: list[TopoDS_Face] = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        faces_list.append(TopoDS.Face_s(exp.Current()))
        exp.Next()
    return shape, faces_list


def _build_face_mesh(shape, faces_list, mesh_deflection=0.01, angular_deflection=0.5) -> pv.PolyData:
    BRepMesh_IncrementalMesh(shape, mesh_deflection, False, angular_deflection, True).Perform()

    all_verts, all_cells, all_face_ids = [], [], []
    offset = 0

    for face_id, face in enumerate(faces_list):
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(trsf)
            all_verts.append([p.X(), p.Y(), p.Z()])
        for t in tri.Triangles():
            i1, i2, i3 = t.Value(1), t.Value(2), t.Value(3)
            a, b, c = offset + i1 - 1, offset + i2 - 1, offset + i3 - 1
            all_cells.extend([3, a, b, c])
            all_face_ids.append(face_id)
        offset += tri.NbNodes()

    poly = pv.PolyData(
        np.array(all_verts, dtype=float),
        np.array(all_cells, dtype=np.int64),
    )
    poly.cell_data["face_id"] = np.array(all_face_ids, dtype=np.int32)
    return poly


def pick_brep_face(shape, faces_list, mesh_deflection=0.01) -> TopoDS_Face:
    mesh = _build_face_mesh(shape, faces_list, mesh_deflection=mesh_deflection)

    picked_points, picked_cells = [], []

    def _on_pick(point):
        picked_points.append(point)
        picked_cells.append(mesh.find_closest_cell(point))

    n_cells = mesh.n_cells
    colors = np.tile(np.array([200, 200, 200], dtype=np.uint8), (n_cells, 1))
    mesh.cell_data["colors"] = colors

    plotter = pv.Plotter()
    plotter.add_mesh(mesh, scalars="colors", rgb=True)
    plotter.enable_surface_point_picking(callback=_on_pick)
    plotter.show()

    cell_id = int(picked_cells[-1])
    face_id = int(mesh.cell_data["face_id"][cell_id])
    return faces_list[face_id]


def visualize_faces_on_mesh(shape, faces_list, selected_face_ids, mesh_deflection=0.001):
    mesh = _build_face_mesh(shape, faces_list, mesh_deflection=mesh_deflection)
    selected = set(int(i) for i in selected_face_ids)
    face_ids = mesh.cell_data.get("face_id")
    colors = np.tile(np.array([200, 200, 200], dtype=np.uint8), (mesh.n_cells, 1))
    colors[np.isin(face_ids, list(selected))] = [255, 0, 0]
    mesh.cell_data["colors"] = colors
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, scalars="colors", rgb=True)
    plotter.show()


def save_shape_to_step(shape: TopoDS_Shape, path: str) -> None:
    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    writer.Write(path)


def save_shape_to_stl(
    shape: TopoDS_Shape,
    path: str,
    mesh_deflection: float = 0.001,
    angular_deflection: float = 0.5,
) -> None:
    BRepMesh_IncrementalMesh(shape, mesh_deflection, False, angular_deflection, True).Perform()
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    writer.Write(shape, path)
