import numpy as np
import pyvista as pv
from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCP.gp import gp_Vec, gp_Dir
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

# Build a shell from feature faces and extrude it
def extrude_feature_patch(feature_faces, direction=(0,0,1), length=200):
    sewing = BRepBuilderAPI_Sewing()
    for f in feature_faces:
        sewing.Add(f)
    sewing.Perform()
    shell = sewing.SewedShape()

    vec = gp_Vec(gp_Dir(direction[0], direction[1], direction[2]))
    vec.Multiply(length)

     # BRepPrimAPI_MakePrism on an open shell produces a compound of faces
    # (bottom + top + lateral walls) but does NOT sew them into a solid.
    prism_shape = BRepPrimAPI_MakePrism(shell, vec).Shape()

   
    # Re-sew all faces and attempt to build a solid so that downstream boolean
    # operations (trimesh or OCC) receive a proper closed volume.
    re_sew = BRepBuilderAPI_Sewing(1e-3)
    face_exp = TopExp_Explorer(prism_shape, TopAbs_FACE)
    while face_exp.More():
        re_sew.Add(face_exp.Current())
        face_exp.Next()
    re_sew.Perform()
    sewn = re_sew.SewedShape()

    solid_builder = BRepBuilderAPI_MakeSolid()
    shell_exp = TopExp_Explorer(sewn, TopAbs_SHELL)
    added = False
    while shell_exp.More():
        solid_builder.Add(TopoDS.Shell_s(shell_exp.Current()))
        added = True
        shell_exp.Next()

    if added and solid_builder.IsDone():
        return solid_builder.Solid()

    # Fallback: the patch boundary is open (feature faces don't form a closed
    # perimeter), so the prism cannot be sealed automatically.  Return the
    # compound and let the caller decide (repair or switch to bbox approach).
    print(
        "Warning: extrusion prism is an open shell — its boundary edges are not "
        "closed because the selected feature patch has free perimeter edges. "
        "Boolean operations may fail. Consider using the bounding-box approach "
        "(remove_feature_bbox) or the BREP pipeline (compute_feature_removal_volume)."
    )
    return prism_shape

# BREP to PyVista mesh
def _shape_to_pyvista(shape: TopoDS_Shape, mesh_deflection: float = 0.1) -> pv.PolyData:
    BRepMesh_IncrementalMesh(shape, mesh_deflection).Perform()

    all_verts, all_cells = [], []
    offset = 0

    face_exp = TopExp_Explorer(shape, TopAbs_FACE)
    while face_exp.More():
        face = TopoDS.Face_s(face_exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                all_verts.append([p.X(), p.Y(), p.Z()])
            for t in tri.Triangles():
                i1, i2, i3 = t.Value(1), t.Value(2), t.Value(3)
                all_cells.extend([3, offset + i1 - 1, offset + i2 - 1, offset + i3 - 1])
            offset += tri.NbNodes()
        face_exp.Next()

    if not all_verts:
        return pv.PolyData()
    return pv.PolyData(np.array(all_verts, dtype=float), np.array(all_cells, dtype=np.int64))

def compute_feature_removal_volume(
    stock_shape: TopoDS_Shape,
    part_shape: TopoDS_Shape,
    extrusion: TopoDS_Shape
):
    # # 1) Compute removal volume: stock minus part.
    # cut_op = BRepAlgoAPI_Cut(stock_shape, part_shape)
    # cut_op.Build()
    # if not cut_op.IsDone():
    #     raise RuntimeError(
    #         "Boolean cut (stock minus part) failed; check that both shapes are valid solids."
    #     )
    # removal_volume = cut_op.Shape()

    # 2) Intersect in-process stock with the feature volume.
    common_op = BRepAlgoAPI_Common(stock_shape, extrusion)
    common_op.Build()
    # if not common_op.IsDone():
    #     raise RuntimeError(
    #         "Boolean common between Stock and removal feature failed; check input shapes."
    #     )
    final_removal = common_op.Shape()

    return final_removal

def visualize_feature_removal_volume(removal_volume, final_removal, part_shape):
    rv_mesh = _shape_to_pyvista(removal_volume)
    fr_mesh = _shape_to_pyvista(final_removal)
    part_mesh = _shape_to_pyvista(part_shape)
    plotter = pv.Plotter(border=False, title="Feature Removal Volume")
    if fr_mesh.n_points > 0:
        plotter.add_mesh(fr_mesh, color="tomato", opacity=0.8)
    else:
        plotter.add_mesh(rv_mesh, color="gray", opacity=0.8)
    plotter.add_mesh(part_mesh, color="blue", opacity=0.8)
    plotter.show()

def get_extrusion_length(stock_shape, direction):
    stock_bbox = Bnd_Box()
    BRepBndLib.Add_s(stock_shape, stock_bbox)
    xmin_stock, ymin_stock, zmin_stock, xmax_stock, ymax_stock, zmax_stock = stock_bbox.Get()
    if direction == (0,0,1) or direction == (0,0,-1):
        return zmax_stock - zmin_stock
    elif direction == (0,1,0) or direction == (0,-1,0):
        return ymax_stock - ymin_stock
    elif direction == (1,0,0) or direction == (-1,0,0):
        return xmax_stock - xmin_stock
    else:
        raise ValueError(f"Invalid direction: {direction}")