import numpy as np
import pyvista as pv

from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_COMPSOLID
from OCP.TopExp import TopExp_Explorer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.ShapeFix import ShapeFix_Shape


def _iter_solids(shape: TopoDS_Shape):
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        yield exp.Current()
        exp.Next()


def _bboxes_intersect(a: TopoDS_Shape, b: TopoDS_Shape) -> bool:
    box_a, box_b = Bnd_Box(), Bnd_Box()
    BRepBndLib.Add_s(a, box_a)
    BRepBndLib.Add_s(b, box_b)
    return not box_a.IsOut(box_b)


def extrude_feature_patch(feature_faces, direction=(0, 0, 1), length=200):
    sewing = BRepBuilderAPI_Sewing(1e-3)
    for f in feature_faces:
        sewing.Add(f)
    sewing.Perform()
    shell = sewing.SewedShape()

    fixer = ShapeFix_Shape(shell)
    fixer.Perform()
    shell = fixer.Shape()

    vec = gp_Vec(direction[0], direction[1], direction[2])
    vec.Multiply(length)

    return BRepPrimAPI_MakePrism(shell, vec).Shape()


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
    extrusion: TopoDS_Shape
):
    if not _bboxes_intersect(stock_shape, extrusion):
        raise RuntimeError(
            "Stock and extrusion bounding boxes do not overlap — "
            "check extrusion direction or length."
        )

    # BRepAlgoAPI_Common rejects COMPSOLID inputs. Intersect each sub-solid
    # with the stock individually, then fuse the partial results together.
    if extrusion.ShapeType() == TopAbs_COMPSOLID:
        result = None
        for solid in _iter_solids(extrusion):
            common_op = BRepAlgoAPI_Common(stock_shape, solid)
            common_op.Build()
            if not common_op.IsDone():
                continue
            part = common_op.Shape()
            if result is None:
                result = part
            else:
                fuse_op = BRepAlgoAPI_Fuse(result, part)
                fuse_op.Build()
                if fuse_op.IsDone():
                    result = fuse_op.Shape()

        if result is None:
            raise RuntimeError("All sub-solid Boolean operations failed.")
        return result

    # Regular case: SOLID or COMPOUND.
    common_op = BRepAlgoAPI_Common(stock_shape, extrusion)
    common_op.Build()
    if not common_op.IsDone():
        raise RuntimeError(
            "Boolean common between stock and removal feature failed; check input shapes."
        )
    return common_op.Shape()


def visualize_feature_removal_volume(removal_volume, final_removal, part_shape):
    rv_mesh = _shape_to_pyvista(removal_volume)
    fr_mesh = _shape_to_pyvista(final_removal)
    part_mesh = _shape_to_pyvista(part_shape)
    plotter = pv.Plotter(border=False, title="Feature Removal Volume")
    if fr_mesh.n_points > 0:
        plotter.add_mesh(fr_mesh, color="tomato", opacity=0.8)
    else:
        plotter.add_mesh(rv_mesh, color="gray", opacity=0.8)
    plotter.add_mesh(part_mesh, color="gray", opacity=0.8)
    plotter.show()


def get_extrusion_length(stock_shape, direction):
    stock_bbox = Bnd_Box()
    BRepBndLib.Add_s(stock_shape, stock_bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = stock_bbox.Get()

    if direction in ((0, 0, 1), (0, 0, -1)):
        return zmax - zmin
    elif direction in ((0, 1, 0), (0, -1, 0)):
        return ymax - ymin
    elif direction in ((1, 0, 0), (-1, 0, 0)):
        return xmax - xmin
    else:
        raise ValueError(f"Invalid direction: {direction}")
