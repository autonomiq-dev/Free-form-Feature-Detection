import numpy as np
import trimesh
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
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


def _resolve_feature_faces(feature_faces, faces_list=None):
    items = list(feature_faces)
    if all(isinstance(item, int) for item in items):
        return [faces_list[i] for i in items]
    return items
