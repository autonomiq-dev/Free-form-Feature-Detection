from OCP.Bnd import Bnd_Box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Splitter
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
from OCP.TopTools import TopTools_ListOfShape


def bounding_box(feature_faces, faces_list=None):
    """
    Compute the axis-aligned XYZ bounding box of feature faces.

    Parameters
    ----------
    feature_faces : Iterable[TopoDS_Face | int]
        Collection of OCC faces or face indices that define the feature.
    faces_list : list[TopoDS_Face] | None
        Full face list used to resolve indices in feature_faces.

    Returns
    -------
    dict
        {
            "x": (xmin, xmax),
            "y": (ymin, ymax),
            "z": (zmin, zmax),
        }
    """
    faces = _resolve_feature_faces(feature_faces, faces_list=faces_list)

    # Build a shell-like shape from faces so one bbox call covers all faces.
    sewing = BRepBuilderAPI_Sewing()
    for face in faces:
        sewing.Add(face)
    sewing.Perform()
    feature_shape = sewing.SewedShape()

    bbox = Bnd_Box()
    BRepBndLib.Add_s(feature_shape, bbox)

    if bbox.IsVoid():
        raise RuntimeError("Failed to compute bounding box for feature_faces")

    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return {
        "x": (xmin, xmax),
        "y": (ymin, ymax),
        "z": (zmin, zmax),
    }


def _resolve_feature_faces(feature_faces, faces_list=None):
    items = list(feature_faces)
    if not items:
        raise ValueError("feature_faces is empty")
    if all(isinstance(item, int) for item in items):
        if faces_list is None:
            raise ValueError("faces_list is required when feature_faces contains face IDs")
        return [faces_list[i] for i in items]
    return items


def build_feature_box_from_bbox(feature_bbox, stock_shape):
    """
    Build a solid box using feature bbox XY extents.

    Z extents:
      - bottom = feature_bbox lower Z
      - top    = stock_shape highest Z
    """
    xmin, xmax = feature_bbox["x"]
    ymin, ymax = feature_bbox["y"]
    zmin_feature, _ = feature_bbox["z"]

    stock_bbox = Bnd_Box()
    BRepBndLib.Add_s(stock_shape, stock_bbox)
    if stock_bbox.IsVoid():
        raise RuntimeError("Failed to compute stock bounding box")
    _, _, _, _, _, zmax_stock = stock_bbox.Get()

    dx = xmax - xmin
    dy = ymax - ymin
    dz = zmax_stock - zmin_feature
    if dx <= 0 or dy <= 0 or dz <= 0:
        raise ValueError(
            "Invalid box dimensions from feature/stock bounds: "
            f"dx={dx}, dy={dy}, dz={dz}"
        )

    base = gp_Pnt(xmin, ymin, zmin_feature)
    return BRepPrimAPI_MakeBox(base, dx, dy, dz).Shape()


