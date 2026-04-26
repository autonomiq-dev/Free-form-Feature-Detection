from __future__ import annotations
import math
import networkx as nx

from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE, TopAbs_VERTEX, TopAbs_Orientation
from OCP.TopoDS import TopoDS_Face, TopoDS_Edge, TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepTools import BRepTools
from OCP.ShapeAnalysis import ShapeAnalysis_Surface
from OCP.gp import gp_Pnt, gp_Dir
from OCP.GeomAbs import (
    GeomAbs_Plane,
    GeomAbs_Cylinder,
    GeomAbs_Cone,
    GeomAbs_Sphere,
    GeomAbs_Torus,
    GeomAbs_BezierSurface,
    GeomAbs_BSplineSurface,
    GeomAbs_SurfaceOfRevolution,
    GeomAbs_SurfaceOfExtrusion,
    GeomAbs_OffsetSurface,
    GeomAbs_OtherSurface,
)
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


SURFACE_TYPE_MAP = {
    GeomAbs_Plane: "Plane",
    GeomAbs_Cylinder: "Cylinder",
    GeomAbs_Cone: "Cone",
    GeomAbs_Sphere: "Sphere",
    GeomAbs_Torus: "Torus",
    GeomAbs_BezierSurface: "Bezier",
    GeomAbs_BSplineSurface: "BSpline",
    GeomAbs_SurfaceOfRevolution: "Revolution",
    GeomAbs_SurfaceOfExtrusion: "Extrusion",
    GeomAbs_OffsetSurface: "Offset",
    GeomAbs_OtherSurface: "Other",
}


def _edge_key(edge: TopoDS_Edge):
    verts = []
    expv = TopExp_Explorer(edge, TopAbs_VERTEX)
    while expv.More():
        v = TopoDS.Vertex_s(expv.Current())
        p = BRep_Tool.Pnt_s(v)
        verts.append((round(p.X(), 6), round(p.Y(), 6), round(p.Z(), 6)))
        expv.Next()
    verts.sort()
    return tuple(verts)


def _surface_type_for_face(face: TopoDS_Face):
    adaptor = BRepAdaptor_Surface(face, True)
    return SURFACE_TYPE_MAP.get(adaptor.GetType(), f"Other({adaptor.GetType()})")


def _find_shared_edge(face1: TopoDS_Face, face2: TopoDS_Face):
    """Return the first shared edge between two faces, or None."""
    edges1: dict[tuple, TopoDS_Edge] = {}
    exp = TopExp_Explorer(face1, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        edges1[_edge_key(edge)] = edge
        exp.Next()

    exp = TopExp_Explorer(face2, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        if _edge_key(edge) in edges1:
            return edge
        exp.Next()
    return None


def _face_normal_at_point(face: TopoDS_Face, point: gp_Pnt) -> gp_Dir:
    """Compute the outward surface normal of face at a 3D point lying on it."""
    surf = BRep_Tool.Surface_s(face)
    uv = ShapeAnalysis_Surface(surf).ValueOfUV(point, 1e-7)
    adaptor = BRepAdaptor_Surface(face, False)
    props = BRepLProp_SLProps(adaptor, uv.X(), uv.Y(), 1, 1e-6)
    if not props.IsNormalDefined():
        raise ValueError("Surface normal is undefined at the given point.")
    normal = props.Normal()
    if face.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
        normal.Reverse()
    return normal


def build_face_adjacency(faces_list: list) -> nx.Graph:
    edge_to_faces: dict[tuple, list[int]] = {}
    for face_id, face in enumerate(faces_list):
        exp = TopExp_Explorer(face, TopAbs_EDGE)
        while exp.More():
            edge = TopoDS.Edge_s(exp.Current())
            key = _edge_key(edge)
            if key not in edge_to_faces:
                edge_to_faces[key] = []
            if face_id not in edge_to_faces[key]:
                edge_to_faces[key].append(face_id)
            exp.Next()

    G = nx.Graph()
    for fid in range(len(faces_list)):
        G.add_node(fid,
                   surface_type=_surface_type_for_face(faces_list[fid]),
                   flag=False)

    pair_data: dict[tuple[int, int], int] = {}
    for _ek, fids in edge_to_faces.items():
        for ii in range(len(fids)):
            for jj in range(ii + 1, len(fids)):
                i, j = fids[ii], fids[jj]
                if i > j:
                    i, j = j, i
                key = (i, j)
                pair_data[key] = pair_data.get(key, 0) + 1

    for (i, j), count in pair_data.items():
        G.add_edge(i, j, shared_edge_count=count)

    return G


def compute_face_normal(face: TopoDS_Face) -> tuple[float, float, float]:
    """Compute one representative outward normal for a face.

    Returns (nx, ny, nz). If the normal cannot be defined, returns NaNs.
    """
    adaptor = BRepAdaptor_Surface(face, False)
    umin, umax, vmin, vmax = BRepTools.UVBounds_s(face)
    u_mid = 0.5 * (umin + umax)
    v_mid = 0.5 * (vmin + vmax)
    du = (umax - umin) / 4.0
    dv = (vmax - vmin) / 4.0
    uv_candidates = [
        (u_mid, v_mid),
        (u_mid - du, v_mid),
        (u_mid + du, v_mid),
        (u_mid, v_mid - dv),
        (u_mid, v_mid + dv),
        (u_mid - du, v_mid - dv),
        (u_mid + du, v_mid + dv),
    ]
    for u, v in uv_candidates:
        props = BRepLProp_SLProps(adaptor, u, v, 1, 1e-6)
        if not props.IsNormalDefined():
            continue
        n = props.Normal()
        if face.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
            n.Reverse()
        return (n.X(), n.Y(), n.Z())

    return (float("nan"), float("nan"), float("nan"))


def compute_face_attributes(face: TopoDS_Face) -> dict:
    adaptor = BRepAdaptor_Surface(face, True)
    surface_type = SURFACE_TYPE_MAP.get(adaptor.GetType(), f"Other({adaptor.GetType()})")

    gprop = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, gprop)
    area = gprop.Mass()
    centre = gprop.CentreOfMass()

    return {
        "surface_type": surface_type,
        "area": area,
        "centroid": (centre.X(), centre.Y(), centre.Z()),
        "normal": compute_face_normal(face),
    }


def compute_angle_between_faces(face1: TopoDS_Face, face2: TopoDS_Face) -> float:
    """Return the angle in degrees between the outward normals of two adjacent faces,
    evaluated at the midpoint of their shared edge.

    Result range [0, 180]:
        0   -> faces are tangent (normals point the same way)
        90  -> faces are perpendicular
        180 -> normals point in opposite directions
    """
    shared_edge = _find_shared_edge(face1, face2)
    if shared_edge is None:
        return float("nan")
    curve = BRepAdaptor_Curve(shared_edge)
    u_mid = 0.5 * (curve.FirstParameter() + curve.LastParameter())
    mid_point = curve.Value(u_mid)
    n1 = _face_normal_at_point(face1, mid_point)
    n2 = _face_normal_at_point(face2, mid_point)
    return math.degrees(n1.Angle(n2))


def attach_edge_angles(G: nx.Graph, faces_list: list) -> None:
    """Compute the dihedral angle for every edge in G and store it as angle_deg."""
    for u, v in G.edges():
        try:
            angle = compute_angle_between_faces(faces_list[u], faces_list[v])
        except ValueError:
            angle = float("nan")
        G.edges[u, v]["angle_deg"] = angle


def attach_face_attributes(G: nx.Graph, faces_list: list) -> None:
    for face_id, face in enumerate(faces_list):
        if not G.has_node(face_id):
            continue
        attrs = compute_face_attributes(face)
        for k, v in attrs.items():
            G.nodes[face_id][k] = v
