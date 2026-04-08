from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeWire
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from graph import _find_shared_edge


def compute_removal_volume(stock_shape, part_shape):
    cut = BRepAlgoAPI_Cut(stock_shape, part_shape)
    cut.Build()
    if not cut.IsDone():
        raise RuntimeError("Boolean cut failed")
    return cut.Shape()


def get_feature_boundary_edges(G, faces_list, feature_face_ids):
    feature_set = set(feature_face_ids)
    seen = set()
    boundary_edges = []
    for fid in feature_set:
        for nbr in G.neighbors(fid):
            if nbr not in feature_set:
                key = (min(fid, nbr), max(fid, nbr))
                if key in seen:
                    continue
                seen.add(key)
                edge = _find_shared_edge(faces_list[fid], faces_list[nbr])
                if edge is not None:
                    boundary_edges.append(edge)
    return boundary_edges


def clip_to_feature_footprint(removal_shape, boundary_edges, direction=(0, 0, 1), length=200):
    wire_builder = BRepBuilderAPI_MakeWire()
    for edge in boundary_edges:
        wire_builder.Add(edge)
    if not wire_builder.IsDone():
        raise RuntimeError("Wire construction from boundary edges failed")
    wire = wire_builder.Wire()

    vec = gp_Vec(*direction)
    vec.Multiply(length)
    fence = BRepPrimAPI_MakePrism(wire, vec).Shape()

    clipped = BRepAlgoAPI_Common(removal_shape, fence)
    clipped.Build()
    if not clipped.IsDone():
        raise RuntimeError("Boolean common (clipping) failed")
    return clipped.Shape()
