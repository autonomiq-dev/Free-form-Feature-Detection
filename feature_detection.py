import math
from collections import deque
import networkx as nx
from typing import Set

from OCP.ShapeFix import ShapeFix_Shape
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing

from geometry_utils import *
from graph import *

def _build_selected_faces_shape(selected_faces):
    if not selected_faces:
        raise ValueError("No faces selected for export.")

    # Sew selected faces into one coherent patch.
    sew = BRepBuilderAPI_Sewing(1e-3)
    for face in selected_faces:
        sew.Add(face)
    sew.Perform()
    patch = sew.SewedShape()

    # Heal the sewn patch so STEP/STL export has a cleaner, more valid shape.
    fixer = ShapeFix_Shape(patch)
    fixer.Perform()
    healed = fixer.Shape()

    if healed.IsNull():
        raise RuntimeError("Selected-face patch became null during healing.")

    if not BRepCheck_Analyzer(healed).IsValid():
        print("Warning: selected-face patch is not fully valid, exporting anyway.")

    return healed

def grow_region(
    G: nx.Graph,
    seed_face_id: int,
    angle_threshold: float = 40.0,
    machining_direction: tuple = (0.0, 0.0, 1.0),
    parallel_angle_tol: float = 5.0,
) -> Set[int]:

    def is_plane(node_id: int) -> bool:
        return G.nodes[node_id].get("surface_type") == "Plane"

    def is_parallel_to_direction(node_id: int) -> bool:
        normal = G.nodes[node_id].get("normal")
        if normal is None:
            return False
        nx_, ny_, nz_ = normal
        dx, dy, dz = machining_direction

        n_mag = math.sqrt(nx_**2 + ny_**2 + nz_**2)
        d_mag = math.sqrt(dx**2 + dy**2 + dz**2)
        if n_mag < 1e-10 or d_mag < 1e-10:
            return False

        dot = (nx_ * dx + ny_ * dy + nz_ * dz) / (n_mag * d_mag)
        dot = max(-1.0, min(1.0, dot))          # clamp for floating-point safety
        angle_deg = math.degrees(math.acos(abs(dot)))   # always in [0°, 90°]

        return angle_deg <= parallel_angle_tol


    region: Set[int] = set()
    queue: deque[int] = deque([seed_face_id])
    while queue:
        current = queue.popleft()
        if current in region:
            continue

        if is_plane(current):
            if not is_parallel_to_direction(current):
                continue
            neighbors = list(G.neighbors(current))
            if any(is_plane(nbr) for nbr in neighbors) or not neighbors:
                continue
        region.add(current)
        G.nodes[current]["flag"] = True

        for nbr in G.neighbors(current):
            if nbr in region:
                continue

            angle = G.edges[current, nbr].get("angle_deg", 0.0)
            if math.isnan(angle) or angle > angle_threshold:
                continue
            queue.append(nbr)
    
    return region

# add test.py
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Detect free-form feature, extrude it, and intersect with stock removal volume."
    )
    parser.add_argument("--part", required=True, help="Path to finished part STEP file")
    args = parser.parse_args()

    step_path = args.part

    # Load part and stock shapes
    shape, faces_list = read_step_from_user(step_path)
    
    # Build face adjacency graph
    G = build_face_adjacency(faces_list)    
    attach_face_attributes(G, faces_list)
    attach_edge_angles(G, faces_list)

    # click-point and select a face
    picked_face = pick_brep_face(shape, faces_list)
    face_id = faces_list.index(picked_face)
    G.nodes[face_id]["flag"] = True

    # Find free-form feature
    feature_faces = grow_region(G, face_id)
    detected_patch_faces = [faces_list[i] for i in feature_faces]

    # visualize selected feature faces
    visualize_faces_on_mesh(shape, faces_list, feature_faces)

    # export selected feature patch as valid BREP/mesh where possible
    selected_faces_shape = _build_selected_faces_shape(detected_patch_faces)
    print(type(selected_faces_shape))

    save_shape_to_step(selected_faces_shape, "selected_feature_faces.step")
    save_shape_to_stl(selected_faces_shape, "selected_feature_faces.stl")
    print("Exported selected faces to selected_feature_faces.step/.stl")


    return 0

if __name__ == "__main__":
    raise SystemExit(main())



