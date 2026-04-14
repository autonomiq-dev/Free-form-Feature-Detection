import math
from collections import deque
import networkx as nx
from typing import Set



from geometry_utils import *
from graph import *
from extrude_feature import *

def grow_region(G: nx.Graph, seed_face_id: int, angle_threshold: float = 40.0) -> Set[int]:

    def is_plane(node_id: int) -> bool:
        return G.nodes[node_id].get("surface_type") == "Plane"

    region: Set[int] = set()
    queue: deque[int] = deque([seed_face_id])
    while queue:
        current = queue.popleft()
        if current in region:
            continue
        if is_plane(current):
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



def main() -> int:
    parser = argparse.ArgumentParser(
        description="Detect free-form feature, extrude it, and intersect with stock removal volume."
    )
    parser.add_argument("--part", required=True, help="Path to finished part STEP file")
    parser.add_argument("--stock", required=True, help="Path to stock STEP file")
    parser.add_argument(
        "--dir",
        required=True,
        nargs=3,
        type=float,
        metavar=("DX", "DY", "DZ"),
        help="Machining direction vector, e.g. --direction 0 0 1",
    )
    parser.add_argument(
        "--out",
        default="feature_removal.step",
        help="Output STEP file for feature removal volume",
    )
    args = parser.parse_args()

    step_path = args.part
    stock_path = args.stock
    direction = tuple(args.dir)

    # Load part and stock shapes
    shape, faces_list = read_step_from_user(step_path)
    stock_shape = load_step(stock_path)
    
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

    # # visualize selected feature faces
    # visualize_faces_on_mesh(shape, faces_list, feature_faces)

    # get stock's bounding box to set extrusion length
    extrusion_length = get_extrusion_length(stock_shape, direction)

    # extrude feature faces
    removal_prism = extrude_feature_patch(
        detected_patch_faces,
        direction,   
        length=extrusion_length,          # large enough to reach stock top
    )

    # # Visualize the extrusion
    # feature_mesh = build_mesh_for_shape(removal_prism, mesh_deflection=0.001)
    # plotter = pv.Plotter()
    # plotter.add_mesh(feature_mesh, color="gray")
    # plotter.show()


    # build_feature_removal_volume(removal_mesh, stock_mesh, part_mesh) #ERROR HERE if convert to mesh before boolean operations

    # Compute the true feature removal volume BREP:
    # (stock - part) ∩ removal_prism
    feature_removal = compute_feature_removal_volume(
        stock_shape=stock_shape,
        part_shape=shape,
        extrusion=removal_prism,
    )

    # Export intersected feature removal to STEP
    output_step = args.out
    save_shape_to_step(feature_removal, output_step)
    print(f"Feature removal volume saved to STEP file: {output_step}")

    # visualize feature removal volume
    visualize_feature_removal_volume(removal_prism, feature_removal, shape)

    return 0

if __name__ == "__main__":
    raise SystemExit(main())


    # bbox = Bnd_Box()
    # BRepBndLib.Add_s(stock_shape, bbox)

    # xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    # return {
    #     "x": (xmin, xmax),
    #     "y": (ymin, ymax),
    #     "z": (zmin, zmax),
    # }

   



    # # build mesh box from feature XY extents + stock Zmax, saved as STL
    # feature_box = build_feature_box_from_bbox(feature_bbox, stock_shape, output_path="feature_bbox_solid.stl")


    # # Build feature removal volume
    # build_material_removal_mesh(feature_box, feature_faces, faces_list=faces_list, output_path="feature_removal_volume.stl")

        # # Compute the true feature removal volume Mesh:
    # # (stock - part) ∩ removal_prism
    # part_mesh = step_to_stl(step_path, stl_path="part_mesh.stl")
    # stock_mesh = step_to_stl(stock_path, stl_path="stock_mesh.stl")
    # removal_mesh = step_to_stl(removal_prism, stl_path="removal_mesh.stl")


    # # visualize feature faces
    # visualize_faces_on_mesh(shape, faces_list, feature_faces)
    
    # # Visualize the intersected removal volume
    # feature_mesh = build_mesh_for_shape(feature_removal, mesh_deflection=0.001)
    # if feature_mesh is not None:
    #     plotter = pv.Plotter()
    #     plotter.add_mesh(feature_mesh, color="gray")
    #     plotter.show()
