from extract_feature import *
from feature_detection import *
from geometry_utils import *
from graph import *


parser = argparse.ArgumentParser(
    description="Detect free-form feature, extrude it, and intersect with stock removal volume."
)
parser.add_argument("--part", required=True, help="Path to workpiece STEP file")
parser.add_argument("--stock", required=True, help="Path to stock STEP file") #TODO: change to STL file
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

# click-point and select a face which is a seed for free-form feature detection
picked_face = pick_brep_face(shape, faces_list)
face_id = faces_list.index(picked_face)
G.nodes[face_id]["flag"] = True

# Find free-form feature by growing region from seed face
feature_faces = grow_region(G, face_id)
detected_patch_faces = [faces_list[i] for i in feature_faces]
# get stock's bounding box to set extrusion length
extrusion_length = get_extrusion_length(stock_shape, direction)

# extrude feature faces
removal_prism = extrude_feature_patch(
    detected_patch_faces,
    direction=direction,
    length=extrusion_length,          # large enough to reach stock top
)

# # Visualize the extrusion if needed (turn it to a function for visualize extrusion mesh) 
# feature_mesh = build_mesh_for_shape(removal_prism, mesh_deflection=0.001)
# plotter = pv.Plotter()
# plotter.add_mesh(feature_mesh, color="gray")
# plotter.show()

# Compute the true feature removal volume BREP:
# stock ∩ removal_prism
feature_removal = compute_feature_removal_volume(stock_shape, removal_prism)

# Export feature removal volume to STEP
output_step = args.out
save_shape_to_step(feature_removal, output_step)
print(f"Feature removal volume saved to STEP file: {output_step}")

# visualize feature removal volume
visualize_feature_removal_volume(removal_prism, feature_removal, shape)