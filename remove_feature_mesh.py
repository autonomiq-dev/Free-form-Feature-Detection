
import trimesh
 
def main():
    extrude_path = "ff_case_1_extrusion.stl"
    stock_path = "ff_case_1_stock.stl"
    # Load the two meshes for boolean intersection
    extrusion = trimesh.load(extrude_path)
    stock = trimesh.load(stock_path)
 
    intersection = extrusion.intersection(stock)
 
    # Save the intersection result
    intersection.export("ff_case_1_intersection.stl")

 
if __name__ == "__main__":
    main()
 