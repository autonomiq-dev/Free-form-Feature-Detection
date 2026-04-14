import trimesh

# (stock - part) ∩ extrusion
def build_feature_removal_volume(extrusion, stock):
    feature_mesh = trimesh.load(extrusion)
    stock_mesh = trimesh.load(stock)
    # part_mesh = trimesh.load(part)
    # to_be_removed = stock_mesh.difference(part_mesh)
    result = feature_mesh.intersection(stock_mesh)
    out_path = "feature_removal_volume.stl"
    result.export(out_path)
    return out_path

# def Stock_Part_Difference(stock, part):
#     stock_mesh = trimesh.load(stock)
#     part_mesh = trimesh.load(part)
#     to_be_removed = stock_mesh.difference(part_mesh)
#     out_path = "removal_volume.stl"
#     to_be_removed.export(out_path)
#     return out_path

def Removal_Prism_Intersection(removal_prism, removal_volume):
    removal_mesh = trimesh.load(removal_prism)
    removal_volume_mesh = trimesh.load(removal_volume)
    result = removal_mesh.intersection(removal_volume_mesh)
    out_path = "feature_removal_volume.stl"
    result.export(out_path)
    return out_path

def main():
    stock = "stock_mesh.stl"
    # part = "part_mesh.stl"
    removal_prism = "removal_mesh.stl"
    # result = Stock_Part_Difference(stock, part)
    removal_mesh = Removal_Prism_Intersection(removal_prism, stock) #ERROR HERE


if __name__ == "__main__":
    main()

