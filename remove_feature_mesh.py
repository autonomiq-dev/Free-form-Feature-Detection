import trimesh

def build_feature_removal_volume(feature_box, part):
    feature_box_mesh = trimesh.load(feature_box)
    part_mesh = trimesh.load(part)
    cut = feature_box_mesh.difference(part_mesh)
    return cut.export("feature_removal_volume.stl")


def main():
    feature_box = "feature_bbox_solid.stl"
    part = "Samples\\freecad_models\\case_2\\ff_case_2.stl"
    build_feature_removal_volume(feature_box, part)

if __name__ == "__main__":
    main()