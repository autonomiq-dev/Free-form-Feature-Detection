from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCP.gp import gp_Vec
from OCP.TopoDS import TopoDS_Shell


def extrude_feature_patch(feature_faces, direction=(0,0,1), length=200):

    sewing = BRepBuilderAPI_Sewing()
    for f in feature_faces:
        sewing.Add(f)

    sewing.Perform()
    shell = sewing.SewedShape()

    vec = gp_Vec(direction[0], direction[1], direction[2])
    vec.Multiply(length)

    prism = BRepPrimAPI_MakePrism(shell, vec).Shape()

    return prism

