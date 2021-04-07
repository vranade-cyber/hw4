import Pkg; Pkg.activate(joinpath(@__DIR__,".."))
using GeometryBasics
using CoordinateTransformations, Rotations
using RobotDynamics
using Colors
using StaticArrays 
using MeshCat
using Blink
using LinearAlgebra

struct SimpleWalker <: AbstractModel end
RobotDynamics.state_dim(::SimpleWalker) = 8
RobotDynamics.control_dim(::SimpleWalker) = 4

function set_mesh!(vis, model::SimpleWalker)
    l,w,h = 0.4,0.4,0.6
    body = HyperRectangle(Vec(-l/2,-w/2,0), Vec(l,w,h)) 
    body = Sphere(Point3f0(0,0,0), 7l/16)
    setobject!(vis["robot"]["torso"]["body"], body, MeshPhongMaterial(color=colorant"gray"))
    axle = Cylinder(Point3f0(0,0,0), Point3f0(0,w/2,0), 0.03f0)
    setobject!(vis["robot"]["torso"]["Laxle"], axle, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["robot"]["torso"]["Raxle"], axle, MeshPhongMaterial(color=colorant"black"))
    settransform!(vis["robot"]["torso"]["Laxle"], Translation(0,+l/4,0))
    settransform!(vis["robot"]["torso"]["Raxle"], Translation(0,-3l/4,0))

    foot = HyperSphere(Point3f0(0,0,0f0), 0.05f0)
    Lfoot = setobject!(vis["robot"]["Lfoot"]["geom"], foot, MeshPhongMaterial(color=colorant"firebrick"))
    setobject!(vis["robot"]["Rfoot"]["geom"], foot, MeshPhongMaterial(color=colorant"firebrick"))
    settransform!(vis["robot"]["Lfoot"]["geom"], Translation(0,+l/2,0))
    settransform!(vis["robot"]["Rfoot"]["geom"], Translation(0,-l/2,0))

    Lleg = Cylinder(Point3f0(0,+l/2,0), Point3f0(0,+l/2,1), 0.03f0)
    Rleg = Cylinder(Point3f0(0,-l/2,0), Point3f0(0,-l/2,1), 0.03f0)
    setobject!(vis["robot"]["torso"]["Lleg"]["geom"], Lleg, MeshPhongMaterial(color=colorant=colorant"green"))
    setobject!(vis["robot"]["torso"]["Rleg"]["geom"], Rleg, MeshPhongMaterial(color=colorant=colorant"green"))
    # settransform!(vis["robot"]["torso"]["Lleg"]["geom"], Translation(0,+l/2,0))
    # settransform!(vis["robot"]["torso"]["Rleg"]["geom"], Translation(0,-l/2,0))

    return Lfoot
end
function visualize!(vis, model::SimpleWalker, x::StaticVector)
    xb,yb = x[1],x[2]
    xl,yl = x[3],x[4]
    xr,yr = x[5],x[6]
    settransform!(vis["robot"]["torso"], Translation(xb,0,yb))
    settransform!(vis["robot"]["Lfoot"], Translation(xl,0,yl))
    settransform!(vis["robot"]["Rfoot"], Translation(xr,0,yr))

    Llen = norm(SA[xl-xb, yl-yb])
    Rlen = norm(SA[xr-xb, yr-yb])
    θl = atan(xl-xb, yl-yb)
    θr = atan(xr-xb, yr-yb)
    settransform!(vis["robot"]["torso"]["Lleg"], LinearMap(RotY(θl)))
    settransform!(vis["robot"]["torso"]["Rleg"], LinearMap(RotY(θr)))

    settransform!(vis["robot"]["torso"]["Lleg"]["geom"], LinearMap(Diagonal(SA[1,1,Llen])))
    settransform!(vis["robot"]["torso"]["Rleg"]["geom"], LinearMap(Diagonal(SA[1,1,Rlen])))
end

## Create visualizer window
vis = Visualizer()
open(vis, Blink.Window())

## Set up model
model = SimpleWalker()
x = SA[0,1, -0.5,0, 0.2,0.5]
delete!(vis)
foot = set_mesh!(vis, model)
visualize!(vis, model, x)