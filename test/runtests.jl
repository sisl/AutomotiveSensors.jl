using AutomotiveSensors
using AutomotiveDrivingModels
using Test

@testset "occlusion checker" begin 
    obs = ConvexPolygon([VecE2(0.0, 0.0), VecE2(10.0, 0.0), VecE2(10.0, 10.0), VecE2(0.0, 10.0)])
    veh1 = VehicleState(VecSE2(-5,-5,0.0), 0.)
    veh2 = VehicleState(VecSE2(15,15,0.0), 0.0)
    veh3 = VehicleState(VecSE2(10.0, -5, 0.0), 0.)

    @test occlusion_checker(veh1, veh2, obs)
    @test !occlusion_checker(veh1, veh3, obs)
end

@testset "perfect sensor" begin 
    sensor = PerfectSensor() 
    roadway = gen_straight_roadway(1, 100.0)

    ego = Vehicle(VehicleState(VecSE2(20.0, 0.0, 0.0), 5.0), VehicleDef(), 1)
    veh1 = Vehicle(VehicleState(VecSE2(30.0, 0.0, 0.0), 5.0), VehicleDef(), 2)
    veh2 = Vehicle(VehicleState(VecSE2(10.0, 0.0, 0.0), 5.0), VehicleDef(), 3)
    scene = Scene([ego, veh1, veh2])

    obs = measure(sensor, ego, scene, roadway, ConvexPolygon[])

    @test obs[1] == veh1 
    @test obs[2] == veh2
end
