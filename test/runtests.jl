using AutomotiveSensors
using AutomotiveSimulator
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

    ego = Entity(VehicleState(VecSE2(20.0, 0.0, 0.0), 5.0), VehicleDef(), 1)
    veh1 = Entity(VehicleState(VecSE2(30.0, 0.0, 0.0), 5.0), VehicleDef(), 2)
    veh2 = Entity(VehicleState(VecSE2(10.0, 0.0, 0.0), 5.0), VehicleDef(), 3)
    scene = Scene([ego, veh1, veh2])

    obs = measure(sensor, ego, scene, roadway, ConvexPolygon[])

    @test obs[1] == veh1 
    @test obs[2] == veh2
end

@testset "gaussian sensor" begin 
    sensor = GaussianSensor(false_positive_rate=0.0, false_negative_rate=0.0) 
    roadway = gen_straight_roadway(1, 100.0)

    ego = Entity(VehicleState(VecSE2(20.0, 0.0, 0.0), 5.0), VehicleDef(), 1)
    veh1 = Entity(VehicleState(VecSE2(30.0, 0.0, 0.0), 5.0), VehicleDef(), 2)
    veh2 = Entity(VehicleState(VecSE2(10.0, 0.0, 0.0), 5.0), VehicleDef(), 3)
    scene = Scene([ego, veh1, veh2])

    obs = measure(sensor, ego, scene, roadway, ConvexPolygon[])

    @test length(obs) == 2

    @test AutomotiveSensors.obs_weight(sensor, ego.state, obs[1].state, veh1.state, ConvexPolygon[]) > 0.0
end
