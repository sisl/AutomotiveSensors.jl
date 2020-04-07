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

@testset "ConstantNoise model" begin
    noise_model = ConstantNoise(noise=1.0)
    @test noise_model.noise == 1.0
    @test AutomotiveSensors.noise(noise_model)          == 1.0
    @test AutomotiveSensors.noise(noise_model, 0.0)     == 1.0
    @test AutomotiveSensors.noise(noise_model, 1000.0)  == 1.0

    @testset begin
        for i in 1:1e4
            @test AutomotiveSensors.noise(noise_model, 1e9*rand()) == 1.0
        end
    end
end

@testset "LinearNoise model" begin
    noise_model = LinearNoise(close_range=1.0, min_noise=0.5, increase_rate=0.5)
    @test AutomotiveSensors.noise(noise_model, 0.0) == 0.5
    @test AutomotiveSensors.noise(noise_model, 1.0) == 0.5
    @test AutomotiveSensors.noise(noise_model, 2.0) == 1.0

    # zero close range
    noise_model = LinearNoise(close_range=0.0, min_noise=0.5, increase_rate=0.5)
    @test AutomotiveSensors.noise(noise_model, 0.0) == 0.5
    @test AutomotiveSensors.noise(noise_model, 1.0) == 1.0
    @test AutomotiveSensors.noise(noise_model, 2.0) == 1.5

    # zero min noise
    noise_model = LinearNoise(close_range=1.0, min_noise=0.0, increase_rate=0.5)
    @test AutomotiveSensors.noise(noise_model, 0.0) == 0.0
    @test AutomotiveSensors.noise(noise_model, 1.0) == 0.0
    @test AutomotiveSensors.noise(noise_model, 2.0) == 0.5

    # zero increase rate (ConstantNoise)
    noise_model = LinearNoise(close_range=1.0, min_noise=0.5, increase_rate=0.0)
    @test AutomotiveSensors.noise(noise_model, 0.0) == 0.5
    @test AutomotiveSensors.noise(noise_model, 1.0) == 0.5
    @test AutomotiveSensors.noise(noise_model, 2.0) == 0.5

    const_noise_model = ConstantNoise(noise=0.5)
    @test AutomotiveSensors.noise(noise_model, 2.0) == AutomotiveSensors.noise(const_noise_model, 2.0)

    #property access
    @test noise_model.close_range   == 1.0
    @test noise_model.min_noise     == 0.5
    @test noise_model.increase_rate == 0.0
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
