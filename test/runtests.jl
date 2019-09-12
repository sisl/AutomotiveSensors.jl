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