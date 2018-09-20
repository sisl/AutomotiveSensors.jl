module AutomotiveSensors

using AutomotiveDrivingModels
using AutoViz
using Parameters
using StaticArrays

abstract type AbstractSensor end
abstract type AbstractNoiseModel end

export 
    AbstractNoiseModel,
    LinearNoise

include("noise_model.jl")

export 
    AbstractSensor,
    measure,
    GaussianSensor,
    GaussianSensorOverlay,
    occlusion_checker,
    OcclusionOverlay

include("gaussian_sensor.jl")
include("false_positive.jl")
include("occlusion_checker.jl")
include("rendering.jl")

end