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

include("Gaussian_sensor.jl")
include("occlusion_checker.jl")
include("rendering.jl")

end