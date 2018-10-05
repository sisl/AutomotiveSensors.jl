module AutomotiveSensors

using AutomotiveDrivingModels
using AutoViz
using Parameters
using StaticArrays
using Random
using LinearAlgebra

export 
    AbstractSensor,
    measure

abstract type AbstractSensor end
abstract type AbstractNoiseModel end

export 
    occlusion_checker,
    OcclusionOverlay

include("occlusion_checker.jl")

export 
    AbstractNoiseModel,
    LinearNoise

include("noise_model.jl")

export
    GaussianSensor,
    GaussianSensorOverlay

include("gaussian_sensor.jl")
include("false_positive.jl")

export
    PerfectSensor

include("perfect_sensor.jl")

end