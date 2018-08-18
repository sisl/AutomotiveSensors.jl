module AutomotiveSensors

using AutomotiveDrivingModels
using AutoViz
using AutomotivePOMDPs
using Parameters

abstract type AbstractSensor end
abstract type AbstractNoiseModel end

export 
    AbstractNoiseModel,
    LinearNoise

include("noise_model.jl")

export 
    AbstractSensor,
    measure,
    NoisySensor,
    NoisySensorOverlay

include("noisy_sensor.jl")

end