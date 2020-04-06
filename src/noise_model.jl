"""
    ConstantNoise(; noise=0.5)

Noise model with noise that is constant with distance from the sensor.

#Fields
- noise: noise standard deviation
"""
@with_kw struct ConstantNoise <: AbstractNoiseModel
    noise::Float64 = 0.5
end

"""
    noise(model, dist)

Compute the sensor noise using `model` at distance `dist` from the sensor.
"""
function noise(model::ConstantNoise, dist::Float64)
    return model.noise
end

function noise(model::ConstantNoise) # optional no distance signature
    return model.noise
end


"""
    LinearNoise(; close_range=2.0, min_noise=0.5, increase_rate=0.1)

Noise model with noise that increases linearly with distance from the sensor.

#Fields
- close_range: distance at which the error starts increasing
- min_noise: minimum noise standard deviation
- increase_rate: rate of noise increase per distance (e.g. position noise increases by 1.0 m per m)
"""
@with_kw struct LinearNoise <: AbstractNoiseModel
    close_range::Float64    = 2.0
    min_noise::Float64      = 0.5
    increase_rate::Float64  = 0.1
end

function noise(model::LinearNoise, dist::Float64)
    return model.min_noise + max(0.0, model.increase_rate*(dist - model.close_range))
end
