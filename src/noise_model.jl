# some noise model (standard deviation)


"""
    LinearNoise 

The noise increase linearly with the distance to the object

#Fields 
- close_range: distance at which the error starts increasing 
- min_noise: minimum error
- increase_rate: noise dimension/meter (e.g. position noise increase by 1m every meter) 
"""
@with_kw struct LinearNoise <: AbstractNoiseModel
    close_range::Float64 = 2.0 
    min_noise::Float64 = 0.5
    increase_rate::Float64 = 0.1
end

function noise(model::LinearNoise, dist::Float64)
    return max(model.min_noise, model.min_noise + dist*model.increase_rate)
end


