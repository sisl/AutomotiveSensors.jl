## Sensor model that returns position and velocities of other traffic participants
# Gaussian Noise
# FP/FN 
# Late detection?
# Position noise depends on distance

"""
Sensor model that returns position and velocities of other traffic participants
Gaussian Noise
FP/FN 
Late detection?
Position noise depends on distance
"""
@with_kw struct GaussianSensor{P <: AbstractNoiseModel, V <: AbstractNoiseModel} <: AbstractSensor
    pos_noise::P = LinearNoise()
    vel_noise::V = LinearNoise()
    false_positive_rate::Float64 = 0.1
    false_negative_rate::Float64 = 0.1
    rng::AbstractRNG = MersenneTwister(1)
end


function measure(sensor::GaussianSensor, ego::Vehicle, scene::Scene, roadway::Roadway, obstacles::Vector{ConvexPolygon})
    obs = Vector{Vehicle}()
    sizehint!(obs, 10) #XXX
    for veh in scene
        if veh.id == ego.id
            continue
        end
        zveh = measure(sensor, ego, veh, roadway, obstacles, sensor.rng)
        if zveh != nothing 
            push!(obs, Vehicle(zveh, veh.def, veh.id))
        end
    end
    if isempty(obs)
        false_car = car_false_positive(sensor.false_positive_rate, roadway, scene, obstacles, sensor.rng)
        false_car == nothing ? nothing : push!(obs, false_car)
        false_ped = ped_false_positive(sensor.false_positive_rate, roadway, scene, obstacles, sensor.rng)
        false_ped == nothing ? nothing: push!(obs, false_ped)
    end
    return obs
end

function measure(sensor::GaussianSensor, ego::Vehicle, veh::Vehicle, roadway::Roadway, obstacles::Vector{ConvexPolygon}, rng::AbstractRNG)
    occluded = occlusion_checker(ego.state, veh.state, obstacles) 
    FN = rand(rng) < sensor.false_negative_rate
    if occluded || FN
        return nothing 
    else
        dist = sqrt((ego.state.posG.x - veh.state.posG.x)^2 + (ego.state.posG.y - veh.state.posG.y)^2)
        pos_std = noise(sensor.pos_noise, dist)
        vel_std = noise(sensor.vel_noise, dist)
        z_pos = VecSE2(veh.state.posG.x + pos_std*randn(rng),
                       veh.state.posG.y + pos_std*randn(rng),
                       veh.state.posG.θ)
        z_v = veh.state.v + vel_std*randn(rng)
        return VehicleState(z_pos, roadway, z_v)
    end
end


function obs_weight(sensor::GaussianSensor, ego::VehicleState, obs::Union{Void, VehicleState}, veh::Union{Void, VehicleState}, obstacles::Vector{ConvexPolygon}) 
    weight = 1.0
    visible = veh != nothing && !occlusion_checker(ego, veh, obstacles) 
    if !visible
        if obs == nothing
            return weight
        else
            return 0. # should handle false positive
        end
    else
        if obs == nothing
            return sensor.false_positive_rate
        else
            dist = sqrt((ego.posG.x - veh.posG.x)^2 + (ego.posG.y - veh.posG.y)^2)
            pos_std = noise(sensor.pos_noise, dist)
            vel_std = noise(sensor.vel_noise, dist)
            weight *= (1 - sensor.false_positive_rate)
            weight *= 1/(sqrt(2*pi*pos_std*vel_std))*exp(-1/pos_std*(veh.posG.x - obs.posG.x)^2 -1/pos_std*(veh.posG.y - obs.posG.y)^2 - 1/vel_std*(veh.v - obs.v)^2)
            return weight
        end
    end
end

@with_kw struct GaussianSensorOverlay <: SceneOverlay
    sensor::GaussianSensor = GaussianSensor()
    o::Vector{Vehicle} = Vector{Vehicle}()
    color::Colorant = RGBA(0.976, 0.592, 0.122, 0.7) # orange
end

function AutoViz.render!(rendermodel::RenderModel, overlay::GaussianSensorOverlay, scene::Scene, roadway::R) where R
    for veh in overlay.o 
        p = veh.state.posG
        add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, veh.def.length, veh.def.width, overlay.color))
    end
    return rendermodel
end
