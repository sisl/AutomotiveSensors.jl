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
@with_kw struct GaussianSensor{P <: AbstractNoiseModel, V <: AbstractNoiseModel, R<:AbstractRNG} <: AbstractSensor
    pos_noise::P = LinearNoise()
    vel_noise::V = LinearNoise()
    false_positive_rate::Float64 = 0.1
    false_negative_rate::Float64 = 0.1
    rng::R = Random.GLOBAL_RNG
end


function measure(sensor::GaussianSensor, ego::Entity, scene::EntityScene{S,D,I}, roadway::Roadway, obstacles::Vector{ConvexPolygon}) where {S,D,I}
    obs = Vector{Entity{VehicleState, D, I}}()
    sizehint!(obs, 10) #XXX
    for veh in scene
        if veh.id == ego.id
            continue
        end
        zveh = measure(sensor, ego, veh, roadway, obstacles, sensor.rng)
        if zveh != nothing 
            push!(obs, Entity(zveh, veh.def, veh.id))
        end
    end
    if isempty(obs)
        false_car = car_false_positive(sensor.false_positive_rate, roadway, scene, obstacles, sensor.rng)
        false_car == nothing ? nothing : push!(obs, false_car)
        false_ped = ped_false_positive(sensor.false_positive_rate, roadway, scene, obstacles, sensor.rng)
        false_ped == nothing ? nothing : push!(obs, false_ped)
    end
    return obs
end

function measure(sensor::GaussianSensor, ego::Entity, veh::Entity, roadway::Roadway, obstacles::Vector{ConvexPolygon}, rng::AbstractRNG)
    occluded = occlusion_checker(ego.state, veh.state, obstacles) 
    FN = rand(rng) < sensor.false_negative_rate
    if occluded || FN
        return nothing 
    else
        dist = sqrt((posg(ego.state).x - posg(veh.state).x)^2 + (posg(ego.state).y - posg(veh.state).y)^2)
        pos_std = noise(sensor.pos_noise, dist)
        vel_std = noise(sensor.vel_noise, dist)
        z_pos = VecSE2(posg(veh.state).x + pos_std*randn(rng),
                       posg(veh.state).y + pos_std*randn(rng),
                       posg(veh.state).θ)
        z_v = veh.state.v + vel_std*randn(rng)
        return VehicleState(z_pos, roadway, z_v)
    end
end


function obs_weight(sensor::GaussianSensor, ego::VehicleState, obs::Union{Nothing, VehicleState}, veh::Union{Nothing, VehicleState}, obstacles::Vector{ConvexPolygon}) 
    weight = 1.0
    visible = veh != nothing && !occlusion_checker(ego, veh, obstacles) 
    if !visible
        if obs == nothing
            return weight*(1. - sensor.false_positive_rate)
        else
            return weight*sensor.false_positive_rate # should handle false positive
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

@with_kw struct GaussianSensorOverlay{E<:Entity}
    sensor::GaussianSensor = GaussianSensor()
    o::Vector{E} = Vector{E}()
    color::Colorant = RGBA(0.976, 0.592, 0.122, 0.7) # orange
end

function AutomotiveVisualization.add_renderable!(rendermodel::RenderModel, overlay::GaussianSensorOverlay, scene::Scene, roadway::R) where R
    for veh in overlay.o 
        p = posg(veh.state)
        # add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, veh.def.length, veh.def.width, overlay.color))
        add_instruction!(rendermodel, render_vehicle, 
                            (p.x, p.y, p.θ, 
                            veh.def.length, veh.def.width, overlay.color,ARGB(1.,1.,1.,0.),ARGB(1.,1.,1.,0.)))
    end
    return rendermodel
end
