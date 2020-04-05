## Sensor model that returns the exact state of the other vehicles 

"""
Sensor that returns the exact state of the other vehicles 
"""
struct PerfectSensor <: AbstractSensor end

function measure(sensor::PerfectSensor, ego::Entity, scene::Scene, roadway::Roadway, obstacles::Vector{ConvexPolygon})
    obs = Vector{eltype(scene)}()
    sizehint!(obs, length(scene))
    for veh in scene
        if veh.id == ego.id
            continue
        end
        push!(obs, veh)
    end
    return obs
end
