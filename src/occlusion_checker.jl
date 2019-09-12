"""
returns true if vehicle b is occluded by `obs` from vehicle a, false otherwise. 
Occlusion checking relies on ray tracing and a the parallel axis theorem to check for collision between the ray and the obstacle.

    occlusion_checker(veh_a::VehicleState, veh_b::VehicleState, obs::ConvexPolygon)
"""
function occlusion_checker(veh_a::VehicleState, veh_b::VehicleState, obs::ConvexPolygon)
    line = @SMatrix [veh_a.posG.x veh_a.posG.y;
                     veh_b.posG.x veh_b.posG.y]
    pts = obs.pts
    obstacle = @SMatrix [pts[1].x pts[1].y;
                         pts[2].x pts[2].y;
                         pts[3].x pts[3].y;
                         pts[4].x pts[4].y]
                                      
    return AutomotiveDrivingModels.overlap(line, obstacle)
end

function occlusion_checker(veh_a::VehicleState, veh_b::VehicleState, veh_b_def::VehicleDef)
    ray = veh_b.posG - veh_a.posG
    line = @SMatrix [ray.x ray.y]
    obstacle = polygon(veh_b, veh_b_def)
    return AutomotiveDrivingModels.overlap(line, obstacle)
end

function occlusion_checker(veh_a::VehicleState, veh_b::VehicleState, obstacles::Vector{ConvexPolygon})
    for obs in obstacles 
        if occlusion_checker(veh_a, veh_b, obs)
            return true 
        end
    end
    return false
end

## For rendering
@with_kw struct OcclusionOverlay <: SceneOverlay
    obstacles::Vector{ConvexPolygon} = ConvexPolygon[]
    egoid::Int64 = 1
    visible_color::Colorant = COLOR_CAR_EGO
    occluded_color::Colorant = RGB(1.0, 0., 0.)
end

function AutoViz.render!(rendermodel::RenderModel, overlay::OcclusionOverlay, scene::Scene, roadway::R) where R
    ego = scene[findfirst(overlay.egoid, scene)]
    #Display ray
    for veh in scene 
        if veh.id == overlay.egoid
            continue
        end
        line_pts = [veh.state.posG.x veh.state.posG.y; ego.state.posG.x ego.state.posG.y]'
        if occlusion_checker(ego.state, veh.state, overlay.obstacles)
            add_instruction!( rendermodel, render_line, (line_pts, overlay.occluded_color, 0.1))
        else
            add_instruction!( rendermodel, render_line, (line_pts, overlay.visible_color, 0.1))
        end
    end
end