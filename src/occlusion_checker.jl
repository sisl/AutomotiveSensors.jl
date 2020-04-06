"""
    occlusion_checker(veh_a, veh_b, obs::ConvexPolygon)

returns true if vehicle b is occluded by `obs` from vehicle a, false otherwise. 
Occlusion checking relies on ray tracing and a the parallel axis theorem to check for collision between the ray and the obstacle.

    
"""
function occlusion_checker(veh_a, veh_b, obs::ConvexPolygon)
    line = @SMatrix [posg(veh_a).x posg(veh_a).y;
                     posg(veh_b).x posg(veh_b).y]
    pts = obs.pts
    obstacle = @SMatrix [pts[1].x pts[1].y;
                         pts[2].x pts[2].y;
                         pts[3].x pts[3].y;
                         pts[4].x pts[4].y]
                                      
    return AutomotiveSimulator.overlap(line, obstacle)
end

function occlusion_checker(veh_a, veh_b, veh_b_def::VehicleDef)
    ray = posg(veh_b) - posg(veh_a)
    line = @SMatrix [ray.x ray.y]
    obstacle = polygon(veh_b, veh_b_def)
    return AutomotiveSimulator.overlap(line, obstacle)
end

function occlusion_checker(veh_a, veh_b, obstacles::Vector{ConvexPolygon})
    for obs in obstacles 
        if occlusion_checker(veh_a, veh_b, obs)
            return true 
        end
    end
    return false
end

## For rendering
@with_kw struct OcclusionOverlay
    obstacles::Vector{ConvexPolygon} = ConvexPolygon[]
    egoid::Int64 = 1
    visible_color::Colorant = COLOR_CAR_EGO
    occluded_color::Colorant = RGB(1.0, 0., 0.)
end

function AutomotiveVisualization.add_renderable!(rendermodel::RenderModel, overlay::OcclusionOverlay, scene::Scene, roadway::R) where R
    ego = scene[findfirst(overlay.egoid, scene)]
    #Display ray
    for veh in scene 
        if veh.id == overlay.egoid
            continue
        end
        line_pts = [posg(veh.state).x posg(veh.state).y; posg(ego.state).x posg(ego.state).y]'
        if occlusion_checker(ego.state, veh.state, overlay.obstacles)
            add_instruction!( rendermodel, render_line, (line_pts, overlay.occluded_color, 0.1))
        else
            add_instruction!( rendermodel, render_line, (line_pts, overlay.visible_color, 0.1))
        end
    end
end
