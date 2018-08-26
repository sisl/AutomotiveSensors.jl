@with_kw struct OcclusionOverlay <: SceneOverlay
    obstacles::Vector{ConvexPolygon} = ConvexPolygon[]
    egoid::Int64 = 1
    visible_color::Colorant = COLOR_CAR_EGO
    occluded_color::Colorant = RGB(1.0, 0., 0.)
end

function AutoViz.render!(rendermodel::RenderModel, overlay::OcclusionOverlay, scene::Scene, roadway::R) where R
    ego = scene[findfirst(scene, overlay.egoid)]
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