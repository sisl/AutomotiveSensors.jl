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
                                      
    return overlap(line, obstacle)
end

function occlusion_checker(veh_a::VehicleState, veh_b::VehicleState, veh_b_def::VehicleDef)
    ray = veh_b.posG - veh_a.posG
    line = @SMatrix [ray.x ray.y]
    obstacle = polygon(veh_b, veh_b_def)
    return AutomotivePOMDPs.overlap(line, obstacle)
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


#TODO Move to AutomotiveDrivingModels.jl
# Duplicate with AutomotivePOMDPs
### POLYGON INTERSECTION USING PARALLEL AXIS THEOREM

""" 
    overlap(poly_a::SMatrix{4, 2, Float64}, poly_b::SMatrix{4, 2, Float64})
Check if two convex polygons overlap, using the parallel axis theorem
a polygon is a nx2 matrix where n in the number of verteces
http://gamemath.com/2011/09/detecting-whether-two-convex-polygons-overlap/ 
  /!\\ edges needs to be ordered
"""
function overlap(poly_a::SMatrix{M, 2, Float64},
                poly_b::SMatrix{N, 2, Float64}) where {M,N}
  
    if find_separating_axis(poly_a, poly_b)
        return false
    end
    if find_separating_axis(poly_b, poly_a)
        return false
    end

    return true

end

""" 
    find_separating_axis(poly_a::SMatrix{4, 2, Float64}, poly_b::SMatrix{4, 2, Float64})

build a list of candidate separating axes from the edges of a
  /!\\ edges needs to be ordered
"""
function find_separating_axis(poly_a::SMatrix{M, 2, Float64},
                              poly_b::SMatrix{N, 2, Float64}) where {M, N}
    n_a = size(poly_a)[1]
    n_b = size(poly_b)[1]
    axis = zeros(2)
    @views previous_vertex = poly_a[end,:]
    for i=1:n_a # loop through all the edges n edges
        @views current_vertex = poly_a[i,:]
        # get edge vector
        edge = current_vertex - previous_vertex

        # rotate 90 degrees to get the separating axis
        axis[1] = edge[2]
        axis[2] = -edge[1]

        #  project polygons onto the axis
        a_min,a_max = polygon_projection(poly_a, axis)
        b_min,b_max = polygon_projection(poly_b, axis)

        # check separation
        if a_max < b_min
            return true#,current_vertex,previous_vertex,edge,a_max,b_min

        end
        if b_max < a_min
            return true#,current_vertex,previous_vertex,edge,a_max,b_min

        end

        @views previous_vertex = poly_a[i,:]
    end

    # no separation was found
    return false
end

"""
    polygon_projection(poly::SMatrix{4, 2, Float64}, axis::Vector{Float64})

return the projection interval for the polygon poly over the axis axis 
"""
function polygon_projection(poly::SMatrix{N, 2, Float64},
                            axis::Vector{Float64}) where {N}
        n_a = size(poly)[1]
        @inbounds @fastmath @views d1 = dot(poly[1,:],axis)
        @inbounds @fastmath @views d2 = dot(poly[2,:],axis)
        # initialize min and max
        if d1<d2
            out_min = d1
            out_max = d2
        else
            out_min = d2
            out_max = d1
        end

        for i=1:n_a
            @inbounds @fastmath @views d = dot(poly[i,:],axis)
            if d < out_min
                out_min = d
            elseif d > out_max
                out_max = d
            end
        end
        return out_min,out_max
end