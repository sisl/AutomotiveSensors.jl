function car_false_positive(false_positive_rate::Float64, roadway::Roadway, scene::Scene, obstacles::Vector{ConvexPolygon}, rng::AbstractRNG)
    FP = rand(rng) < false_positive_rate
    if FP
        return random_car(roadway, scene, rng) 
    else
        return nothing 
    end
end

function ped_false_positive(false_positive_rate::Float64, roadway::Roadway, scene::Scene, obstacles::Vector{ConvexPolygon}, rng::AbstractRNG)
    FP = rand(rng) < false_positive_rate
    if FP
        return random_pedestrian(roadway, scene, rng) 
    else
        return nothing 
    end
end


function random_pedestrian(roadway::Roadway, scene::Scene, rng::AbstractRNG, ped_max_speed::Float64=2.0, ped_def::VehicleDef= VehicleDef(AgentClass.PEDESTRIAN, 1., 1.) )
    crosswalk = rand(rng, get_crosswalks(roadway))

    # position along the crosswalk
    t0 = rand(rng, Uniform(-crosswalk.width/2, crosswalk.width/2))
    s0 = rand(rng, Uniform(0., crosswalk.curve[end].s))
    ϕ0 = rand(rng, [0.,float(π)])

    #velocity
    v0 = rand(rng, Uniform(0., ped_max_speed))
    posF = Frenet(crosswalk, s0, t0, ϕ0)

    ped_state = VehicleState(posF, roadway, v0);

    return Vehicle(ped_state, ped_def, 100+length(scene))
end

function random_car(roadway::Roadway, scene::Scene, rng::AbstractRNG, car_max_speed::Float64=2.0, car_def::VehicleDef=VehicleDef())
    lane = rand(rng, get_car_lanes(roadway))

    s0 = rand(rng, Uniform(0., lane.curve[end].s))
    v0 = rand(rng, Uniform(0., car_max_speed))

    car_state = VehicleState(Frenet(lane, s0), roadway, v0)

    return Vehicle(car_state, car_def, length(scene)+1)
end

function get_car_lanes(roadway::Roadway)
    lanes = Lane[]
    for i=1:length(roadway.segments)
        for lane in roadway.segments[i].lanes
            if !is_crosswalk(lane) && !(lane.tag ∈ [LaneTag(15,1), LaneTag(16, 1), LaneTag(6,1), LaneTag(13, 1), LaneTag(14,1)]) #XXX hack
                push!(lanes, lane)
            end
        end
    end
    return lanes
end

function get_crosswalks(roadway::Roadway)
    lanes = Lane[]
    for i=1:length(roadway.segments)
        for lane in roadway.segments[i].lanes
            if is_crosswalk(lane)
                push!(lanes, lane)
            end
        end
    end
    return lanes
end

is_crosswalk(lane::Lane) = lane.width > 3.0  #XXX hack