# AutomotiveSensors

Sensor models to work with AutomotiveDrivingModels.jl

## Installation 

```julia
Pkg.clone("https://github.com/sisl/AutomotiveSensors.jl")
```

## Models 

- `GaussianSensor`: Simple sensor model that returns the state of other vehicle with Gaussian noise on position and velocity. 
It observes the heading perfectly, it is sensitive to occlusion and can return false positive and false negative.
The noise can be specified using an `AbstractNoiseModel`, the default is `LinearNoise`. 
- `PerfectSensor`: Sensor that returns the exact state of other vehicles. 

## Tools 

- `occlusion_checker`: Check whether a vehicle is occluded by an obstacle. 
It relies on tracing a ray between the two vehicle of interest and checking if the ray intersects with an obstacle. 
The rays can be visualized using the `OcclusionOverlay`. See [AutoViz.jl](https://github.com/sisl/AutoViz.jl) to learn more on how to use overlays. 
