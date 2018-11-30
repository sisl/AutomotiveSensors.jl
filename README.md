# AutomotiveSensors

[![Build Status](https://travis-ci.org/sisl/AutomotiveSensors.jl.svg?branch=master)](https://travis-ci.org/sisl/AutomotiveSensors.jl)
[![CodeCov](https://codecov.io/gh/sisl/AutomotiveSensors.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/sisl/AutomotiveSensors.jl)
[![Coveralls](https://coveralls.io/repos/github/sisl/AutomotiveSensors.jl/badge.svg?branch=master)](https://coveralls.io/github/sisl/AutomotiveSensors.jl?branch=master)

Sensor models to work with AutomotiveDrivingModels.jl

## Installation 

To install this package and its dependencies, run the following in the julia REPL.

```julia
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveDrivingModels.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoViz.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSensors.jl"))
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
