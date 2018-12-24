## unfair-dice-simulation

This project intends to simulate the unfair dice throw outcome, and to visualize its individual trajectories.

It computes the trajectory with physics-based precision, solving equations of motion in the gravitational field,
equations of rotation of the rigid body. It will use a surface collision model that can reflect various surfaces
from an ideally slippery and reflecting surface to a completely non-slippery and dampening surface.

The purpose of the project is research into the trajectories of complex dynamic systems.


### Project status
unfair-dice-simulation is in a very early stage of its development, don't expect much yet.


### Dependencies
* GNU Make (build)
* Clang compiler
* OpenSceneGraph (odg): for 3D rendering
* Boost: for ODE solving, and for some container types
* nlohmann-json: for config files parsing

