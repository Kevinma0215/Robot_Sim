# Simulation Module Package

Use gazebo to do simulation, including environment building and each module integration.

## Structure

```
.
├── CMakeLists.txt
├── launch
│   ├── display_world.launch
│   └── my_sim.launch
├── package.xml
├── README.md
└── worlds
    └── line_following_world.world

```

## Status

Successfully integrate world environment with robot module.

### To-do
- integrate Perception Module.
- integrate Control Module.
- Design more suitable visualization system.