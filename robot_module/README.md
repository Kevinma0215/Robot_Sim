# Robot Module Package

## Structure
```
.
├── CMakeLists.txt
├── config
│   ├── caster_wheel.yaml
│   ├── chassis.yaml
│   └── wheel.yaml
├── launch
│   └── display_xacro.launch
├── package.xml
├── README.md
├── rviz
│   └── my_bot.rviz
└── urdf
    ├── include
    │   ├── inertia.xacro
    │   ├── robot_macros.xacro
    │   ├── rviz_rgba_def.xacro
    │   └── sensor_macros.xacro
    └── my_bot.xacro

```

## Note
- `config` : store model parameters.
- `display_xacro.launch` : use rviz to check model.