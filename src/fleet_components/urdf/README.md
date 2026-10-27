# xacro macros

One file per category (`<category>.xacro`); each macro loads `components/<category>/<name>.yaml` with
`xacro.load_yaml` and emits the link, joint, inertial and Gazebo sensor or plugin for that component.
