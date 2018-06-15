# To launch the jogging simulation:

Clone this package and UT-NRG's jog_arm package and build everything.

Launch the SIA5 simulation:

```roslaunch motoman\_gazebo sia5\_gazebo\_nishida\_lab.launch```

```roslaunch motoman\_moveit sia5\_gazebo\_nishida\_lab\_moveit\_planning\_execution.launch```

Move to a nonsingular starting position.

Launch the jogger:

```roslaunch jog\_arm jog\_with\_spacenav.launch```
