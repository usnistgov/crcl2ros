
## <a name="Q:_On_longer_trajectory_paths_it_appears_as_if_the_moveit_trajectory_overshoots._What_gives?"></a>Q: On longer trajectory paths it appears as if the moveit trajectory overshoots. What gives?


It sounds like a controller issue more than a moveit issue. In the robot's URDF I'd check to see if the <joint> tags have the "effort" property set. I set effort to -1 (no force limits) so that the robot can stop on a dime.








