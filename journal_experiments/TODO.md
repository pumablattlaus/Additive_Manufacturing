# TODO for SImualtion of journal_experiments

## Launch

```bash
roslaunch journal_experiments mur_sim.launch
roslaunch journal_experiments start_experiment.launch
```

wait for URs to reach home pose, then:

```bash
roslaunch journal_experiments twist_sim.launch
```

Start Ur is at (52.411910, 43.385410)

## Problems
- Twist Controller: orientation control correct?

- No PointCloud!
  - lateral_nozzle_pose_override is set to 0 in control_ur

## Future
- All in namespace like move_to_start_pose
- Include speeds
  - calc speed mir/ur in Parse Path
  - change ur_control accordingly
- Path publisher for viewing (less points)
