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

Start Ur is at (52.411910, 43.385410) for trafo (set rostopic pub /wall_frame_base)

## Problems
- Twist Controller: orientation control correct?

- No PointCloud!
  - lateral_nozzle_pose_override is set to 0 in control_ur

## Future
- for error: take only part orthogonal to path
  - use n path-points before and after current point if short distance between points
  - rest is done via feedforward
- All parameters etc in namespace like move_to_start_pose
- Include speeds
  - calc speed mir like UR
- Implement Scale_Map

### Multi-robot
2nd robot which moves to path point and back
- 3 poses: pickup, wait/start, drop off
- timing
  - no collision
  - concrete doesnt harden (right after 1st robot)
    - goto wait position where no collision with 1st robot
    - get current pose of 1st robot
    - if 1st robot is at path point, start 2nd robot
    - 2nd robot: reach path point from same direction of 1st robot or orthogonal?
      - same direction --> printed path is definetly reachable with same robot type
    - 2nd robot: move back to start pose
- drop something on the printed path

#### StateMachine
Concurrent StateMachine for multiple robots



## Helper
### **pub_paths_view.py**
- publishes the paths with less points for viewing

### Gazebo Simulation Speed (RT-Factor)
rosservice call /gazebo/set_physics_properties "time_step: 0.001
max_update_rate: 5000.0
gravity: 
  x: 0.0
  y: 0.0
  z: -9.8
ode_config: 
  auto_disable_bodies: False
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20"