**Services:**
- `plan_to_cart_rpy`:  
    Plans joint trajectories to reach a given end-effector pose in cartesian space where the orientation is given as Euler angles (RPY).
- `plan_to_cart_quat`:  
    Plans joint trajectories to reach a given end-effector pose in cartesian space where the orientation is given as quaternion.
- `plan_to_joint`:  
  Plans joint trajectories to reach a desired joint angle goal
- `execute_plan`:  
  Executes the trajectories planned through various `plan_*` service calls
- `move_home`:  
  Plans joint trajectory and moves the manipulator to the configuration where all joint angles are zero.
- `move_to_cart_rpy`:  
  Plans joint trajectory and moves the end-effector to a given pose where the orientation is given as Euler angles (RPY).
- `move_to_cart_quat`:  
  Moves the end-effector to a given pose where the orientation is given as quaternions.
- `move_to_joint`  
  Executes the trajectories planned through various service `plan_*` service calls
