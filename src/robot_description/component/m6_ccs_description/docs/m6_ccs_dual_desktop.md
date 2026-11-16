# Marvin M6-CCS Dual-Arm Desktop

Dual-arm **booth / demo stand** layout for m6_ccs. Use **`variant:=desktop`** (m6_ccs-specific platform variant), not `type:=desktop`.

End effectors are selected independently via `type` / `left_end_effector` / `right_end_effector` (see [m6_ccs_description/README.md](../README.md)).

## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to m6_ccs_description --symlink-install
```

## 2. Visualize

* Dual arm on desktop (no gripper)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_visualize manipulator.launch.py robot:=m6_ccs variant:=desktop collider:=simple
```

* Desktop + Jodell RG75 gripper

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_visualize manipulator.launch.py robot:=m6_ccs variant:=desktop end_effector:=jodell_rg75 collider:=simple
```


## 3. OCS2 Arm Controller

### 3.1 Mock Components

* Desktop only

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup dual_arm.launch.py robot:=m6_ccs variant:=desktop
```

* Desktop + Jodell RG75

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup dual_arm.launch.py robot:=m6_ccs variant:=desktop end_effector:=jodell_rg75
```


### 3.2 Isaac Sim

* Desktop only

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup dual_arm.launch.py robot:=m6_ccs variant:=desktop hardware:=isaac
```

* Desktop + Jodell RG75

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup dual_arm.launch.py robot:=m6_ccs variant:=desktop end_effector:=jodell_rg75 hardware:=isaac
```

