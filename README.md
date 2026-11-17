# SO-101 arm: code and configuration

ROS 2 Jazzy workspace for one SO-101 arm (5-DOF + gripper, Feetech STS3215 servos): MoveIt 2 pose commander, forward-kinematics verifier, gravity feed-forward, servo calibration and gains.

Layout:
- `src/so101_pose_milestone/`: pose commander, verifier, gravity feed-forward, launch, trial script
- `my_follower_joints.yaml`: per-servo calibration and gains loaded at launch
- `urdf/so101.urdf`: robot description used by the verifier
- `src/so101-ros-physical-ai/`: modified copy of [legalaspro/so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai) (Apache-2.0), including a modified copy of `feetech_ros2_driver` (BSD-3-Clause). Their LICENSE files are kept; changes are mine.
