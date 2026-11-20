# gravity_comp.py — gravity feed-forward for the SO-101's three pitch joints.
#
# Why: each servo holds its position with a proportional-derivative loop and no
# integral term. A joint carrying a gravity torque tau therefore settles short of
# its command by tau / K, where K is that joint's holding stiffness. Commanding
# theta - tau(theta) / K instead of theta pre-cancels the sag.
#
# The stiffness values were measured on this arm: median of 10 paired trials
# (docs/data/runs/20261003_*_real, targets 0 and 1), at the servo gains in
# my_follower_joints.yaml. Change a joint's P gain and its value must be
# re-measured.
#
# On by default on the real arm (launch argument compensate; off in simulation,
# which has no sag to cancel). Measured effect on hardware:
# 20.7 -> 4.9 mm at target 0 and 34.4 -> 7.9 mm at target 1 (5 trials each).

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

URDF_FILE = os.path.expanduser("~/so101_ws/urdf/so101.urdf")
CHAIN = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRAVITY = np.array([0.0, 0.0, -9.81])

# Holding stiffness, N.m/rad. shoulder_pan turns about a vertical axis and
# wrist_roll about the forearm, so gravity loads neither; they get no correction.
STIFFNESS = {
    "shoulder_lift": 25.6,   # at P = 16 (varies with load: 35 at target 0, 20 at target 1)
    "elbow_flex":     16.6,  # at P = 48
    "wrist_flex":     4.23,  # at P = 32
}


def _rotation(axis, angle):
    """Rotation matrix for `angle` radians about unit vector `axis` (Rodrigues)."""
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _rpy(r, p, y):
    """URDF fixed-axis roll/pitch/yaw: R = Rz(y) Ry(p) Rx(r)."""
    return _rotation(np.array([0, 0, 1.0]), y) @ _rotation(np.array([0, 1.0, 0]), p) \
        @ _rotation(np.array([1.0, 0, 0]), r)


def _transform(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def _numbers(text, default):
    return [float(v) for v in (text or default).split()]


def load_model(path=URDF_FILE):
    """Joints (origin, axis, parent, child) and link masses / centres of mass."""
    root = ET.parse(path).getroot()
    joints, links = {}, {}
    for j in root.findall("joint"):
        origin, axis = j.find("origin"), j.find("axis")
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            xyz=np.array(_numbers(origin.get("xyz") if origin is not None else None, "0 0 0")),
            rpy=_numbers(origin.get("rpy") if origin is not None else None, "0 0 0"),
            axis=np.array(_numbers(axis.get("xyz") if axis is not None else None, "0 0 1")))
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None or inertial.find("mass") is None:
            continue
        origin = inertial.find("origin")
        links[link.get("name")] = dict(
            mass=float(inertial.find("mass").get("value")),
            com=np.array(_numbers(origin.get("xyz") if origin is not None else None, "0 0 0")))
    return joints, links


def _frames(theta, joints):
    """World pose of every link, and the world origin/axis of each chain joint."""
    T, link_pose, joint_axis = np.eye(4), {}, {}
    for name, angle in zip(CHAIN, theta):
        j = joints[name]
        T_joint = T @ _transform(_rpy(*j["rpy"]), j["xyz"])
        joint_axis[name] = (T_joint[:3, 3], T_joint[:3, :3] @ j["axis"])
        T = T_joint @ _transform(_rotation(j["axis"], angle), np.zeros(3))
        link_pose[j["child"]] = T
    for _ in range(3):  # links hanging off the chain at fixed or unactuated joints
        for j in joints.values():
            if j["child"] not in link_pose and j["parent"] in link_pose:
                link_pose[j["child"]] = link_pose[j["parent"]] @ _transform(_rpy(*j["rpy"]), j["xyz"])
    return link_pose, joint_axis


def _links_beyond(joint, joints):
    """Every link the joint carries: its child link and everything downstream."""
    found, frontier = set(), [joints[joint]["child"]]
    while frontier:
        link = frontier.pop()
        found.add(link)
        frontier += [j["child"] for j in joints.values() if j["parent"] == link]
    return found


def gravity_torques(theta, model):
    """Static gravity torque (N.m) about each pitch joint's axis. Positive means
    gravity tries to increase that joint's angle."""
    joints, links = model
    link_pose, joint_axis = _frames(theta, joints)
    torques = {}
    for name in STIFFNESS:
        origin, axis = joint_axis[name]
        axis = axis / np.linalg.norm(axis)
        tau = 0.0
        for link in _links_beyond(name, joints):
            if link in links and link in link_pose:
                com = (link_pose[link] @ np.append(links[link]["com"], 1.0))[:3]
                tau += float(axis @ np.cross(com - origin, links[link]["mass"] * GRAVITY))
        torques[name] = tau
    return torques


def compensate(theta, model):
    """The joint command that should make the arm settle AT theta: each pitch
    joint is offset against its predicted sag, theta_j - tau_j / K_j."""
    torques = gravity_torques(theta, model)
    out = list(theta)
    for name, tau in torques.items():
        out[CHAIN.index(name)] -= tau / STIFFNESS[name]
    return out
