"""
Human-to-Robot 3D Kinematics Retargeter & Synthetic ARKit/LiDAR Generator.
Converts Apple ARKit 3D Skeleton joints (metric root-relative coordinates)
into humanoid robot actuator angles (radians & degrees) with low-pass smoothing.
"""

import math
import time
from typing import Dict, List, Tuple

# Bone connectivity pairs for 3D visualization (ARKit joint names)
ARKIT_BONE_PAIRS: List[Tuple[str, str]] = [
    # Spine & Head
    ("hips_joint", "spine_1_joint"),
    ("spine_1_joint", "spine_4_joint"),
    ("spine_4_joint", "spine_7_joint"),
    ("spine_7_joint", "neck_1_joint"),
    ("neck_1_joint", "head_joint"),
    # Left Arm
    ("spine_7_joint", "left_shoulder_1_joint"),
    ("left_shoulder_1_joint", "left_arm_joint"),
    ("left_arm_joint", "left_forearm_joint"),
    ("left_forearm_joint", "left_hand_joint"),
    # Right Arm
    ("spine_7_joint", "right_shoulder_1_joint"),
    ("right_shoulder_1_joint", "right_arm_joint"),
    ("right_arm_joint", "right_forearm_joint"),
    ("right_forearm_joint", "right_hand_joint"),
    # Left Leg
    ("hips_joint", "left_upLeg_joint"),
    ("left_upLeg_joint", "left_leg_joint"),
    ("left_leg_joint", "left_foot_joint"),
    ("left_foot_joint", "left_toes_joint"),
    # Right Leg
    ("hips_joint", "right_upLeg_joint"),
    ("right_upLeg_joint", "right_leg_joint"),
    ("right_leg_joint", "right_foot_joint"),
    ("right_foot_joint", "right_toes_joint"),
]


def _vec_sub(a: List[float], b: List[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_norm(v: List[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _vec_normalize(v: List[float]) -> List[float]:
    n = _vec_norm(v)
    if n < 1e-6:
        return [0.0, 0.0, 0.0]
    return [v[0] / n, v[1] / n, v[2] / n]


def _vec_dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _angle_between(v1: List[float], v2: List[float]) -> float:
    """Returns angle in radians between two 3D vectors."""
    u1 = _vec_normalize(v1)
    u2 = _vec_normalize(v2)
    dot = max(-1.0, min(1.0, _vec_dot(u1, u2)))
    return math.acos(dot)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class HumanToRobotRetargeter:
    """
    Maps ARKit 3D human joint positions to a 16-DOF Humanoid Robot joint space
    with exponential moving average (EMA) filtering for smooth servo commands.
    """

    def __init__(self, smoothing_alpha: float = 0.35):
        self.alpha = smoothing_alpha
        self.smoothed_angles_rad: Dict[str, float] = {}

    def _get_pos(self, joints: Dict[str, List[float]], name: str, fallback: List[float]) -> List[float]:
        arr = joints.get(name)
        if arr and len(arr) >= 3:
            return [float(arr[0]), float(arr[1]), float(arr[2])]
        return fallback

    def retarget(self, joints: Dict[str, List[float]]) -> Dict[str, Dict[str, float]]:
        """
        Computes humanoid robot joint angles from ARKit metric 3D joint positions.
        Coordinate convention (ARKit body space):
          +X = Subject's Left, +Y = Up, +Z = Forward (out of chest)
        """
        hips = self._get_pos(joints, "hips_joint", [0.0, 0.0, 0.0])
        spine7 = self._get_pos(joints, "spine_7_joint", [0.0, 0.45, 0.0])
        neck = self._get_pos(joints, "neck_1_joint", [0.0, 0.55, 0.0])
        head = self._get_pos(joints, "head_joint", [0.0, 0.68, 0.0])

        l_arm = self._get_pos(joints, "left_arm_joint", [0.18, 0.44, 0.0])
        l_forearm = self._get_pos(joints, "left_forearm_joint", [0.35, 0.25, 0.0])
        l_hand = self._get_pos(joints, "left_hand_joint", [0.48, 0.05, 0.1])

        r_arm = self._get_pos(joints, "right_arm_joint", [-0.18, 0.44, 0.0])
        r_forearm = self._get_pos(joints, "right_forearm_joint", [-0.35, 0.25, 0.0])
        r_hand = self._get_pos(joints, "right_hand_joint", [-0.48, 0.05, 0.1])

        l_upleg = self._get_pos(joints, "left_upLeg_joint", [0.10, -0.05, 0.0])
        l_leg = self._get_pos(joints, "left_leg_joint", [0.10, -0.45, 0.0])
        l_foot = self._get_pos(joints, "left_foot_joint", [0.10, -0.85, 0.0])

        r_upleg = self._get_pos(joints, "right_upLeg_joint", [-0.10, -0.05, 0.0])
        r_leg = self._get_pos(joints, "right_leg_joint", [-0.10, -0.45, 0.0])
        r_foot = self._get_pos(joints, "right_foot_joint", [-0.10, -0.85, 0.0])

        # 1. Spine / Torso Orientation
        spine_vec = _vec_normalize(_vec_sub(spine7, hips))
        torso_pitch = math.atan2(spine_vec[2], max(0.01, spine_vec[1]))
        torso_roll = -math.atan2(spine_vec[0], max(0.01, spine_vec[1]))

        shoulder_line = _vec_sub(l_arm, r_arm)
        torso_yaw = -math.atan2(shoulder_line[2], max(0.01, shoulder_line[0]))

        # 2. Head Pan / Tilt relative to torso
        head_vec = _vec_normalize(_vec_sub(head, neck))
        head_tilt = math.atan2(head_vec[2], max(0.01, head_vec[1])) - torso_pitch
        head_pan = math.atan2(head_vec[0], max(0.01, head_vec[1]))

        # 3. Left Arm (Shoulder Pitch, Shoulder Roll, Elbow Flexion)
        l_upper = _vec_normalize(_vec_sub(l_forearm, l_arm))
        l_lower = _vec_normalize(_vec_sub(l_hand, l_forearm))
        l_shoulder_pitch = math.atan2(l_upper[2], -l_upper[1])
        l_shoulder_roll = math.atan2(l_upper[0], math.sqrt(l_upper[1] ** 2 + l_upper[2] ** 2))
        l_elbow_pitch = _clamp(_angle_between(l_upper, l_lower), 0.0, 2.6)

        # 4. Right Arm (Shoulder Pitch, Shoulder Roll, Elbow Flexion)
        r_upper = _vec_normalize(_vec_sub(r_forearm, r_arm))
        r_lower = _vec_normalize(_vec_sub(r_hand, r_forearm))
        r_shoulder_pitch = math.atan2(r_upper[2], -r_upper[1])
        r_shoulder_roll = math.atan2(-r_upper[0], math.sqrt(r_upper[1] ** 2 + r_upper[2] ** 2))
        r_elbow_pitch = _clamp(_angle_between(r_upper, r_lower), 0.0, 2.6)

        # 5. Left Leg (Hip Pitch, Hip Roll, Knee Flexion)
        l_thigh = _vec_normalize(_vec_sub(l_leg, l_upleg))
        l_shin = _vec_normalize(_vec_sub(l_foot, l_leg))
        l_hip_pitch = math.atan2(l_thigh[2], -l_thigh[1])
        l_hip_roll = math.atan2(l_thigh[0], -l_thigh[1])
        l_knee_pitch = _clamp(_angle_between(l_thigh, l_shin), 0.0, 2.4)

        # 6. Right Leg (Hip Pitch, Hip Roll, Knee Flexion)
        r_thigh = _vec_normalize(_vec_sub(r_leg, r_upleg))
        r_shin = _vec_normalize(_vec_sub(r_foot, r_leg))
        r_hip_pitch = math.atan2(r_thigh[2], -r_thigh[1])
        r_hip_roll = math.atan2(-r_thigh[0], -r_thigh[1])
        r_knee_pitch = _clamp(_angle_between(r_thigh, r_shin), 0.0, 2.4)

        raw_angles = {
            "torso_pitch": _clamp(torso_pitch, -0.8, 0.8),
            "torso_roll": _clamp(torso_roll, -0.6, 0.6),
            "torso_yaw": _clamp(torso_yaw, -1.2, 1.2),
            "head_pan": _clamp(head_pan, -1.2, 1.2),
            "head_tilt": _clamp(head_tilt, -0.7, 0.7),
            "left_shoulder_pitch": _clamp(l_shoulder_pitch, -2.8, 2.8),
            "left_shoulder_roll": _clamp(l_shoulder_roll, -0.3, 2.6),
            "left_elbow_pitch": l_elbow_pitch,
            "right_shoulder_pitch": _clamp(r_shoulder_pitch, -2.8, 2.8),
            "right_shoulder_roll": _clamp(r_shoulder_roll, -0.3, 2.6),
            "right_elbow_pitch": r_elbow_pitch,
            "left_hip_pitch": _clamp(l_hip_pitch, -1.6, 1.2),
            "left_hip_roll": _clamp(l_hip_roll, -0.7, 0.7),
            "left_knee_pitch": l_knee_pitch,
            "right_hip_pitch": _clamp(r_hip_pitch, -1.6, 1.2),
            "right_hip_roll": _clamp(r_hip_roll, -0.7, 0.7),
            "right_knee_pitch": r_knee_pitch,
        }

        # Apply EMA smoothing filter
        result_rad: Dict[str, float] = {}
        result_deg: Dict[str, float] = {}
        for k, v in raw_angles.items():
            prev = self.smoothed_angles_rad.get(k, v)
            smoothed = prev + self.alpha * (v - prev)
            self.smoothed_angles_rad[k] = smoothed
            result_rad[k] = round(smoothed, 4)
            result_deg[k] = round(math.degrees(smoothed), 1)

        return {"radians": result_rad, "degrees": result_deg}


def generate_demo_frame(frame_idx: int, t: float) -> dict:
    """
    Generates a realistic synthetic iPhone 13 Pro Max ARKit + LiDAR packet at 60 FPS
    so the Windows 3D Visualizer and Robot Retargeter can be tested immediately.
    """
    # Dynamic teleop choreography: torso sway, arm waving/reaching, knee squat
    squat = 0.08 * (0.5 - 0.5 * math.cos(t * 1.3))
    torso_lean_z = 0.06 * math.sin(t * 1.1)
    torso_lean_x = 0.05 * math.cos(t * 0.8)

    # Left arm wave & reach
    l_sh_pitch = 0.9 * math.sin(t * 1.8) + 0.5
    l_sh_roll = 0.45 + 0.35 * math.sin(t * 1.2)
    l_elb = 0.8 + 0.6 * math.sin(t * 2.4)

    # Right arm counter-motion
    r_sh_pitch = -0.8 * math.sin(t * 1.8) + 0.4
    r_sh_roll = 0.40 + 0.30 * math.cos(t * 1.4)
    r_elb = 0.75 + 0.55 * math.cos(t * 2.1)

    upper_arm_len = 0.28
    forearm_len = 0.25

    def compute_arm(side_sign: float, sh_pos: List[float], pitch: float, roll: float, elbow: float):
        # Upper arm vector from shoulder
        ux = side_sign * math.sin(roll)
        uy = -math.cos(roll) * math.cos(pitch)
        uz = math.cos(roll) * math.sin(pitch)
        elb_pos = [
            sh_pos[0] + upper_arm_len * ux,
            sh_pos[1] + upper_arm_len * uy,
            sh_pos[2] + upper_arm_len * uz,
        ]
        # Forearm bends forward/upward by `elbow` angle
        total_pitch = pitch + elbow
        fx = side_sign * math.sin(roll * 0.7)
        fy = -math.cos(roll * 0.7) * math.cos(total_pitch)
        fz = math.cos(roll * 0.7) * math.sin(total_pitch)
        hand_pos = [
            elb_pos[0] + forearm_len * fx,
            elb_pos[1] + forearm_len * fy,
            elb_pos[2] + forearm_len * fz,
        ]
        return elb_pos, hand_pos

    hips = [0.0, -squat, 0.0]
    spine1 = [torso_lean_x * 0.3, 0.14 - squat, torso_lean_z * 0.3]
    spine4 = [torso_lean_x * 0.6, 0.29 - squat, torso_lean_z * 0.6]
    spine7 = [torso_lean_x, 0.44 - squat, torso_lean_z]
    neck = [torso_lean_x * 1.1, 0.54 - squat, torso_lean_z * 1.1]
    head = [
        torso_lean_x * 1.2 + 0.04 * math.sin(t * 2.0),
        0.68 - squat,
        torso_lean_z * 1.2 + 0.03 * math.cos(t * 1.5),
    ]

    l_sh = [spine7[0] + 0.08, spine7[1] - 0.01, spine7[2]]
    r_sh = [spine7[0] - 0.08, spine7[1] - 0.01, spine7[2]]
    l_arm = [spine7[0] + 0.18, spine7[1] - 0.02, spine7[2]]
    r_arm = [spine7[0] - 0.18, spine7[1] - 0.02, spine7[2]]

    l_forearm, l_hand = compute_arm(1.0, l_arm, l_sh_pitch, l_sh_roll, l_elb)
    r_forearm, r_hand = compute_arm(-1.0, r_arm, r_sh_pitch, r_sh_roll, r_elb)

    knee_z = math.sqrt(max(0.0, 0.42**2 - (0.42 - squat * 0.6) ** 2))
    l_upleg = [0.10, -0.04 - squat, 0.0]
    r_upleg = [-0.10, -0.04 - squat, 0.0]
    l_leg = [0.11, -0.45 - squat * 0.4, knee_z]
    r_leg = [-0.11, -0.45 - squat * 0.4, knee_z]
    l_foot = [0.11, -0.86, 0.0]
    r_foot = [-0.11, -0.86, 0.0]
    l_toes = [0.11, -0.88, 0.12]
    r_toes = [-0.11, -0.88, 0.12]

    raw_joints = {
        "root": [0.0, 0.0, 0.0],
        "hips_joint": hips,
        "spine_1_joint": spine1,
        "spine_4_joint": spine4,
        "spine_7_joint": spine7,
        "neck_1_joint": neck,
        "head_joint": head,
        "left_shoulder_1_joint": l_sh,
        "left_arm_joint": l_arm,
        "left_forearm_joint": l_forearm,
        "left_hand_joint": l_hand,
        "right_shoulder_1_joint": r_sh,
        "right_arm_joint": r_arm,
        "right_forearm_joint": r_forearm,
        "right_hand_joint": r_hand,
        "left_upLeg_joint": l_upleg,
        "left_leg_joint": l_leg,
        "left_foot_joint": l_foot,
        "left_toes_joint": l_toes,
        "right_upLeg_joint": r_upleg,
        "right_leg_joint": r_leg,
        "right_foot_joint": r_foot,
        "right_toes_joint": r_toes,
    }

    joints_payload = {
        k: [round(p[0], 4), round(p[1], 4), round(p[2], 4), 0.0, 0.0, 0.0, 1.0]
        for k, p in raw_joints.items()
    }

    # Simulated 8x6 LiDAR depth map (subject at ~2.1m in center, background wall at ~3.4m)
    subject_dist = 2.10 + torso_lean_z
    grid = []
    for r in range(6):
        for c in range(8):
            if 2 <= c <= 5 and 1 <= r <= 4:
                d = subject_dist + 0.05 * math.sin(c + r + t)
            else:
                d = 3.35 - 0.15 * (r / 5.0)
            grid.append(round(d, 3))

    return {
        "timestamp": time.time(),
        "frameNumber": frame_idx,
        "lidarAvailable": True,
        "bodyTracked": True,
        "estimatedHeightScale": 1.02,
        "rootPosition": [round(torso_lean_x, 4), 0.0, round(-subject_dist, 4)],
        "rootOrientation": [0.0, 0.0, 0.0, 1.0],
        "centerLidarDepthM": round(subject_dist, 3),
        "lidarGrid8x6": grid,
        "joints": joints_payload,
    }
