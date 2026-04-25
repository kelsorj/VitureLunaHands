"""Canonical 26-joint ordering, matching Unity's XRHandJointID (OpenXR)."""

JOINT_IDS: tuple[str, ...] = (
    "palm",
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)

JOINT_INDEX: dict[str, int] = {name: i for i, name in enumerate(JOINT_IDS)}

N_JOINTS = len(JOINT_IDS)
assert N_JOINTS == 26
