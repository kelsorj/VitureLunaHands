from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel, Field, field_validator

from .joints import JOINT_IDS, N_JOINTS


class Joint(BaseModel):
    id: str
    p: tuple[float, float, float]
    r: tuple[float, float, float, float]


class Frame(BaseModel):
    t: float
    seq: int
    hand: Literal["left", "right"]
    tracked: bool
    wrist_pos: tuple[float, float, float]
    wrist_rot: tuple[float, float, float, float]
    joints: list[Joint] = Field(min_length=N_JOINTS, max_length=N_JOINTS)

    @field_validator("joints")
    @classmethod
    def _joint_order(cls, v: list[Joint]) -> list[Joint]:
        names = [j.id for j in v]
        if tuple(names) != JOINT_IDS:
            missing = set(JOINT_IDS) - set(names)
            extra = set(names) - set(JOINT_IDS)
            raise ValueError(
                f"joint order/set mismatch; missing={missing}, extra={extra}"
            )
        return v

    def joint_positions(self) -> np.ndarray:
        """Return (26, 3) positions in frame order, as float32 ndarray."""
        return np.array([j.p for j in self.joints], dtype=np.float32)
