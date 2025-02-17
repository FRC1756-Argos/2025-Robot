#!/usr/bin/env python3

import math
from dataclasses import dataclass


@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"w={self.w:.4f}, x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}"


def normalize_quaternion(q: Quaternion) -> Quaternion:
    """
    Normalizes a quaternion to unit length.

    Args:
        q (Quaternion): Input quaternion

    Returns:
        Quaternion: Normalized quaternion

    Raises:
        ValueError: If quaternion has zero magnitude
    """
    magnitude = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)

    if magnitude == 0:
        raise ValueError("Cannot normalize quaternion with zero magnitude")

    return Quaternion(
        w=q.w / magnitude, x=q.x / magnitude, y=q.y / magnitude, z=q.z / magnitude
    )


def get_quaternion_angle_difference(q1: Quaternion, q2: Quaternion) -> float:
    """
    Calculates the angular difference between two quaternions in degrees.

    Args:
        q1 (Quaternion): First quaternion
        q2 (Quaternion): Second quaternion

    Returns:
        float: Angle difference in degrees
    """
    # First normalize both quaternions
    q1_norm = normalize_quaternion(q1)
    q2_norm = normalize_quaternion(q2)

    # Calculate dot product
    dot_product = (
        q1_norm.w * q2_norm.w
        + q1_norm.x * q2_norm.x
        + q1_norm.y * q2_norm.y
        + q1_norm.z * q2_norm.z
    )

    # Clamp the dot product to [-1, 1] to avoid numerical errors
    clamped_dot = max(-1, min(1, dot_product))

    # Calculate the angle in radians and convert to degrees
    angle_radians = 2 * math.acos(abs(clamped_dot))
    angle_degrees = angle_radians * (180 / math.pi)

    return angle_degrees


@dataclass
class Example:
    name: str
    q1: Quaternion
    q2: Quaternion


def main():
    # Example quaternions
    examples = [
        Example(
            name="90-degree X rotation",
            q1=Quaternion(
                w=0.000000000000000006123233995736766, x=0, y=0, z=1.0
            ),  # Identity quaternion
            q2=Quaternion(
                w=0.001740148123557302,
                x=-0.005060063594917707,
                y=-0.002028597171044279,
                z=0.999836260831679,
            ),  # 90-degree rotation around X
        ),
        Example(
            name="180-degree Y rotation",
            q1=Quaternion(w=1, x=0, y=0, z=0),  # Identity quaternion
            q2=Quaternion(w=0, x=0, y=1, z=0),  # 180-degree rotation around Y
        ),
        Example(
            name="45-degree Z rotation",
            q1=Quaternion(w=1, x=0, y=0, z=0),  # Identity quaternion
            q2=Quaternion(
                w=0.9238795325112867, x=0, y=0, z=0.3826834323650898
            ),  # 45-degree rotation around Z
        ),
    ]

    print("Quaternion Angle Comparison Examples:\n")

    for example in examples:
        try:
            difference = get_quaternion_angle_difference(example.q1, example.q2)
            print(f"{example.name}:")
            print(f"Q1: {example.q1}")
            print(f"Q2: {example.q2}")
            print(f"Angle difference: {difference:.2f} degrees\n")
        except ValueError as e:
            print(f"Error in {example.name}: {str(e)}\n")

    # Example of how to test custom quaternions
    # custom_q1 = Quaternion(w=1, x=0, y=0, z=0)
    # custom_q2 = Quaternion(w=0.7071, x=0.7071, y=0, z=0)
    # custom_difference = get_quaternion_angle_difference(custom_q1, custom_q2)
    # print(f"Custom quaternions angle difference: {custom_difference:.2f} degrees")


if __name__ == "__main__":
    main()
