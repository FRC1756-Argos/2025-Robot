import argparse
import json
from dataclasses import dataclass

from quaternian_lib import Quaternion, get_quaternion_angle_difference


@dataclass
class Translation:
    x: float
    y: float
    z: float


def proc_args():
    parser = argparse.ArgumentParser(
        prog='quaternion "stuff"', description="ideal to calibrated field"
    )
    parser.add_argument("ideal_map")
    parser.add_argument("cal_map")
    args = parser.parse_args()

    with open(args.ideal_map, "r") as f:
        ideal_map_json = json.load(f)

    with open(args.cal_map, "r") as f:
        measured_map_json = json.load(f)

    def load_measurements(cal_map):
        return [
            (
                tag["ID"],
                Translation(
                    tag["pose"]["translation"]["x"],
                    tag["pose"]["translation"]["y"],
                    tag["pose"]["translation"]["z"],
                ),
                Quaternion(
                    tag["pose"]["rotation"]["quaternion"]["W"],
                    tag["pose"]["rotation"]["quaternion"]["X"],
                    tag["pose"]["rotation"]["quaternion"]["Y"],
                    tag["pose"]["rotation"]["quaternion"]["Z"],
                ),
            )
            for tag in cal_map["tags"]
        ]

    ideal_map_measurements = sorted(
        load_measurements(ideal_map_json), key=lambda x: x[0]
    )
    calibrated_map_measurements = sorted(
        load_measurements(measured_map_json), key=lambda x: x[0]
    )

    map_pairs = zip(ideal_map_measurements, calibrated_map_measurements)

    for ideal, cal in map_pairs:
        print(f"ID: {ideal[0]}")
        print(f"X: {ideal[1].x - cal[1].x}")
        print(f"Y: {ideal[1].y - cal[1].y}")
        print(f"Z: {ideal[1].z - cal[1].z}")
        print(f"Angle: {get_quaternion_angle_difference(ideal[2], cal[2])}\n")


if __name__ == "__main__":
    proc_args()
