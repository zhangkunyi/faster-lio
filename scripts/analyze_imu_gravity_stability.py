#!/usr/bin/env python3

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rospy
from sensor_msgs.msg import Imu


def angle_deg(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
    norm_a = np.linalg.norm(vec_a)
    norm_b = np.linalg.norm(vec_b)
    if norm_a <= 1e-12 or norm_b <= 1e-12:
        return float("nan")
    cos_theta = float(np.dot(vec_a, vec_b) / (norm_a * norm_b))
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


@dataclass
class StabilityResult:
    sample_count: int
    time_sec: float
    angle_to_final_deg: float
    delta_angle_deg: float
    acc_norm_error: float
    gyro_bias_norm: float


class LiveImuCollector:
    def __init__(self, topic: str, max_samples: int):
        self.topic = topic
        self.max_samples = max_samples
        self.acc_samples: List[np.ndarray] = []
        self.gyr_samples: List[np.ndarray] = []
        self.stamps: List[float] = []
        self.sub = rospy.Subscriber(topic, Imu, self._callback, queue_size=20000, tcp_nodelay=True)

    def _callback(self, msg: Imu) -> None:
        if len(self.acc_samples) >= self.max_samples:
            return
        self.acc_samples.append(
            np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                dtype=np.float64,
            )
        )
        self.gyr_samples.append(
            np.array(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                dtype=np.float64,
            )
        )
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()
        self.stamps.append(float(stamp))

    def collect(self, duration_sec: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        start_wall = rospy.get_time()
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - start_wall
            if elapsed >= duration_sec:
                break
            if len(self.acc_samples) >= self.max_samples:
                break
            rate.sleep()

        self.sub.unregister()

        if not self.acc_samples:
            raise RuntimeError(f"No IMU messages received on {self.topic}")

        return (
            np.vstack(self.acc_samples),
            np.vstack(self.gyr_samples),
            np.asarray(self.stamps, dtype=np.float64),
        )


def cumulative_means(samples: np.ndarray) -> np.ndarray:
    csum = np.cumsum(samples, axis=0)
    counts = np.arange(1, samples.shape[0] + 1, dtype=np.float64).reshape(-1, 1)
    return csum / counts


def evaluate_stability(
    acc_samples: np.ndarray,
    gyr_samples: np.ndarray,
    stamps: np.ndarray,
    min_samples: int,
    window: int,
    angle_threshold_deg: float,
    delta_threshold_deg: float,
    acc_norm_threshold: float,
) -> Tuple[Sequence[StabilityResult], Optional[StabilityResult], np.ndarray, np.ndarray]:
    mean_acc = cumulative_means(acc_samples)
    mean_gyr = cumulative_means(gyr_samples)
    final_acc = mean_acc[-1]
    final_gravity_dir = -final_acc / np.linalg.norm(final_acc)

    stable_result: Optional[StabilityResult] = None
    results: List[StabilityResult] = []
    delta_angles: List[float] = []

    for idx in range(mean_acc.shape[0]):
        cur_gravity_dir = -mean_acc[idx] / np.linalg.norm(mean_acc[idx])
        angle_to_final = angle_deg(cur_gravity_dir, final_gravity_dir)
        if idx == 0:
            delta_angle = float("nan")
        else:
            prev_gravity_dir = -mean_acc[idx - 1] / np.linalg.norm(mean_acc[idx - 1])
            delta_angle = angle_deg(cur_gravity_dir, prev_gravity_dir)
        delta_angles.append(delta_angle)

        acc_norm_error = abs(np.linalg.norm(mean_acc[idx]) - 9.81)
        gyro_bias_norm = float(np.linalg.norm(mean_gyr[idx]))
        time_sec = float(stamps[idx] - stamps[0]) if len(stamps) > 1 else 0.0

        results.append(
            StabilityResult(
                sample_count=idx + 1,
                time_sec=time_sec,
                angle_to_final_deg=angle_to_final,
                delta_angle_deg=delta_angle,
                acc_norm_error=acc_norm_error,
                gyro_bias_norm=gyro_bias_norm,
            )
        )

    for start_idx in range(max(0, min_samples - 1), len(results)):
        end_idx = min(len(results), start_idx + window)
        if end_idx - start_idx < window:
            break
        window_results = results[start_idx:end_idx]
        if all(
            item.angle_to_final_deg <= angle_threshold_deg
            and (math.isnan(item.delta_angle_deg) or item.delta_angle_deg <= delta_threshold_deg)
            and item.acc_norm_error <= acc_norm_threshold
            for item in window_results
        ):
            stable_result = results[start_idx]
            break

    return results, stable_result, mean_acc, mean_gyr


def percentile_result(results: Sequence[StabilityResult], ratio: float) -> StabilityResult:
    idx = min(len(results) - 1, max(0, int(math.ceil(len(results) * ratio)) - 1))
    return results[idx]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Collect live IMU data and estimate how many samples are needed for gravity alignment stability."
    )
    parser.add_argument("--topic", default="/mavros/imu/data", help="IMU topic")
    parser.add_argument("--duration", type=float, default=10.0, help="Collection duration in seconds")
    parser.add_argument("--max-samples", type=int, default=5000, help="Maximum IMU samples to collect")
    parser.add_argument("--min-samples", type=int, default=20, help="Do not declare stable before this sample count")
    parser.add_argument("--window", type=int, default=50, help="Consecutive samples that must satisfy the thresholds")
    parser.add_argument("--angle-threshold-deg", type=float, default=0.1, help="Max angle to final gravity estimate")
    parser.add_argument(
        "--delta-threshold-deg",
        type=float,
        default=0.02,
        help="Max step-to-step change in gravity direction estimate",
    )
    parser.add_argument(
        "--acc-norm-threshold",
        type=float,
        default=0.05,
        help="Max absolute error of mean acceleration norm from 9.81 m/s^2",
    )
    args = parser.parse_args()

    rospy.init_node("imu_gravity_stability_analyzer", anonymous=True, disable_signals=True)

    collector = LiveImuCollector(args.topic, args.max_samples)
    acc_samples, gyr_samples, stamps = collector.collect(args.duration)

    results, stable_result, mean_acc, mean_gyr = evaluate_stability(
        acc_samples=acc_samples,
        gyr_samples=gyr_samples,
        stamps=stamps,
        min_samples=args.min_samples,
        window=args.window,
        angle_threshold_deg=args.angle_threshold_deg,
        delta_threshold_deg=args.delta_threshold_deg,
        acc_norm_threshold=args.acc_norm_threshold,
    )

    final_mean_acc = mean_acc[-1]
    final_mean_gyr = mean_gyr[-1]
    avg_dt = np.diff(stamps).mean() if len(stamps) > 1 else float("nan")
    hz = 1.0 / avg_dt if avg_dt and not math.isnan(avg_dt) and avg_dt > 0.0 else float("nan")

    print(f"topic: {args.topic}")
    print(f"collected_samples: {len(results)}")
    print(f"duration_sec: {stamps[-1] - stamps[0]:.4f}" if len(stamps) > 1 else "duration_sec: 0.0000")
    print(f"estimated_hz: {hz:.2f}" if not math.isnan(hz) else "estimated_hz: nan")
    print(
        "final_mean_acc: "
        f"[{final_mean_acc[0]:.6f}, {final_mean_acc[1]:.6f}, {final_mean_acc[2]:.6f}]"
    )
    print(f"final_mean_acc_norm: {np.linalg.norm(final_mean_acc):.6f}")
    print(
        "final_mean_gyr: "
        f"[{final_mean_gyr[0]:.6f}, {final_mean_gyr[1]:.6f}, {final_mean_gyr[2]:.6f}]"
    )
    print(f"final_mean_gyr_norm: {np.linalg.norm(final_mean_gyr):.6f}")

    print("milestones:")
    for ratio in (0.01, 0.02, 0.05, 0.10, 0.20, 0.50, 1.00):
        item = percentile_result(results, ratio)
        print(
            f"  n={item.sample_count:4d} t={item.time_sec:7.4f}s "
            f"angle_to_final={item.angle_to_final_deg:8.5f} deg "
            f"delta={item.delta_angle_deg:8.5f} deg "
            f"|acc|-9.81={item.acc_norm_error:8.5f} "
            f"|bg|={item.gyro_bias_norm:8.6f}"
        )

    if stable_result is None:
        print("stable_sample_count: not_found")
        return 2

    print(
        "stable_sample_count: "
        f"{stable_result.sample_count} "
        f"(t={stable_result.time_sec:.4f}s, "
        f"angle_to_final={stable_result.angle_to_final_deg:.5f} deg, "
        f"delta={stable_result.delta_angle_deg:.5f} deg, "
        f"|acc|-9.81={stable_result.acc_norm_error:.5f}, "
        f"|bg|={stable_result.gyro_bias_norm:.6f})"
    )

    return 0


if __name__ == "__main__":
    sys.exit(main())
