import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def load_trajectories(csv_path):
    """Load tracked person observations from a trajectory CSV."""

    tracks = defaultdict(list)

    with csv_path.open("r", newline="") as file:
        reader = csv.DictReader(file)

        required_columns = {
            "frame",
            "video_time_s",
            "track_id",
            "x_center",
            "y_center",
            "confidence",
        }

        missing = required_columns - set(reader.fieldnames or [])

        if missing:
            raise ValueError(
                f"Missing required CSV columns: {sorted(missing)}"
            )

        for row in reader:
            tracks[int(row["track_id"])].append({
                "frame": int(row["frame"]),
                "time": float(row["video_time_s"]),
                "x": float(row["x_center"]),
                "y": float(row["y_center"]),
                "confidence": float(row["confidence"]),
            })

    for observations in tracks.values():
        observations.sort(key=lambda item: item["frame"])

    return tracks


def calculate_movement_features(observations, max_gap_frames=2):
    """
    Calculate image-space movement features.

    Large observation gaps are not bridged. Movement across such gaps
    cannot be reliably inferred from the available coordinates.
    """

    results = []

    previous = None
    previous_valid_velocity = None

    for observation in observations:
        result = {
            **observation,
            "displacement_px": None,
            "speed_px_s": None,
            "direction_deg": None,
            "direction_change_deg": None,
            "valid_motion_sample": False,
        }

        if previous is not None:
            frame_gap = observation["frame"] - previous["frame"]
            time_gap = observation["time"] - previous["time"]

            # Avoid interpreting long gaps as continuous movement.
            if (
                1 <= frame_gap <= max_gap_frames
                and time_gap > 0
            ):
                dx = observation["x"] - previous["x"]
                dy = observation["y"] - previous["y"]

                displacement = math.hypot(dx, dy)
                speed = displacement / time_gap

                # Image coordinates increase downward along the y-axis.
                direction = math.degrees(math.atan2(-dy, dx))

                result["displacement_px"] = displacement
                result["speed_px_s"] = speed
                result["direction_deg"] = direction
                result["valid_motion_sample"] = True

                if previous_valid_velocity is not None:
                    direction_difference = (
                        direction - previous_valid_velocity
                    )

                    # Wrap angle into the range [-180, 180].
                    direction_difference = (
                        direction_difference + 180
                    ) % 360 - 180

                    result["direction_change_deg"] = abs(
                        direction_difference
                    )

                previous_valid_velocity = direction

            else:
                # A large gap breaks the continuous trajectory segment.
                previous_valid_velocity = None

        previous = observation
        results.append(result)

    return results


def detect_movement_events(
    features,
    stop_speed_px_s=12.0,
    stop_duration_s=1.0,
    direction_change_deg=45.0,
):
    """
    Generate preliminary observable movement events.

    Thresholds are initial experimental settings, not validated
    classifications of shopper intent or confusion.
    """

    events = []

    # Detect low image-space movement sustained across consecutive
    # valid observations. Gaps break the stopping interval.
    stationary_start = None
    stationary_last_time = None
    stationary_last_frame = None

    for item in features:
        valid = item["valid_motion_sample"]
        slow = (
            valid
            and item["speed_px_s"] is not None
            and item["speed_px_s"] < stop_speed_px_s
        )

        if slow:
            if (
                stationary_start is None
                or item["frame"] - stationary_last_frame > 2
            ):
                stationary_start = item["time"]

            stationary_last_time = item["time"]
            stationary_last_frame = item["frame"]

            duration = stationary_last_time - stationary_start

            if duration >= stop_duration_s:
                events.append({
                    "track_id": item["track_id"],
                    "event": "prolonged_low_image_motion",
                    "start_time_s": round(stationary_start, 3),
                    "end_time_s": round(stationary_last_time, 3),
                    "duration_s": round(duration, 3),
                    "frame": item["frame"],
                    "confidence": item["confidence"],
                })

                # Reset after recording an event to avoid logging
                # the same interval on every subsequent frame.
                stationary_start = item["time"]
                stationary_last_time = item["time"]
                stationary_last_frame = item["frame"]

        else:
            stationary_start = None
            stationary_last_time = None
            stationary_last_frame = None

        angle_change = item["direction_change_deg"]

        if (
            angle_change is not None
            and angle_change >= direction_change_deg
        ):
            events.append({
                "track_id": item["track_id"],
                "event": "direction_change",
                "start_time_s": round(item["time"], 3),
                "end_time_s": round(item["time"], 3),
                "duration_s": 0.0,
                "frame": item["frame"],
                "confidence": item["confidence"],
            })

    return events


def main():
    parser = argparse.ArgumentParser(
        description="ARC offline person movement analysis"
    )

    parser.add_argument(
        "--source",
        required=True,
        help="Trajectory CSV generated by track_and_log.py",
    )

    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Directory for movement-analysis results",
    )

    parser.add_argument(
        "--max-gap-frames",
        type=int,
        default=2,
        help="Largest frame gap allowed for a motion estimate",
    )

    parser.add_argument(
        "--stop-speed",
        type=float,
        default=12.0,
        help="Initial low-motion threshold in pixels per second",
    )

    parser.add_argument(
        "--stop-duration",
        type=float,
        default=1.0,
        help="Minimum duration for a prolonged low-motion event",
    )

    parser.add_argument(
        "--direction-change",
        type=float,
        default=45.0,
        help="Initial direction-change threshold in degrees",
    )

    args = parser.parse_args()

    if args.max_gap_frames < 1:
        parser.error("--max-gap-frames must be at least 1")

    source = Path(args.source)

    if not source.is_file():
        raise FileNotFoundError(
            f"Trajectory CSV not found: {source.resolve()}"
        )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    tracks = load_trajectories(source)

    if not tracks:
        raise RuntimeError("No trajectory observations were found.")

    feature_path = output_dir / (
        f"{source.stem}_features.csv"
    )

    event_path = output_dir / (
        f"{source.stem}_events.csv"
    )

    all_features = []
    all_events = []

    for track_id, observations in tracks.items():
        features = calculate_movement_features(
            observations,
            max_gap_frames=args.max_gap_frames,
        )

        for item in features:
            item["track_id"] = track_id

        events = detect_movement_events(
            features,
            stop_speed_px_s=args.stop_speed,
            stop_duration_s=args.stop_duration,
            direction_change_deg=args.direction_change,
        )

        all_features.extend(features)
        all_events.extend(events)

    feature_fields = [
        "frame",
        "time",
        "track_id",
        "x",
        "y",
        "confidence",
        "displacement_px",
        "speed_px_s",
        "direction_deg",
        "direction_change_deg",
        "valid_motion_sample",
    ]

    with feature_path.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=feature_fields,
        )
        writer.writeheader()

        for item in sorted(
            all_features,
            key=lambda row: (row["frame"], row["track_id"]),
        ):
            writer.writerow({
                key: item.get(key)
                for key in feature_fields
            })

    event_fields = [
        "track_id",
        "event",
        "start_time_s",
        "end_time_s",
        "duration_s",
        "frame",
        "confidence",
    ]

    with event_path.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=event_fields,
        )
        writer.writeheader()
        writer.writerows(all_events)

    valid_samples = sum(
        item["valid_motion_sample"]
        for item in all_features
    )

    print("\n===== ARC MOVEMENT ANALYSIS =====")
    print(f"Input: {source}")
    print(f"Tracks analysed: {len(tracks)}")
    print(f"Observations analysed: {len(all_features)}")
    print(f"Valid motion samples: {valid_samples}")
    print(f"Movement events logged: {len(all_events)}")
    print(f"Feature CSV: {feature_path}")
    print(f"Event CSV: {event_path}")

    print("\nEvent counts:")

    counts = defaultdict(int)

    for event in all_events:
        counts[event["event"]] += 1

    if counts:
        for event_name, count in sorted(counts.items()):
            print(f"  {event_name}: {count}")
    else:
        print("  No events detected with the current thresholds.")

    print("\nAnalysis complete.")


if __name__ == "__main__":
    main()