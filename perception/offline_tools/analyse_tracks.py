import csv
from collections import defaultdict
from pathlib import Path

csv_path = Path("outputs/multiple_3_trajectories.csv")

if not csv_path.exists():
    raise FileNotFoundError(f"CSV not found: {csv_path.resolve()}")

tracks = defaultdict(list)

with csv_path.open("r", newline="") as file:
    reader = csv.DictReader(file)

    for row in reader:
        tracks[int(row["track_id"])].append({
            "frame": int(row["frame"]),
            "time": float(row["video_time_s"]),
            "confidence": float(row["confidence"]),
        })

if not tracks:
    raise RuntimeError("The CSV contains no tracking observations.")

print("\n===== ARC TRACKING BASELINE =====")
print(f"Unique track IDs: {len(tracks)}")
print(f"Total observations: {sum(len(rows) for rows in tracks.values())}")

all_frames = [
    row["frame"]
    for observations in tracks.values()
    for row in observations
]

print(f"First observed frame: {min(all_frames)}")
print(f"Last observed frame: {max(all_frames)}")

print("\n===== INDIVIDUAL TRACKS =====")
print(
    f"{'ID':>5} {'Observations':>12} {'Duration(s)':>12} "
    f"{'Largest gap(frames)':>20} {'Mean confidence':>16}"
)

for track_id, observations in sorted(
    tracks.items(),
    key=lambda item: len(item[1]),
    reverse=True,
):
    observations.sort(key=lambda row: row["frame"])

    first_frame = observations[0]["frame"]
    last_frame = observations[-1]["frame"]

    # A gap of 1 means consecutive frames.
    gaps = [
        observations[i]["frame"] - observations[i - 1]["frame"]
        for i in range(1, len(observations))
    ]

    largest_gap = max(gaps, default=1)

    duration = (
        observations[-1]["time"] - observations[0]["time"]
    )

    mean_confidence = sum(
        row["confidence"] for row in observations
    ) / len(observations)

    print(
        f"{track_id:>5} "
        f"{len(observations):>12} "
        f"{duration:>12.2f} "
        f"{largest_gap:>20} "
        f"{mean_confidence:>16.3f}"
    )

print("\n===== TRACK 28: OBSERVATION GAPS =====")

track_id = 28
observations = sorted(
    tracks[track_id],
    key=lambda row: row["frame"],
)

for previous, current in zip(observations, observations[1:]):
    gap = current["frame"] - previous["frame"]

    if gap >= 5:
        print(
            f"Frames {previous['frame']} -> {current['frame']} | "
            f"Times {previous['time']:.2f}s -> "
            f"{current['time']:.2f}s | "
            f"Frame difference: {gap}"
        )

print("\nAnalysis complete.")