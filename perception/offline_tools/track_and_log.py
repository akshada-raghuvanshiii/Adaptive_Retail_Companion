import argparse
import csv
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(
        description="ARC person tracking and trajectory logger"
    )
    parser.add_argument("--source", required=True, help="Input video path")
    parser.add_argument(
        "--model",
        default="yolov8n.pt",
        help="YOLO model path or name",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Directory for CSV and annotated video",
    )
    parser.add_argument(
        "--imgsz", type=int, default=640,
        help="Inference image size",
    )
    parser.add_argument(
        "--conf", type=float, default=0.25,
        help="Minimum detection confidence",
    )
    args = parser.parse_args()

    source = Path(args.source).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not source.is_file():
        raise FileNotFoundError(f"Video not found: {source}")

    model = YOLO(args.model)

    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video: {source}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    if fps <= 0:
        capture.release()
        raise RuntimeError("Video reported an invalid frame rate.")

    video_stem = source.stem

    csv_path = output_dir / f"{video_stem}_trajectories.csv"
    video_path = output_dir / f"{video_stem}_tracked.mp4"

    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )

    if not writer.isOpened():
        capture.release()
        raise RuntimeError("Could not create output video.")

    fields = [
        "frame",
        "video_time_s",
        "wall_time_s",
        "track_id",
        "x_center",
        "y_center",
        "box_width",
        "box_height",
        "confidence",
    ]

    frame_index = 0
    start_time = time.perf_counter()
    inference_times = []

    print(f"Input: {source}")
    print(f"Video FPS: {fps:.2f}")
    print(f"Resolution: {width}x{height}")
    print(f"Frames: {frame_count}")
    print("Processing on CPU...")

    try:
        with csv_path.open("w", newline="") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fields)
            csv_writer.writeheader()

            while True:
                success, frame = capture.read()
                if not success:
                    break

                frame_index += 1

                inference_start = time.perf_counter()

                result = model.track(
                    frame,
                    persist=True,
                    tracker="bytetrack.yaml",
                    classes=[0],
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device="cpu",
                    verbose=False,
                )[0]

                inference_times.append(
                    time.perf_counter() - inference_start
                )

                annotated = result.plot()

                if result.boxes is not None and result.boxes.id is not None:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    ids = result.boxes.id.int().cpu().tolist()
                    confidences = (
                        result.boxes.conf.cpu().tolist()
                    )

                    for box, track_id, confidence in zip(
                        boxes, ids, confidences
                    ):
                        x1, y1, x2, y2 = box

                        csv_writer.writerow({
                            "frame": frame_index,
                            "video_time_s": (frame_index - 1) / fps,
                            "wall_time_s": (
                                time.perf_counter() - start_time
                            ),
                            "track_id": track_id,
                            "x_center": (x1 + x2) / 2,
                            "y_center": (y1 + y2) / 2,
                            "box_width": x2 - x1,
                            "box_height": y2 - y1,
                            "confidence": confidence,
                        })

                writer.write(annotated)

                if frame_index % 50 == 0:
                    print(
                        f"Processed {frame_index}/{frame_count} frames"
                    )

    finally:
        capture.release()
        writer.release()

    elapsed = time.perf_counter() - start_time

    print("\nProcessing complete.")
    print(f"Frames processed: {frame_index}")
    print(f"Elapsed time: {elapsed:.2f} seconds")
    print(
        f"Processing throughput: "
        f"{frame_index / elapsed:.2f} FPS"
        if elapsed > 0 else "Processing throughput unavailable"
    )

    if inference_times:
        print(
            f"Mean model-call time: "
            f"{sum(inference_times) / len(inference_times) * 1000:.2f} ms"
        )

    print(f"Trajectory CSV: {csv_path}")
    print(f"Annotated video: {video_path}")


if __name__ == "__main__":
    main()