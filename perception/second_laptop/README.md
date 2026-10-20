# ARC perception on a second laptop

Replaces the Raspberry Pi. This laptop runs the webcam, YOLOv8n, ByteTrack and the target lock,
and sends two flags plus a small preview to the ARC laptop over UDP (port 5600). The ARC laptop
sends the shopper keys back (port 5601). No ROS on this laptop. Same code as the Pi version.

1. Once, with internet: `./setup.sh` (Linux) or `setup.bat` (Windows). It ends by listing cameras.
2. Join the same Wi-Fi or hotspot as the ARC laptop.
3. On the ARC laptop: `PERC=<this laptop's IP> ~/arc_pi_udp/laptop/arc_demo.sh up`. It prints the ARC laptop's IP.
4. Here: `./run_perception.sh <ARC laptop IP> <camera>` (Windows: `run_perception.bat <ARC laptop IP> <camera>`).
5. ARC_relay on the ARC laptop prints `receiving from the Pi at <this laptop>`.

This laptop's IP: `hostname -I` (Linux) or `ipconfig` → IPv4 Address of the Wi-Fi adapter (Windows).
Slow? Start with `IMGSZ=320` (Linux: `IMGSZ=320 ./run_perception.sh ...`; Windows: `set IMGSZ=320` first).
