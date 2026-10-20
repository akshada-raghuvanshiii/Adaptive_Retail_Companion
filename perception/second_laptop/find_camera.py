"""Lists which camera indices open on this computer:  python find_camera.py"""
import cv2
found = []
for i in range(5):
    cap = cv2.VideoCapture(i)
    ok, frame = cap.read() if cap.isOpened() else (False, None)
    if ok:
        found.append(i)
        print(f"camera {i}: works, {frame.shape[1]}x{frame.shape[0]}")
    cap.release()
print("use: CAMERA =", found[0] if found else "none found (plug the webcam in, close other apps using it)")
