import time
from picamera2 import Picamera2, MappedArray
from PIL import Image
import piexif
from datetime import datetime
import os

# Output directory
os.makedirs("captured_images", exist_ok=True)

# Function to generate EXIF bytes
def create_exif_bytes(camera_name):
    timestamp = datetime.now().strftime('%Y:%m:%d %H:%M:%S')
    exif_dict = {
        "0th": {
            piexif.ImageIFD.Make: camera_name.encode("utf-8"),
            piexif.ImageIFD.Model: b"PiCamera",
            piexif.ImageIFD.Software: b"picamera2",
            piexif.ImageIFD.DateTime: timestamp.encode("utf-8"),
        },
        "Exif": {
            piexif.ExifIFD.DateTimeOriginal: timestamp.encode("utf-8"),
            piexif.ExifIFD.LensMake: b"Raspberry Pi Foundation",
        },
    }
    return piexif.dump(exif_dict)

# Initialize each camera by index (0 and 1)
def capture_from_camera(index, name):
    cam = Picamera2(index)
    config = cam.create_still_configuration(main={"size": (1280, 960)}, buffer_count=1)
    cam.configure(config)
    cam.start()
    time.sleep(1.0)  # Let camera warm up

    image = cam.capture_array()
    cam.stop()

    img = Image.fromarray(image)
    zexif_bytes = create_exif_bytes(name)
    filename = f"captured_images/{name}_{int(time.time())}.jpg"
    img.save(filename, "jpeg", exif=exif_bytes)
    print(f"Saved {filename}")

# Capture from both cameras
capture_from_camera(0, "CameraA")
capture_from_camera(1, "CameraB")
time.sleep(1)
