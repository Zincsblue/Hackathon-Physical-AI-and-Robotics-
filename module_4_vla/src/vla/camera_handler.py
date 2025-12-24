"""
Camera input handling for VLA system
"""

import cv2
import numpy as np
from typing import Tuple, Optional


class CameraHandler:
    """
    Handles camera input for the VLA system
    """

    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = None

    def initialize_camera(self):
        """
        Initialize the camera
        """
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self.camera_index}")

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def capture_image(self) -> Optional[np.ndarray]:
        """
        Capture a single image from the camera
        """
        if self.cap is None:
            self.initialize_camera()

        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture image from camera")
            return None

        return frame

    def capture_image_bytes(self) -> Optional[bytes]:
        """
        Capture an image and return as bytes
        """
        image = self.capture_image()
        if image is None:
            return None

        # Encode image as JPEG bytes
        success, encoded_image = cv2.imencode('.jpg', image)
        if not success:
            print("Failed to encode image")
            return None

        return encoded_image.tobytes()

    def get_frame_dimensions(self) -> Tuple[int, int]:
        """
        Get the current frame dimensions
        """
        if self.cap is None:
            self.initialize_camera()

        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    def release_camera(self):
        """
        Release the camera resource
        """
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __del__(self):
        """
        Destructor to ensure camera is released
        """
        self.release_camera()