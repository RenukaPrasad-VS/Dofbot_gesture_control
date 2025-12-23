#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import mediapipe as mp
import numpy as np

class HandTracker(Node):
    def __init__(self):
        super().__init__('hand_tracker')

        # ROS2 publisher
        self.publisher_ = self.create_publisher(Point, '/hand_position', 10)

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot open webcam.")
            rclpy.shutdown()
            return

        # Initialize Mediapipe Hands
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6
        )

        self.get_logger().info("🖐 Hand Tracker Node Started (tracking index fingertip)")

        # Timer for continuous tracking (~20 Hz)
        self.timer = self.create_timer(0.05, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ No frame captured from webcam.")
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        h, w, _ = frame.shape

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                lm = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]

                # Normalized to [-1, 1]
                norm_x = (lm.x - 0.5) * 2.0
                norm_y = (0.5 - lm.y) * 2.0
                norm_z = -lm.z * 2.0

                # Scale to robot workspace
                # Dofbot reachable region ≈ X:[0.1–0.3], Y:[-0.15–0.15], Z:[0.05–0.25]
                x = np.clip(0.2 + norm_z * 0.1, 0.1, 0.3)
                y = np.clip(norm_x * 0.15, -0.15, 0.15)
                z = np.clip(0.15 + norm_y * 0.1, 0.05, 0.25)

                msg = Point(x=float(x), y=float(y), z=float(z))
                self.publisher_.publish(msg)

                cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 10, (0, 255, 0), -1)
                cv2.putText(frame, f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Hand Tracker", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            self.cleanup()

    def cleanup(self):
        self.get_logger().info("🛑 Shutting down Hand Tracker...")
        self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()


if __name__ == "__main__":
    main()
