#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import math


class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')

        # Publishers for arm and gripper
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)

        self.get_logger().info("🖐️ Gesture Control Node Started (Peace = Home)")

        # Initialize Mediapipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Open laptop camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot access webcam!")
            rclpy.shutdown()
            return

        # Track last published gesture (debounce)
        self.last_published = None
        self.overlay_text = ""

        # Timer to read frames continuously (~20 FPS)
        self.timer = self.create_timer(0.05, self.process_frame)

    # ------------------------------------------------------------------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to read frame from camera")
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture = "None"

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )

                gesture, thumb_px, index_px = self.detect_gesture(hand_landmarks, frame.shape)

                cv2.circle(frame, thumb_px, 10, (0, 255, 0), -1)  # Thumb
                cv2.circle(frame, index_px, 10, (0, 0, 255), -1)  # Index

                self.publish_commands_for_gesture(gesture)

        # Draw overlay and text
        cv2.putText(frame, f"Gesture: {gesture}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        if self.overlay_text:
            cv2.putText(frame, self.overlay_text, (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow("Gesture Control", frame)

        # Exit on ESC key
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("🛑 Exiting Gesture Control Node")
            self.cleanup()

    # ------------------------------------------------------------------
    def detect_gesture(self, landmarks, frame_shape):
        """Detect gestures and return pixel coordinates for thumb + index finger tips."""
        h, w, _ = frame_shape
        lm = landmarks.landmark

        # Fingertips
        thumb_tip = lm[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = lm[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = lm[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = lm[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = lm[self.mp_hands.HandLandmark.PINKY_TIP]
        wrist = lm[self.mp_hands.HandLandmark.WRIST]

        # MCP joints for extension detection
        index_mcp = lm[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        middle_mcp = lm[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        ring_mcp = lm[self.mp_hands.HandLandmark.RING_FINGER_MCP]
        pinky_mcp = lm[self.mp_hands.HandLandmark.PINKY_MCP]

        thumb_px = (int(thumb_tip.x * w), int(thumb_tip.y * h))
        index_px = (int(index_tip.x * w), int(index_tip.y * h))

        # Determine which fingers are extended
        index_extended = index_tip.y < index_mcp.y
        middle_extended = middle_tip.y < middle_mcp.y
        ring_extended = ring_tip.y < ring_mcp.y
        pinky_extended = pinky_tip.y < pinky_mcp.y

        # Distances
        index_middle_dist = math.hypot(index_tip.x - middle_tip.x, index_tip.y - middle_tip.y)
        thumb_index_dist = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)

        # Thresholds
        PINCH_THRESH = 0.08
        SPREAD_THRESH = 0.20

        # --- Gesture logic ---
        # ✌️ Peace sign → index and middle extended, others folded, and visible gap
        if (index_extended and middle_extended and not ring_extended and not pinky_extended
                and index_middle_dist > 0.05):
            gesture = "Peace"

        # ✋ Open hand → all fingers extended
        elif index_extended and middle_extended and ring_extended and pinky_extended and thumb_index_dist > SPREAD_THRESH:
            gesture = "OpenHand"

        # 🤏 Pinch → thumb and index close
        elif thumb_index_dist < PINCH_THRESH:
            gesture = "Pinch"

        # 👉 Index pointing → only index extended
        elif index_extended and not middle_extended and not ring_extended and not pinky_extended:
            gesture = "IndexPoint"

        else:
            gesture = "Neutral"

        return gesture, thumb_px, index_px

    # ------------------------------------------------------------------
    def publish_commands_for_gesture(self, gesture):
        """
        Gesture mappings:
          ✌️ Peace → /arm_command 'home'
          ✋ Open → /arm_command 'stand'
          🤏 Pinch → /gripper_command 'close'
          👉 Point → /gripper_command 'open'
        """
        if gesture == self.last_published:
            return

        self.last_published = gesture
        arm_msg = String()
        gripper_msg = String()

        if gesture == "Peace":
            arm_msg.data = "home"
            self.arm_pub.publish(arm_msg)
            self.overlay_text = "✌️ Moving arm to HOME"
            self.get_logger().info("✌️ Peace detected → /arm_command 'home'")

        elif gesture == "OpenHand":
            arm_msg.data = "stand"
            self.arm_pub.publish(arm_msg)
            self.overlay_text = "✋ Moving arm to STAND"
            self.get_logger().info("✋ Open hand → /arm_command 'stand'")

        elif gesture == "Pinch":
            gripper_msg.data = "close"
            self.gripper_pub.publish(gripper_msg)
            self.overlay_text = "🤏 Closing gripper"
            self.get_logger().info("🤏 Pinch → /gripper_command 'close'")

        elif gesture == "IndexPoint":
            gripper_msg.data = "open"
            self.gripper_pub.publish(gripper_msg)
            self.overlay_text = "👉 Opening gripper"
            self.get_logger().info("👉 Index point → /gripper_command 'open'")

        else:
            self.overlay_text = ""

    # ------------------------------------------------------------------
    def cleanup(self):
        """Release camera and close windows."""
        self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()
