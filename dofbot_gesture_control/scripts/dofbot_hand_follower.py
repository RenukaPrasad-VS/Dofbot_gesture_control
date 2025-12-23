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
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        self.done_sub = self.create_subscription(String, '/action_done', self.done_callback, 10)

        self.get_logger().info("🖐️ Gesture Control Node Started (Synchronized Mode)")

        # busy flag — prevents new gestures while robot is moving
        self.is_busy = False
        self.last_published = None

        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot access webcam!")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.05, self.process_frame)

    def done_callback(self, msg: String):
        """Called when robot finishes previous motion."""
        if msg.data.strip().lower() == "done":
            self.is_busy = False
            self.get_logger().info("✅ Robot completed previous action. Ready for next gesture.")

    def process_frame(self):
        if self.is_busy:
            # Skip gesture detection while robot is busy
            return

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
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                gesture, thumb_px, index_px = self.detect_gesture(hand_landmarks, frame.shape)
                cv2.circle(frame, thumb_px, 10, (0, 255, 0), -1)
                cv2.circle(frame, index_px, 10, (0, 0, 255), -1)
                self.publish_commands_for_gesture(gesture)

        cv2.putText(frame, f"Gesture: {gesture}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("Gesture Control", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("🛑 Exiting Gesture Control Node")
            self.cleanup()

    def detect_gesture(self, landmarks, frame_shape):
        h, w, _ = frame_shape
        lm = landmarks.landmark
        thumb_tip = lm[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = lm[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = lm[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = lm[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = lm[self.mp_hands.HandLandmark.PINKY_TIP]
        wrist = lm[self.mp_hands.HandLandmark.WRIST]

        index_mcp = lm[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        middle_mcp = lm[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        ring_mcp = lm[self.mp_hands.HandLandmark.RING_FINGER_MCP]
        pinky_mcp = lm[self.mp_hands.HandLandmark.PINKY_MCP]

        thumb_px = (int(thumb_tip.x * w), int(thumb_tip.y * h))
        index_px = (int(index_tip.x * w), int(index_tip.y * h))

        thumb_index_dist = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
        idx_wrist = math.hypot(index_tip.x - wrist.x, index_tip.y - wrist.y)
        mid_wrist = math.hypot(middle_tip.x - wrist.x, middle_tip.y - wrist.y)
        ring_wrist = math.hypot(ring_tip.x - wrist.x, ring_tip.y - wrist.y)
        pinky_wrist = math.hypot(pinky_tip.x - wrist.x, pinky_tip.y - wrist.y)

        index_extended = index_tip.y < index_mcp.y
        middle_extended = middle_tip.y < middle_mcp.y
        ring_extended = ring_tip.y < ring_mcp.y
        pinky_extended = pinky_tip.y < pinky_mcp.y

        ALL_FOLDED_DIST = 0.07
        SPREAD_THRESH = 0.20
        PINCH_THRESH = 0.08

        if idx_wrist < ALL_FOLDED_DIST and mid_wrist < ALL_FOLDED_DIST and ring_wrist < ALL_FOLDED_DIST and pinky_wrist < ALL_FOLDED_DIST:
            gesture = "ClosedHand"
        elif index_extended and middle_extended and ring_extended and pinky_extended and thumb_index_dist > SPREAD_THRESH:
            gesture = "OpenHand"
        elif thumb_index_dist < PINCH_THRESH:
            gesture = "Pinch"
        elif index_extended and not middle_extended and not ring_extended and not pinky_extended:
            gesture = "IndexPoint"
        else:
            gesture = "Neutral"

        return gesture, thumb_px, index_px

    def publish_commands_for_gesture(self, gesture):
        if gesture == self.last_published or self.is_busy:
            return

        arm_msg = String()
        gripper_msg = String()

        if gesture == "ClosedHand":
            arm_msg.data = "home"
            self.arm_pub.publish(arm_msg)
            self.is_busy = True
            self.get_logger().info("➡️ Gesture ClosedHand -> /arm_command 'home'")

        elif gesture == "OpenHand":
            arm_msg.data = "stand"
            self.arm_pub.publish(arm_msg)
            self.is_busy = True
            self.get_logger().info("➡️ Gesture OpenHand -> /arm_command 'stand'")

        elif gesture == "Pinch":
            gripper_msg.data = "close"
            self.gripper_pub.publish(gripper_msg)
            self.is_busy = True
            self.get_logger().info("➡️ Gesture Pinch -> /gripper_command 'close'")

        elif gesture == "IndexPoint":
            gripper_msg.data = "open"
            self.gripper_pub.publish(gripper_msg)
            self.is_busy = True
            self.get_logger().info("➡️ Gesture IndexPoint -> /gripper_command 'open'")

        self.last_published = gesture

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


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
