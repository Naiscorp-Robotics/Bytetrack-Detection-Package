#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bboxes_ex_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class YoloxVisualizer(Node):
    def __init__(self):
        super().__init__('yolox_visualizer')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('bbox_topic', '/yolox/bounding_boxes')
        self.declare_parameter('window_name', 'YOLOX Detection Visualization')
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.window_name = self.get_parameter('window_name').value
        
        # Create subscribers with time synchronizer
        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.bbox_sub = Subscriber(self, BoundingBoxes, self.bbox_topic)
        
        # Synchronize messages
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.bbox_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.callback)
        
        self.get_logger().info(f'YOLOX Visualizer started')
        self.get_logger().info(f'Subscribing to image: {self.image_topic}')
        self.get_logger().info(f'Subscribing to bboxes: {self.bbox_topic}')
        
        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
    def callback(self, image_msg, bbox_msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Draw bounding boxes
            for bbox in bbox_msg.bounding_boxes:
                # Draw bounding box
                x1 = int(bbox.xmin)
                y1 = int(bbox.ymin)
                x2 = int(bbox.xmax)
                y2 = int(bbox.ymax)
                
                # Use green color for detections
                color = (0, 255, 0)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                
                # Prepare label
                label = f"{bbox.class_id}"
                if hasattr(bbox, 'probability') and bbox.probability > 0:
                    label = f"{label} ({bbox.probability:.2f})"
                
                # Draw label
                cv2.putText(cv_image, label, 
                          (x1, y1 - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Add info overlay
            info_text = f"Detected: {len(bbox_msg.bounding_boxes)} objects"
            cv2.putText(cv_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display image
            cv2.imshow(self.window_name, cv_image)
            
            # Check for ESC key
            if cv2.waitKey(1) & 0xFF == 27:
                self.get_logger().info('ESC pressed, shutting down...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = YoloxVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
