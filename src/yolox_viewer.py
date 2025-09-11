#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class YoloxViewer(Node):
    def __init__(self):
        super().__init__('yolox_viewer')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.window_name = 'YOLOX Detection'
        
        # Create subscribers with time synchronizer
        self.image_sub = Subscriber(self, Image, '/image_raw')
        self.det_sub = Subscriber(self, Detection2DArray, '/yolox/bounding_boxes')
        
        # Synchronize messages
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.det_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.callback)
        
        self.get_logger().info('YOLOX Viewer started')
        self.get_logger().info('Press ESC to exit')
        
        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
    def callback(self, image_msg, det_msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Draw detections
            for detection in det_msg.detections:
                # Get bounding box
                bbox = detection.bbox
                x = int(bbox.center.position.x - bbox.size_x/2)
                y = int(bbox.center.position.y - bbox.size_y/2)
                w = int(bbox.size_x)
                h = int(bbox.size_y)
                
                # Draw rectangle
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Get class and score
                if detection.results:
                    result = detection.results[0]
                    if hasattr(result, 'hypothesis') and result.hypothesis:
                        class_name = result.hypothesis.class_id
                        score = result.hypothesis.score
                    else:
                        class_name = "Unknown"
                        score = result.score if hasattr(result, 'score') else 0.0
                    
                    label = f"{class_name}: {score:.2f}"
                    cv2.putText(cv_image, label, (x, y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add info overlay
            info_text = f"Detections: {len(det_msg.detections)}"
            cv2.putText(cv_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display image
            cv2.imshow(self.window_name, cv_image)
            
            # Check for ESC key
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info('ESC pressed, shutting down...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = YoloxViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
