#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bboxes_ex_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
import colorsys

class ByteTrackVisualizer(Node):
    def __init__(self):
        super().__init__('bytetrack_visualizer')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('bbox_topic', '/bytetrack/bounding_boxes')
        self.declare_parameter('window_name', 'ByteTrack Realtime Visualization')
        self.declare_parameter('save_video', False)
        self.declare_parameter('output_file', 'tracking_output.mp4')
        self.declare_parameter('fps', 30)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.save_video = self.get_parameter('save_video').value
        self.output_file = self.get_parameter('output_file').value
        self.fps = self.get_parameter('fps').value
        
        # Video writer
        self.video_writer = None
        self.frame_size = None
        
        # Create subscribers with time synchronizer
        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.bbox_sub = Subscriber(self, BoundingBoxes, self.bbox_topic)
        
        # Synchronize messages
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.bbox_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.callback)
        
        # Color map for different track IDs
        self.color_map = {}
        
        self.get_logger().info(f'ByteTrack Visualizer started')
        self.get_logger().info(f'Subscribing to image: {self.image_topic}')
        self.get_logger().info(f'Subscribing to bboxes: {self.bbox_topic}')
        
        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
    def get_color_for_id(self, track_id):
        """Generate consistent color for each track ID"""
        if track_id not in self.color_map:
            # Generate a unique color using HSV color space
            hue = (track_id * 0.618033988749895) % 1.0  # Golden ratio
            rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
            self.color_map[track_id] = (int(rgb[2]*255), int(rgb[1]*255), int(rgb[0]*255))
        return self.color_map[track_id]
    
    def callback(self, image_msg, bbox_msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Initialize video writer if needed
            if self.save_video and self.video_writer is None:
                h, w = cv_image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(
                    self.output_file, fourcc, self.fps, (w, h)
                )
                self.get_logger().info(f'Saving video to: {self.output_file}')
            
            # Draw bounding boxes and track IDs
            for bbox in bbox_msg.bounding_boxes:
                # Get color for this track ID
                color = self.get_color_for_id(bbox.id)
                
                # Draw bounding box
                x1 = int(bbox.xmin)
                y1 = int(bbox.ymin)
                x2 = int(bbox.xmax)
                y2 = int(bbox.ymax)
                
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                
                # Prepare label with class name, track ID and confidence
                label = f"ID:{bbox.id}"
                if hasattr(bbox, 'class_id') and bbox.class_id:
                    label = f"{bbox.class_id} {label}"
                if hasattr(bbox, 'probability') and bbox.probability > 0:
                    label = f"{label} ({bbox.probability:.2f})"
                
                # Draw label background
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                label_y = y1 - 10 if y1 - 10 > 0 else y1 + label_size[1] + 10
                cv2.rectangle(cv_image, 
                            (x1, label_y - label_size[1] - 5),
                            (x1 + label_size[0], label_y + 5),
                            color, -1)
                
                # Draw label text
                cv2.putText(cv_image, label, 
                          (x1, label_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Draw center point for tracking
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                cv2.circle(cv_image, (center_x, center_y), 3, color, -1)
            
            # Add info overlay
            info_text = f"Tracking: {len(bbox_msg.bounding_boxes)} objects"
            cv2.putText(cv_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show timestamp
            timestamp = f"Time: {bbox_msg.header.stamp.sec}.{bbox_msg.header.stamp.nanosec//1000000:03d}"
            cv2.putText(cv_image, timestamp, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
            
            # Display image
            cv2.imshow(self.window_name, cv_image)
            
            # Save to video if enabled
            if self.save_video and self.video_writer:
                self.video_writer.write(cv_image)
            
            # Check for ESC key to exit
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info('ESC pressed, shutting down...')
                self.cleanup()
                rclpy.shutdown()
            elif key == ord('s'):  # 's' to save screenshot
                filename = f'screenshot_{bbox_msg.header.stamp.sec}.png'
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Screenshot saved: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')
    
    def cleanup(self):
        """Clean up resources"""
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info(f'Video saved to: {self.output_file}')
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = ByteTrackVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'visualizer' in locals():
            visualizer.cleanup()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
