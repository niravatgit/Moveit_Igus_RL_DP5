#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tf2_ros
import geometry_msgs.msg
import tf.transformations

class ArucoPoseEstimator:
    def __init__(self):
        rospy.init_node('aruco_pose_estimator')
        
        # Camera matrix (from your original code)
        self.matrix_coefficients = np.array([
            [206.6461181640625, 0, 110.93448638916016],
            [0, 206.6461181640625, 83.46406555175781],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.distortion_coefficients = np.array([
            0.34155794978141785, -1.1316627264022827, 
            0.00038912298623472, -0.0017616129480302334,
            0.6371225118637085
        ], dtype=np.float32)
        
        # Try multiple dictionaries including 4x4 which works better for small markers
        self.aruco_dicts = [
            (cv2.aruco.DICT_ARUCO_ORIGINAL, "DICT_ARUCO_ORIGINAL")
        ]
        
        # Start with a larger size and adjust if needed
        self.marker_size = 0.05  # 5cm
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Store transformation matrix for marker ID 1
        self.marker_1_transform = None
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/royale_cam0/gray_image", Image, self.image_callback
        )
        
        rospy.loginfo("ArUco Pose Estimator initialized")
    
    def super_enhance(self, img):
        """Extreme preprocessing for the extremely low contrast depth camera"""
        # Step 1: Normalize to full range
        normalized = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        
        # Step 2: Apply CLAHE with stronger parameters
        clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4,4))
        enhanced = clahe.apply(normalized)

        # Step 3: Apply sharpening
        kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpened = cv2.filter2D(enhanced, -1, kernel)
        
        # Step 4: Create multiple threshold versions
        _, binary_otsu = cv2.threshold(sharpened, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # Step 5: Try multi-threshold approach
        thresh_images = []
        for thresh in range(50, 200, 30):
            _, binary = cv2.threshold(sharpened, thresh, 255, cv2.THRESH_BINARY)
            thresh_images.append(("Binary-"+str(thresh), binary))
        
        # Step 6: Adaptive thresholding with different window sizes
        adaptive_small = cv2.adaptiveThreshold(sharpened, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                              cv2.THRESH_BINARY, 7, 3)
        
        adaptive_mid = cv2.adaptiveThreshold(sharpened, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                           cv2.THRESH_BINARY, 11, 2)
        
        # Combine all processed images
        processed = [
            ("Normalized", normalized),
            ("Enhanced", enhanced),
            ("Sharpened", sharpened),
            ("Otsu", binary_otsu),
            ("Adaptive-Small", adaptive_small),
            ("Adaptive-Mid", adaptive_mid),
        ]
        
        processed.extend(thresh_images)
        return processed

    def image_callback(self, msg):
        try:
            # Convert ROS image
            gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Log stats
            rospy.loginfo(f"Image shape: {gray.shape}, min: {np.min(gray)}, max: {np.max(gray)}")
            
            # Get enhanced versions
            processed_images = self.super_enhance(gray)
            
            # Create color frame for visualization
            frame = cv2.cvtColor(processed_images[0][1], cv2.COLOR_GRAY2BGR)
            
            marker_found = False
            
            # Try each processed image and dictionary until a marker is found
            for proc_name, proc_img in processed_images:
                if marker_found:
                    break
                
                for dict_id, dict_name in self.aruco_dicts:
                    # Setup detector with custom parameters
                    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                    parameters = cv2.aruco.DetectorParameters()
                    
                    # CRITICAL: Enable corner refinement
                    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                    parameters.cornerRefinementWinSize = 5
                    parameters.cornerRefinementMaxIterations = 30
                    parameters.cornerRefinementMinAccuracy = 0.01
                    
                    # Very permissive parameters for detection
                    parameters.adaptiveThreshConstant = 7
                    parameters.adaptiveThreshWinSizeMin = 3
                    parameters.adaptiveThreshWinSizeMax = 23
                    parameters.adaptiveThreshWinSizeStep = 2
                    
                    # Extremely permissive marker parameters for tiny or distorted markers
                    parameters.minMarkerPerimeterRate = 0.01  # Very small markers
                    parameters.maxMarkerPerimeterRate = 4.0   # Very large markers
                    parameters.polygonalApproxAccuracyRate = 0.1  # More lenient corner detection
                    parameters.minCornerDistanceRate = 0.05
                    parameters.minOtsuStdDev = 1.0  # More lenient for low contrast
                    parameters.perspectiveRemoveIgnoredMarginPerCell = 0.4
                    
                    # Try to detect markers
                    corners, ids, _ = cv2.aruco.detectMarkers(
                        proc_img, aruco_dict, parameters=parameters
                    )
                    
                    # If markers detected
                    if ids is not None and len(ids) > 0:
                        marker_found = True
                        
                        # Update visualization frame with the detected image
                        if proc_name != "Normalized":
                            frame = cv2.cvtColor(proc_img, cv2.COLOR_GRAY2BGR)
                        
                        rospy.loginfo(f"SUCCESS! Marker detected with {proc_name} and {dict_name}")
                        
                        # Draw markers
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        
                        for i in range(len(ids)):
                            # Estimate pose
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                                corners[i], self.marker_size, 
                                self.matrix_coefficients, self.distortion_coefficients
                            )
                            
                            # Draw axis
                            cv2.drawFrameAxes(
                                frame, self.matrix_coefficients, self.distortion_coefficients, 
                                rvec, tvec, 0.03
                            )
                            
                            # Convert rotation vector to quaternion
                            rotation_matrix, _ = cv2.Rodrigues(rvec)
                            
                            # Create transformation matrix
                            transformation = np.eye(4)
                            transformation[:3, :3] = rotation_matrix
                            transformation[:3, 3] = tvec.flatten()
                            
                            # Store transformation if marker ID is 1
                            if ids[i][0] == 1:
                                self.marker_1_transform = transformation
                                rospy.loginfo("\n\nFINAL TRANSFORMATION MATRIX FOR MARKER ID 1:")
                                rospy.loginfo(f"\n{transformation}\n")
                            
                            # Convert to quaternion
                            quaternion = tf.transformations.quaternion_from_matrix(transformation)
                            
                            # Create transform message
                            transform = geometry_msgs.msg.TransformStamped()
                            transform.header.stamp = rospy.Time.now()
                            transform.header.frame_id = "royale_camera_0_optical_frame"
                            transform.child_frame_id = f"aruco_marker_{ids[i][0]}"
                            
                            # Set translation
                            transform.transform.translation.x = tvec[0][0][0]
                            transform.transform.translation.y = tvec[0][0][1]
                            transform.transform.translation.z = tvec[0][0][2]
                            
                            # Set rotation
                            transform.transform.rotation.x = quaternion[0]
                            transform.transform.rotation.y = quaternion[1]
                            transform.transform.rotation.z = quaternion[2]
                            transform.transform.rotation.w = quaternion[3]
                            
                            # Broadcast transform
                            self.tf_broadcaster.sendTransform(transform)
                            
                            # Log transformation
                            rospy.loginfo(f"Marker ID: {ids[i][0]}")
                            rospy.loginfo(f"Position: {tvec[0][0].round(5)}")
                            rospy.loginfo(f"Rotation: {rvec[0][0].round(5)}")
                            rospy.loginfo(f"Transform matrix:\n{transformation}")
                        
                        break
            
            # Display images
            cv2.imshow("Original", processed_images[0][1])
            cv2.imshow("Enhanced", processed_images[1][1])
            cv2.imshow("Detection", frame)
            
            # Also show binary version that worked best
            if marker_found:
                cv2.imshow("Successful Method", proc_img)
            else:
                cv2.imshow("Binary", processed_images[3][1])
                rospy.logwarn("No markers detected")
            
            cv2.waitKey(1)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    try:
        estimator = ArucoPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
