#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import time

class LineDetectorTester:
    def __init__(self):
        rospy.init_node('line_detector_tester')
        
        # Subscribers
        self.error_sub = rospy.Subscriber('/line_error', Float64, self.error_callback)
        self.image_sub = rospy.Subscriber('/my_bot/camera1/image_raw', Image, self.image_callback)
        
        # Test variables
        self.error_values = []
        self.image_received = False
        self.error_received = False
        self.start_time = time.time()
        
        print("=== Line Detector Test Started ===")
        print("Testing for 10 seconds...")
        
    def error_callback(self, msg):
        self.error_received = True
        self.error_values.append(msg.data)
        
        # Print every 10th error value to avoid spam
        if len(self.error_values) % 10 == 0:
            print(f"Error: {msg.data:.3f}")
    
    def image_callback(self, msg):
        self.image_received = True
        
    def run_test(self):
        # Wait for initial data
        print("Waiting for data...")
        rospy.sleep(2.0)
        
        # Check if we're receiving data
        if not self.image_received:
            print("❌ ERROR: No camera images received!")
            print("   Check if camera topic is publishing: rostopic hz /camera/image_raw")
            return False
            
        if not self.error_received:
            print("❌ ERROR: No line error data received!")
            print("   Check if line detector is running: rosnode list | grep line")
            return False
            
        print("✅ Receiving camera images and error data")
        
        # Collect data for analysis
        test_duration = 10.0
        rospy.sleep(test_duration)
        
        # Analyze results
        self.analyze_results()
        return True
        
    def analyze_results(self):
        if len(self.error_values) < 10:
            print("❌ ERROR: Not enough error data collected")
            return
            
        # Calculate statistics
        avg_error = sum(self.error_values) / len(self.error_values)
        max_error = max(self.error_values)
        min_error = min(self.error_values)
        
        print("\n=== Test Results ===")
        print(f"Total error samples: {len(self.error_values)}")
        print(f"Average error: {avg_error:.3f}")
        print(f"Error range: [{min_error:.3f}, {max_error:.3f}]")
        
        # Check for reasonable values
        if abs(avg_error) > 1.0:
            print("⚠️  WARNING: Average error > 1.0 - check normalization")
        
        if max_error > 1.0 or min_error < -1.0:
            print("⚠️  WARNING: Error values outside [-1, 1] range")
            
        # Check for line detection
        if all(abs(e) < 0.01 for e in self.error_values[-20:]):
            print("⚠️  WARNING: Error consistently near zero - line might not be detected")
            print("   Try adjusting HSV thresholds or ROI parameters")
        else:
            print("✅ Line detection appears to be working")
            
        # Check for variability (indicates responsive detection)
        error_range = max_error - min_error
        if error_range > 0.1:
            print("✅ Good error variability - detector is responsive")
        else:
            print("⚠️  Low error variability - detector might be too stable")

if __name__ == '__main__':
    try:
        tester = LineDetectorTester()
        success = tester.run_test()
        
        if success:
            print("\n=== Manual Tests to Try ===")
            print("1. Move robot left/right and watch error values")
            print("2. Check debug image windows for visual feedback")
            print("3. Use: rostopic echo /line_error")
            print("4. Adjust lighting in Gazebo world")
            print("5. Test with different line colors/widths")
        
    except rospy.ROSInterruptException:
        print("\nTest interrupted")
    except Exception as e:
        print(f"Test failed: {e}")