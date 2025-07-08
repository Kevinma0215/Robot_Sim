#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class LineDetector {
public:
    LineDetector() : nh_("~"), it_(nh_)
    {
        // Initialize parameters
        loadParameters();
        
        // Publishers
        error_pub_ = nh_.advertise<std_msgs::Float64>("/line_error", 10);
        
        // Subscribers
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &LineDetector::imageCallback, this);
        
        // Debug publisher (optional)
        if (debug_mode_)
        {
            debug_pub_ = it_.advertise("/line_detector/debug_image", 1);
            ROS_INFO("Debug mode enabled - publishing debug images");
        }
        
        ROS_INFO("Line detector initialized");
    }
    
    ~LineDetector()
    {
        if (debug_mode_)
        {
            cv::destroyAllWindows();
        }
    }

private:
    // ROS handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_pub_;
    ros::Publisher error_pub_;
    
    // Parameters
    bool debug_mode_;
    double roi_height_ratio_;
    cv::Scalar lower_white_;
    cv::Scalar upper_white_;
    
    // Image processing variables
    int image_width_;
    int image_height_;
    
    void loadParameters() {
        // Load parameters from ROS parameter server
        nh_.param("debug", debug_mode_, false);
        nh_.param("roi_height_ratio", roi_height_ratio_, 0.4);
        
        // HSV thresholds for white line detection
        std::vector<int> lower_white_vec = {0, 0, 200};
        std::vector<int> upper_white_vec = {180, 30, 255};
        
        nh_.param("lower_white", lower_white_vec, lower_white_vec);
        nh_.param("upper_white", upper_white_vec, upper_white_vec);
        
        // Convert to OpenCV Scalar
        lower_white_ = cv::Scalar(lower_white_vec[0], lower_white_vec[1], lower_white_vec[2]);
        upper_white_ = cv::Scalar(upper_white_vec[0], upper_white_vec[1], upper_white_vec[2]);
        
        ROS_INFO("Parameters loaded - Debug: %s, ROI ratio: %.2f", 
                 debug_mode_ ? "true" : "false", roi_height_ratio_);
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Store image dimensions
            image_height_ = image.rows;
            image_width_ = image.cols;
            
            // Process image to detect line
            double error = detectLine(image);
            
            // Publish error signal
            std_msgs::Float64 error_msg;
            error_msg.data = error;
            error_pub_.publish(error_msg);
            
            // Debug visualization
            if (debug_mode_)
            {
                visualizeDetection(image, error);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("CV bridge exception: %s", e.what());
        }
    }
    
    double detectLine(const cv::Mat& image)
    {
        // 1. Define Region of Interest (ROI)
        int roi_start = static_cast<int>(image_height_ * (1.0 - roi_height_ratio_));
        cv::Rect roi_rect(0, roi_start, image_width_, image_height_ - roi_start);
        cv::Mat roi_image = image(roi_rect);
        
        // 2. Convert to HSV for better color detection
        cv::Mat hsv;
        cv::cvtColor(roi_image, hsv, cv::COLOR_BGR2HSV);
        
        // 3. Create mask for white pixels
        cv::Mat mask;
        cv::inRange(hsv, lower_white_, upper_white_, mask);
        
        // 4. Apply morphological operations to clean up the mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        
        // 5. Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 6. Calculate line centroid
        double line_center_x = -1;
        double normalized_error = 0.0;
        
        if (!contours.empty())
        {
            // Find the largest contour (assuming it's the line)
            double max_area = 0;
            int largest_contour_idx = 0;
            
            for (size_t i = 0; i < contours.size(); ++i)
            {
                double area = cv::contourArea(contours[i]);
                if (area > max_area)
                {
                    max_area = area;
                    largest_contour_idx = i;
                }
            }
            
            // Calculate centroid of largest contour
            cv::Moments M = cv::moments(contours[largest_contour_idx]);
            if (M.m00 != 0)
            {
                line_center_x = M.m10 / M.m00;
                
                // Calculate normalized error
                double image_center_x = image_width_ / 2.0;
                double pixel_error = line_center_x - image_center_x;
                
                // Normalize to [-1, 1] range
                normalized_error = pixel_error / (image_width_ / 2.0);
                
                // Clamp to [-1, 1] range
                normalized_error = std::max(-1.0, std::min(1.0, normalized_error));
                
                if (debug_mode_)
                {
                    ROS_INFO_THROTTLE(1.0, "Line center: %.1f, Error: %.3f", line_center_x, normalized_error);
                }
            }
        }
        else
        {
            if (debug_mode_)
            {
                ROS_WARN_THROTTLE(2.0, "No line detected!");
            }
        }
        
        return normalized_error;
    }
    
    void visualizeDetection(const cv::Mat& original, double error)
    {
        cv::Mat debug_image = original.clone();
        
        // Draw ROI rectangle
        int roi_start = static_cast<int>(image_height_ * (1.0 - roi_height_ratio_));
        cv::rectangle(debug_image, cv::Point(0, roi_start), cv::Point(image_width_, image_height_), 
                     cv::Scalar(0, 255, 0), 2);
        
        // Draw image center reference
        cv::circle(debug_image, cv::Point(image_width_ / 2, image_height_ / 2), 5, cv::Scalar(255, 0, 0), -1);
        
        // Draw error text
        std::string error_text = "Error: " + std::to_string(error);
        cv::putText(debug_image, error_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        
        // Show image
        cv::imshow("Line Detection Debug", debug_image);
        cv::waitKey(1);
        
        // Publish debug image
        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
        debug_pub_.publish(debug_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_detector");
    LineDetector detector;
    
    ROS_INFO("Line detector node started");
    ros::spin();
    
    return 0;
}