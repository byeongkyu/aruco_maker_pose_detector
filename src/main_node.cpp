#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <vector>
#include <math.h>
#include <angles/angles.h>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::placeholders;

class ArucoMakerPCLDetector: public rclcpp::Node
{
    public:
        ArucoMakerPCLDetector() : Node("aruco_marker_pcl_detector")
        {
            aruco_marker_length_ = this->declare_parameter<double>("marker_length", 0.025);
            pub_marker_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", 10);

            sub_color_image_.subscribe(this, "/camera/color/image_raw");
            sub_depth_image_.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
            sub_depth_camera_info_.subscribe(this, "/camera/aligned_depth_to_color/camera_info");

            sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(
                            sub_color_image_, sub_depth_image_, sub_depth_camera_info_, 10);
            sync_->registerCallback(std::bind(&ArucoMakerPCLDetector::image_callback, this, _1, _2, _3));

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }
        ~ArucoMakerPCLDetector() {}

    private:
        void image_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& img, const sensor_msgs::msg::Image::ConstSharedPtr& img2, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
        {
            cv_bridge::CvImagePtr img_color;
            cv_bridge::CvImagePtr img_depth;

            try
            {
                img_color = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
                img_depth = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch (cv_bridge::Exception & e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid frame. Exception from cv_bridge: %s", e.what());
                return;
            }

            // Detect marker ARUCO Marker
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            cv::aruco::detectMarkers(img_color->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


            if(markerIds.size() == 0)
            {
                RCLCPP_WARN(this->get_logger(), "No marker detected...");
                return;
            }

            double center_x = 0, center_y = 0;
            for(size_t i = 0; i < markerCorners[0].size(); i++)
            {
                center_x += markerCorners[0][i].x;
                center_y += markerCorners[0][i].y;
            }
            center_x = center_x / markerCorners[0].size();
            center_y = center_y / markerCorners[0].size();
            cv::circle(img_color->image, cv::Point2f(center_x, center_y), 10, cv::Scalar(255, 0, 0, 0), 1);

            std::vector<cv::Point3f> pts(4);
            for(size_t i = 0; i < markerCorners[0].size(); i++)
            {
                cv::Point2f pt(markerCorners[0][i].x, markerCorners[0][i].y);

                cv::Mat subpixel_patch;
                cv::remap(img_depth->image, subpixel_patch, cv::Mat(1, 1, CV_32FC2, &pt), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
                uint16_t z = subpixel_patch.at<uint16_t>(0, 0);

                pts[i].z = z / 1000.0;
                pts[i].x = pts[i].z * (pt.x - info->k[2]) / info->k[0];
                pts[i].y = pts[i].z * (pt.y - info->k[5]) / info->k[4];

                RCLCPP_WARN(this->get_logger(), "%f %f %f", pts[i].x, pts[i].y, pts[i].z);

                if(i == 1)
                {
                    cv::circle(img_color->image, pt, 10, cv::Scalar(255, 0, 0, 0), 1);
                }
            }

            // cv::aruco::drawDetectedMarkers(img_color->image, markerCorners, markerIds);
            // cv::imshow("Display window", img_color->image);
            // cv::waitKey(1);

            /*
            pts[0] left top
            pts[1] right top
            pts[2] right bottom
            pts[3] left bottom
            */

            // Get center point
            cv::Point3f center_pt(0.0, 0.0, 0.0);
            cv::Point2f pt_center(center_x, center_y);

            cv::Mat subpixel_patch2;
            cv::remap(img_depth->image, subpixel_patch2, cv::Mat(1, 1, CV_32FC2, &pt_center), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
            uint16_t z_center = subpixel_patch2.at<uint16_t>(0, 0);

            center_pt.z = z_center / 1000.0;
            center_pt.x = center_pt.z * (pt_center.x - info->k[2]) / info->k[0];
            center_pt.y = center_pt.z * (pt_center.y - info->k[5]) / info->k[4];


            // Pose estimation
            double pitch1 = angles::normalize_angle(M_PI - atan2(pts[0].z - pts[1].z, pts[0].x - pts[1].x));
            double pitch2 = angles::normalize_angle(M_PI - atan2(pts[3].z - pts[2].z, pts[3].x - pts[2].x));

            double roll1 = angles::normalize_angle(M_PI - atan2(pts[2].z - pts[1].z, pts[2].y - pts[1].y));
            double roll2 = angles::normalize_angle(M_PI - atan2(pts[3].z - pts[0].z, pts[3].y - pts[0].y));

            double yaw1 = angles::normalize_angle(M_PI + atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x));
            double yaw2 = angles::normalize_angle(M_PI + atan2(pts[2].y - pts[3].y, pts[2].x - pts[3].x));

            // RCLCPP_WARN(this->get_logger(), "%f %f", pitch1, pitch2);


            // Publish result
            auto pose_msg = geometry_msgs::msg::PoseStamped();

            pose_msg.header.frame_id = info->header.frame_id;
            pose_msg.header.stamp = this->get_clock()->now();

            pose_msg.pose.position.x = center_pt.x;
            pose_msg.pose.position.y = center_pt.y;
            pose_msg.pose.position.z = center_pt.z;

            tf2::Quaternion q1, q2, q3, q_rot, q_new;
            // q_new.setRPY(0, 0, 0);
            q1.setRPY(0.0, (pitch1 + pitch2) / 2.0, 0.0);
            q2.setRPY((roll1 + roll2) / 2.0, 0.0, 0.0);
            q3.setRPY(0.0, 0.0, (yaw1 + yaw2) / 2.0);

            q_rot.setRPY(M_PI/2, -M_PI/2, 0);
            q_new = q1 * q_rot;// * q2 * q3;// * q_rot; // rotate to original frame
            q_new.normalize();

            pose_msg.pose.orientation.x = q_new.getX();
            pose_msg.pose.orientation.y = q_new.getY();
            pose_msg.pose.orientation.z = q_new.getZ();
            pose_msg.pose.orientation.w = q_new.getW();

            pub_marker_pose_->publish(pose_msg);
        }

    private:
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_color_image_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_image_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_depth_camera_info_;
        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> sync_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_marker_pose_;
        double aruco_marker_length_;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMakerPCLDetector>());
    rclcpp::shutdown();
    return 0;
}
