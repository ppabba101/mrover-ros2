#include "perception.hpp"

// ROS Headers, ros namespace
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // "spin" blocks until our node dies
    rclcpp::spin(std::make_shared<mrover::Perception>());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() : Node("perception") {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = create_subscription<sensor_msgs::msg::Image>("zed/left/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = create_publisher<msg::StarterProjectTag>("tag", 1);
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

    auto Perception::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage) -> void {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat imageBGRA{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                      CV_8UC4, const_cast<uint8_t*>(imageMessage->data.data())};
        cv::Mat image;

        cv::cvtColor(imageBGRA, image, cv::COLOR_BGRA2BGR);

        // TODO: implement me!
        // hint: think about the order in which these functions were implemented ;)
        // Clear the previous tag data
        mTags.clear();

        // Detect tags in the image
        findTagsInImage(image, mTags);

        // If tags were found, select the closest one to the center
        if (!mTags.empty()) {
            // Select the closest tag
            msg::StarterProjectTag closestTag = selectTag(image, mTags);

            // Find the corresponding corners of the selected tag (using its ID or index)
            auto it = std::find(mTagIds.begin(), mTagIds.end(), closestTag.tag_id);
            if (it != mTagIds.end()) {
                size_t index = std::distance(mTagIds.begin(), it);

                // Get the tag corners for the selected tag
                std::vector<cv::Point2f> const& tagCorners = mTagCorners[index];

                // Publish the updated closest tag
                publishTag(closestTag);
            } else {
                RCLCPP_WARN(this->get_logger(), "No corners found for the closest tag.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No tags detected in the current frame.");
        }
    }

    auto Perception::findTagsInImage(cv::Mat const& image, std::vector<msg::StarterProjectTag>& tags) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, and mTagIds member variables already defined! (look in perception.hpp)
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!
        mTagCorners.clear();
        mTagIds.clear();
    
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

        // Detect ArUco markers
        cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), detectorParams);
        detector.detectMarkers(image, mTagCorners, mTagIds);

        // Iterate over detected markers and create StarterProjectTag messages
        for (size_t i = 0; i < mTagIds.size(); i++) {
            msg::StarterProjectTag tag;
            tag.tag_id = mTagIds[i];
            auto center = getCenterFromTagCorners(mTagCorners[i]);
            tag.x_tag_center_pixel = center.first;
            tag.y_tag_center_pixel = center.second;
            tag.closeness_metric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            tags.push_back(tag);
        }

    }

    auto Perception::selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float imageCenterX = image.cols / 2.0;
        float imageCenterY = image.rows / 2.0;

        msg::StarterProjectTag closestTag;
        float minDistance = std::numeric_limits<float>::max();

        // Find the tag closest to the center
        for (auto const& tag : tags) {
            float dx = tag.x_tag_center_pixel - imageCenterX;
            float dy = tag.y_tag_center_pixel - imageCenterY;
            float distanceToCenter = std::sqrt(dx * dx + dy * dy);

            if (distanceToCenter < minDistance) {
                minDistance = distanceToCenter;
                closestTag = tag;
            }
        }
        return closestTag;
    }

    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        // TODO: implement me!
        mTagPublisher->publish(tag);

    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        // Approximate closeness by calculating the perimeter of the tag
        float perimeter = 0.0;
        for (size_t i = 0; i < 4; i++) {
            cv::Point2f pt1 = tagCorners[i];
            cv::Point2f pt2 = tagCorners[(i + 1) % 4];
            perimeter += std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
        }
        return perimeter;
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float centerX = 0.0, centerY = 0.0;
        for (auto const& corner : tagCorners) {
            centerX += corner.x;
            centerY += corner.y;
        }
        centerX /= 4.0;
        centerY /= 4.0;
        return {centerX, centerY};
    }
} // namespace mrover