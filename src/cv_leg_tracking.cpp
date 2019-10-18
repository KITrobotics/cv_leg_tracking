#include <cv_leg_tracking/cv_leg_tracking.h>
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>


CvLegTracking::CvLegTracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer), it_(nh)
{
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    nh_.param("from_pc2_depth_image_topic", from_pc2_depth_image_topic, std::string("/cv_leg_tracking/from_pc2_depth_image"));
    nh_.param("cv_fs_image_id", cv_fs_image_id, std::string("avg_image"));
    nh_.param("depth_image_sub_topic", depth_image_sub_topic, std::string("/cv_leg_tracking/from_pc2_depth_image"));
    nh_.param("image_cols", C, 640);
    nh_.param("image_rows", R, 480);
    nh_.param("num_images_for_background", num_images_for_background, 3);
    nh_.param("person_distance", person_distance, 1.);
    nh_.param("min_distance_near_camera", min_distance_near_camera, 0.4);
    nh_.param("bad_point", bad_point, 9.9);

    std::string avg_image_path_param;
    nh_.param("avg_image_path", avg_image_path_param, std::string("background_image/avg_image.yml"));
    cv_leg_tracking_path = ros::package::getPath("cv_leg_tracking");
    avg_image_path = cv_leg_tracking_path + "/" + avg_image_path_param;

    pc2_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(topic_point_cloud, 1,
                                                             &CvLegTracking::processPointCloud2, this);
    depth_image_sub_ = it_.subscribe(depth_image_sub_topic, 1, &CvLegTracking::imageCb, this);
    sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, &CvLegTracking::cameraInfoCb, this);

    calculateAvgService_ = nh_.advertiseService("calculateAvg", &CvLegTracking::calculateAvgImageCb, this);
    subtract_image_pub_ = it_.advertise("/cv_leg_tracking/subtract_image", 1);
    working_image_pub_ = it_.advertise("/cv_leg_tracking/working_image", 1);
    avg_image_pub_ = it_.advertise("/cv_leg_tracking/avg_image", 1);
    erosion_image_pub_ = it_.advertise("/cv_leg_tracking/erosion_image", 1);
    from_pc2_depth_image_pub_ = it_.advertise(from_pc2_depth_image_topic, 1);


    remainedImagesToCalcAvg = num_images_for_background;
    avg_image.create(R,C,CV_32FC1);
    isCalculateAvgSrvCalled = isAvgCalculated = hasCameraInfo = has_avg_image = false;
}

void CvLegTracking::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if (!hasCameraInfo)
    {
        model_.fromCameraInfo(info_msg);
        center_x = model_.cx();
        center_y = model_.cy();

        double unit_scaling = depth_image_proc::DepthTraits<float>::toMeters( float(1) );
        constant_x = unit_scaling / model_.fx();
        constant_y = unit_scaling / model_.fy();
        hasCameraInfo = true;
        ROS_INFO("Get camera info.");
    }
}

void CvLegTracking::processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!msg || !hasCameraInfo) { return; }
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    cv::Mat from_pc2_depth_image = cv::Mat(R, C, CV_32FC1, cv::Scalar(bad_point));
    int m = 0, n = 0;
    for (int i = 0; i < msg->width * msg->height; i++) {
        if (iter_x[2] == iter_x[2]) {
            from_pc2_depth_image.at<float>(m, n) = iter_x[2];
        }
        iter_x += 1;
        n++;
        if (n == C) {
            n = 0;
            m++;
        }
    }
    sensor_msgs::ImagePtr from_pc2_depth_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "32FC1", from_pc2_depth_image).toImageMsg();
    from_pc2_depth_image_ptr->header = msg->header;
    from_pc2_depth_image_pub_.publish(from_pc2_depth_image_ptr);
}


void CvLegTracking::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!has_avg_image) {
        cv::Mat read_image;
        cv::FileStorage fs(avg_image_path, cv::FileStorage::READ);
        fs[cv_fs_image_id] >> read_image;

        if (!read_image.empty()) {
            for(int r = 0; r < read_image.rows; r++) {
                float* avg_image_raw_ptr = avg_image.ptr<float>(r);
                float* read_image_ptr = read_image.ptr<float>(r);

                for(int c = 0; c < read_image.cols; c++) {
                    avg_image_raw_ptr[c] = read_image_ptr[c];
                }
            }

            has_avg_image = true;
            ROS_INFO("Average image is read from '%s'!", avg_image_path.c_str());

        } else {

            if (remainedImagesToCalcAvg == num_images_for_background) {
                ROS_INFO("Call service 'calculateAvg' to calculate average image.");
            }

            if (!isCalculateAvgSrvCalled) { return; }

            if (remainedImagesToCalcAvg == num_images_for_background) {
                ROS_INFO("Average image was not found in '%s', calculating with %d images!", avg_image_path.c_str(), num_images_for_background);
            }

            if (remainedImagesToCalcAvg > 0)
            {
                for(int r = 0; r < avg_image.rows; r++) {
                    float* avg_image_raw_ptr = avg_image.ptr<float>(r);
                    float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);

                    for(int c = 0; c < avg_image.cols; c++) {

                        if (cv_ptr_image_ptr[c] == cv_ptr_image_ptr[c]) { // if not NaN
                            if (avg_image_raw_ptr[c] > 0. || remainedImagesToCalcAvg == num_images_for_background) {
                                avg_image_raw_ptr[c] += (1. / (float) num_images_for_background) * cv_ptr_image_ptr[c];
                            } else {
                                avg_image_raw_ptr[c] = ((num_images_for_background - remainedImagesToCalcAvg + 1) / num_images_for_background) * cv_ptr_image_ptr[c];
                            }
                        } else if (avg_image_raw_ptr[c] > 0.) {
                            avg_image_raw_ptr[c] += (1 / (num_images_for_background - remainedImagesToCalcAvg)) * avg_image_raw_ptr[c];
                        }
                    }
                }
                remainedImagesToCalcAvg--;
                return;
            }

            if (remainedImagesToCalcAvg == 0)
            {
                remainedImagesToCalcAvg--;

                for(int r = 0; r < avg_image.rows; r++) {
                    float* avg_image_raw_ptr = avg_image.ptr<float>(r);
                    float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);

                    for(int c = 0; c < avg_image.cols; c++) {
                        // if NaN
                        if (avg_image_raw_ptr[c] != avg_image_raw_ptr[c] || avg_image_raw_ptr[c] > bad_point) {
                            avg_image_raw_ptr[c] = bad_point;
                        }
                        if (cv_ptr_image_ptr[c] != cv_ptr_image_ptr[c] || cv_ptr_image_ptr[c] > bad_point) {
                            cv_ptr_image_ptr[c] = bad_point;
                        }
                    }
                }
                cv::FileStorage fs(avg_image_path, cv::FileStorage::WRITE);
                fs << cv_fs_image_id << avg_image;


                has_avg_image = true;
                ROS_INFO("Average image is calculated!");
            }
        }
    }

    if (!has_avg_image) { return; }

    cv::Mat diffMat;
    diffMat.create(R,C,CV_32FC1);
    for(int r = 0; r < avg_image.rows; r++) {
        float* avg_image_raw_ptr = avg_image.ptr<float>(r);
        float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
        float* diffMat_ptr = diffMat.ptr<float>(r);

        for(int c = 0; c < avg_image.cols; c++) {
            // if NaN
            if (avg_image_raw_ptr[c] == bad_point || cv_ptr_image_ptr[c] == bad_point) {
                diffMat_ptr[c] = 0.;
            } else {
                diffMat_ptr[c] = std::abs(avg_image_raw_ptr[c] - cv_ptr_image_ptr[c]);
            }
        }
    }

    cv::Mat workingImg;
    workingImg = cv::Mat::zeros(R,C,CV_8UC1);
    for(int r = 0; r < workingImg.rows; r++) {
        uchar* workingImg_ptr = workingImg.ptr<uchar>(r);
        float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
        float* avg_image_raw_ptr = avg_image.ptr<float>(r);
        float* diffMat_ptr = diffMat.ptr<float>(r);

        for(int c = 0; c < workingImg.cols; c++) {

            if (diffMat_ptr[c] > 0.1 && cv_ptr_image_ptr[c] < person_distance && cv_ptr_image_ptr[c] > min_distance_near_camera
                && avg_image_raw_ptr[c] > min_distance_near_camera && avg_image_raw_ptr[c] > cv_ptr_image_ptr[c])
            {
                workingImg_ptr[c] = 255;
            }
        }
    }

    cv::Mat erosion_dst;
    int erosion_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size) );

    erode(workingImg, erosion_dst, element);
    cv::Mat subtract_dst = workingImg - erosion_dst;

    sensor_msgs::ImagePtr msg_to_pub;

    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", avg_image).toImageMsg();
    avg_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", workingImg).toImageMsg();
    working_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", subtract_dst).toImageMsg();
    subtract_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", erosion_dst).toImageMsg();
    erosion_image_pub_.publish(msg_to_pub);
}

bool CvLegTracking::calculateAvgImageCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    isCalculateAvgSrvCalled = true;
    return true;
}