/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

// ros msgs
#include <nav_msgs/Odometry.h>
#include <loxo_msgs_ros1/AllSensors.h>

// Pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <inoviz_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>



// ros opencv transport
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// ros tf
#include <tf2_ros/transform_listener.h>

// grid map
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <dynamic_reconfigure/server.h>
#include <groundgrid/GroundGrid.h>
#include <groundgrid/GroundGridConfig.h>
#include <groundgrid/GroundGridFwd.h>
#include <groundgrid/GroundSegmentation.h>

namespace groundgrid {


class GroundGridNodelet : public nodelet::Nodelet {
   public:
    // typedef velodyne_pointcloud::PointXYZIR PCLPoint;
    typedef pcl::PointXYZI PCLPoint;

private:
    bool m_useAllSensors;
    std::string m_inputAllSensorsTopic;
    std::string m_inputCloudTopic;

    float m_inputFeafSize;
    float m_outputRadiusSearch;
    int m_outputMinNeighborsInRadius;

public:

    /** Constructor.
     */
    GroundGridNodelet() : mTfListener(mTfBuffer){}

    /** Destructor.
     */
    virtual ~GroundGridNodelet() {
    }

    /** Nodelet initialization. Called by nodelet manager on initialization,
     */
    virtual void onInit() override {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("groundgrid/obstructed_space", 1);
        groundgrid_ = std::make_shared<GroundGrid>();

        config_server_ = boost::make_shared<dynamic_reconfigure::Server<groundgrid::GroundGridConfig> >(pnh);
        dynamic_reconfigure::Server<groundgrid::GroundGridConfig>::CallbackType f;
        f = boost::bind(&GroundGridNodelet::callbackReconfigure, this, _1, _2);

        ground_segmentation_.init(nh, groundgrid_->mDimension, groundgrid_->mResolution);

        config_server_->setCallback(f);

        // ego-position
        pos_sub_ = nh.subscribe( "/localization/odometry/filtered_map", 1, &groundgrid::GroundGridNodelet::odom_callback, this);


        bool m_useAllSensors;
        nh.param<bool>("use_all_sensors", m_useAllSensors, true);
        nh.param<std::string>("all_sensors_topic", m_inputAllSensorsTopic, "/all_sensors");
        nh.param<std::string>("pointcloud_topic", m_inputCloudTopic, "/cloud");

        if (m_useAllSensors){
            ROS_INFO_STREAM("Using " << m_inputAllSensorsTopic << " as input topic. (Type: loxo_msgs_ros1/AllSensors)");
            all_sensors_points_sub_ = nh.subscribe(m_inputAllSensorsTopic, 1, &groundgrid::GroundGridNodelet::all_sensors_points_callback, this);
        } else {
            ROS_INFO_STREAM("Using " << m_inputCloudTopic << " as input topic. (Type: sensor_msgs/PointCloud2)");
            cloud_points_sub_ = nh.subscribe(m_inputCloudTopic, 1, &groundgrid::GroundGridNodelet::cloud_points_callback, this);
        }

        nh.param<float>("input_leaf_size", m_inputFeafSize, 0.3f);
        nh.param<float>("output_radius_search", m_outputRadiusSearch, 0.5f);
        nh.param<int>("output_min_neighbors_in_radius", m_outputMinNeighborsInRadius, 6);

        ROS_INFO_STREAM("Downscaling input cloud with a leaf of " << m_inputFeafSize);
        ROS_INFO_STREAM("Filtering output cloud with a radius search of " << m_outputRadiusSearch << 
                        " and " << m_outputMinNeighborsInRadius << " min neighbors");
   }

   protected:
    virtual void odom_callback(const nav_msgs::OdometryConstPtr& inOdom){
        static nav_msgs::OdometryConstPtr lastOdom;
        if(!lastOdom || std::hypot(lastOdom->pose.pose.position.x-inOdom->pose.pose.position.x, 2.0f) + std::hypot(lastOdom->pose.pose.position.y-inOdom->pose.pose.position.y, 2.0f) >= 1.0){
            auto start = std::chrono::steady_clock::now();
            map_ptr_ = groundgrid_->update(inOdom);
            auto end = std::chrono::steady_clock::now();
            ROS_DEBUG_STREAM("grid map update took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
        }
    }

    void transformToMapCoordSystem(pcl::PointCloud<PCLPoint>::Ptr cloudIn, pcl::PointCloud<PCLPoint>::Ptr cloudOut, const sensor_msgs::PointCloud2& cloud_msg, const ros::Time& stamp){
        // Transform to map
        geometry_msgs::TransformStamped transformStamped;
        cloudOut->header = cloudIn->header;
        cloudOut->header.frame_id = "map";
        cloudOut->points.reserve(cloudIn->points.size());

        try{
            //mTfBuffer.canTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.0));
            mTfBuffer.canTransform("base_link", "base_link", stamp, ros::Duration(0.0));
            //transformStamped = mTfBuffer.lookupTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.0));
            transformStamped = mTfBuffer.lookupTransform("base_link", "base_link", stamp, ros::Duration(0.0));

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Failed to get map transform for point cloud transformation: %s",ex.what());
            return;
        }

        geometry_msgs::PointStamped psIn;
        psIn.header = cloud_msg.header;
        psIn.header.frame_id = "map";
        psIn.header.stamp = stamp;

        for(const auto& point : cloudIn->points){
            psIn.point.x = point.x;
            psIn.point.y = point.y;
            psIn.point.z = point.z;

            tf2::doTransform(psIn, psIn, transformStamped);

            PCLPoint& point_transformed = cloudOut->points.emplace_back(point);
            point_transformed.x = psIn.point.x;
            point_transformed.y = psIn.point.y;
            point_transformed.z = psIn.point.z;
        }
    }

    void segmentAndPublish(pcl::PointCloud<PCLPoint>::Ptr cloud, const sensor_msgs::PointCloud2& cloud_msg, const ros::Time& stamp){
        static size_t time_vals = 0;
        static double avg_time = 0.0;
        static double avg_cpu_time = 0.0;
        auto start = std::chrono::steady_clock::now();
        std::clock_t c_clock = std::clock();

        geometry_msgs::PointStamped origin;
        origin.header.stamp = stamp;
        origin.header.frame_id = "base_link";
        origin.point.x = 0.0f;
        origin.point.y = 0.0f;
        origin.point.z = 0.0f;

        geometry_msgs::TransformStamped mapToBaseTransform, cloudOriginTransform;

        // Map not initialized yet, this means the node hasn't received any odom message so far.
        if(!map_ptr_){
            return;
        }

        try{
            //mTfBuffer.canTransform("base_link", "map", cloud_msg->header.stamp, ros::Duration(0.0));
            //mapToBaseTransform = mTfBuffer.lookupTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.0));
            mapToBaseTransform = mTfBuffer.lookupTransform("base_link", "base_link", stamp, ros::Duration(0.0));
            //mTfBuffer.canTransform(cloud_msg->header.frame_id, "map", cloud_msg->header.stamp, ros::Duration(0.0));
            cloudOriginTransform = mTfBuffer.lookupTransform("base_link", "base_link", stamp, ros::Duration(0.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Received point cloud but transforms are not available: %s",ex.what());
            return;
        }

        tf2::doTransform(origin, origin, cloudOriginTransform);

        PCLPoint origin_pclPoint;
        origin_pclPoint.x = origin.point.x;
        origin_pclPoint.y = origin.point.y;
        origin_pclPoint.z = origin.point.z;

        sensor_msgs::PointCloud2 cloud_msg_out;
        auto cloud_out = ground_segmentation_.filter_cloud(cloud, origin_pclPoint, mapToBaseTransform, *map_ptr_);


        // Apply a radius outlier filter
        pcl::RadiusOutlierRemoval<PCLPoint> radiusFilter;
        radiusFilter.setInputCloud(cloud_out);
        radiusFilter.setRadiusSearch(m_outputRadiusSearch);
        radiusFilter.setMinNeighborsInRadius(m_outputMinNeighborsInRadius);
        radiusFilter.filter(*cloud_out);


        // Filter out points with z values above a threshold
        pcl::PointCloud<PCLPoint>::Ptr filtered_cloud(new pcl::PointCloud<PCLPoint>);
        filtered_cloud->header = cloud_out->header;
        filtered_cloud->reserve(cloud_out->points.size());

        const float z_threshold = 2.5f; // A bit more than the height of the car
        for (const auto& point : cloud_out->points) {
            if (point.z <= z_threshold) {
                filtered_cloud->points.push_back(point);
            }
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = cloud_out->is_dense;

        cloud_out = filtered_cloud;

        for (auto& point : cloud_out->points) {
            point.z = 0;
        }

        pcl::toROSMsg(*cloud_out, cloud_msg_out);

        cloud_msg_out.header = cloud_msg.header;
        cloud_msg_out.header.stamp = stamp;
        cloud_msg_out.header.frame_id = "base_link";
        filtered_cloud_pub_.publish(cloud_msg_out);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        const double milliseconds = elapsed_seconds.count() * 1000;
        const double c_millis = double(std::clock() - c_clock) / CLOCKS_PER_SEC * 1000;
        avg_time = (milliseconds + time_vals * avg_time) / (time_vals+1);
        avg_cpu_time = (c_millis + time_vals * avg_cpu_time) / (time_vals+1);
        time_vals++;
        ROS_INFO_STREAM_THROTTLE(10, "groundgrid took " << milliseconds << "ms (avg: " << avg_time << "ms)");
        ROS_DEBUG_STREAM("total cpu time used: " << c_millis << "ms (avg: " << avg_cpu_time << "ms)") ;


        end = std::chrono::steady_clock::now();
        ROS_DEBUG_STREAM("overall " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
    }

   private:

      // Duplicate but w/e
      void all_sensors_points_callback(const loxo_msgs_ros1::AllSensors::ConstPtr& all_sensors_msg){
        const ros::Time& stamp = all_sensors_msg->stamp;

        // Create fake odometry message to trigger map creation by calling the odom_callback.
        // TODO: This should be removed later and replaced with correct odometry
        {
            nav_msgs::Odometry inOdom;
            inOdom.pose.pose.position.x = 0.0;
            inOdom.pose.pose.position.y = 0.0;
            inOdom.pose.pose.position.z = 0.0;
            inOdom.pose.pose.orientation.x = 0,0;
            inOdom.pose.pose.orientation.y = 0,0;
            inOdom.pose.pose.orientation.z = 0,0;
            inOdom.pose.pose.orientation.w = 1.0;
            inOdom.header.frame_id = "base_link";
            auto odom = boost::make_shared<nav_msgs::Odometry>(inOdom);
            odom_callback(odom);
        }

        pcl::PointCloud<PCLPoint>::Ptr cloud_raw(new pcl::PointCloud<PCLPoint>);
        pcl::PointCloud<PCLPoint>::Ptr cloud(new pcl::PointCloud<PCLPoint>);
        pcl::fromROSMsg (all_sensors_msg->combined_cloud, *cloud_raw);

        ROS_DEBUG_STREAM("Point cloud size before filtering: " << cloud_raw->size());

        // Filter to reduce number of points
        pcl::VoxelGrid<PCLPoint> sor;
        sor.setInputCloud (cloud_raw);
        const float leaf_size = m_inputFeafSize;
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);
        sor.filter(*cloud);

        ROS_DEBUG_STREAM("Point cloud size after filtering: " << cloud->size());

        // Transform to map
        geometry_msgs::TransformStamped transformStamped;
        pcl::PointCloud<PCLPoint>::Ptr transformed_cloud(new pcl::PointCloud<PCLPoint>);
        transformToMapCoordSystem(cloud, transformed_cloud, all_sensors_msg->combined_cloud, stamp);
        cloud = transformed_cloud;

        segmentAndPublish(cloud, all_sensors_msg->combined_cloud, stamp);
   }

   void cloud_points_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        const ros::Time& stamp = cloud_msg->header.stamp;

        // Create fake odometry message to trigger map creation by calling the odom_callback.
        // TODO: This should be removed later and replaced with correct odometry
        {
            nav_msgs::Odometry inOdom;
            inOdom.pose.pose.position.x = 0.0;
            inOdom.pose.pose.position.y = 0.0;
            inOdom.pose.pose.position.z = 0.0;
            inOdom.pose.pose.orientation.x = 0,0;
            inOdom.pose.pose.orientation.y = 0,0;
            inOdom.pose.pose.orientation.z = 0,0;
            inOdom.pose.pose.orientation.w = 1.0;
            inOdom.header.frame_id = "base_link";
            auto odom = boost::make_shared<nav_msgs::Odometry>(inOdom);
            odom_callback(odom);
        }

        pcl::PointCloud<PCLPoint>::Ptr cloud_raw(new pcl::PointCloud<PCLPoint>);
        pcl::PointCloud<PCLPoint>::Ptr cloud(new pcl::PointCloud<PCLPoint>);
        pcl::fromROSMsg (*cloud_msg, *cloud_raw);

        ROS_DEBUG_STREAM("Point cloud size before filtering: " << cloud_raw->size());

        // Filter to reduce number of points
        pcl::VoxelGrid<PCLPoint> sor;
        sor.setInputCloud (cloud_raw);
        const float leaf_size = m_inputFeafSize;
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);
        sor.filter(*cloud);

        ROS_DEBUG_STREAM("Point cloud size after filtering: " << cloud->size());

        // Transform cloud into map coordinate system
        if(cloud_msg->header.frame_id != "map"){

            // Transform to map
            geometry_msgs::TransformStamped transformStamped;
            pcl::PointCloud<PCLPoint>::Ptr transformed_cloud(new pcl::PointCloud<PCLPoint>);
            transformToMapCoordSystem(cloud, transformed_cloud, *cloud_msg, stamp);
            cloud = transformed_cloud;
        }

        segmentAndPublish(cloud, *cloud_msg, stamp);
   }


    /** Callback for dynamic_reconfigure.
     **
     ** @param msg
     */
    void callbackReconfigure(groundgrid::GroundGridConfig & config, uint32_t level) {
        groundgrid_->setConfig(config);
        ground_segmentation_.setConfig(config);
    }

    /// subscriber
    ros::Subscriber all_sensors_points_sub_, cloud_points_sub_, pos_sub_;

    /// publisher
    ros::Publisher filtered_cloud_pub_;

    /// pointer to dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<groundgrid::GroundGridConfig> > config_server_;

    /// pointer to the functionality class
    GroundGridPtr groundgrid_;

    /// grid map
    std::shared_ptr<grid_map::GridMap> map_ptr_;

    /// Filter class for grid map
    GroundSegmentation ground_segmentation_;

    double m_resolution = 0.1;
    double m_length = 50;

    /// tf stuff
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(groundgrid::GroundGridNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
