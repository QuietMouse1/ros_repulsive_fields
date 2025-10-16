#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <repulsive_fields/setRepulsiveFieldsConfig.h>
#include <visualization_msgs/Marker.h>

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

// Repulsive Field Parameters
float k_repulsive = 1.0;
float MAX_force = 1.0;
float AGENT_radius = 0.1;
float MAX_radius = 1.0;
float MAX_height = 2.5;
float MIN_height = 0.15;

class RepulsiveFields{
    public:
        RepulsiveFields(ros::NodeHandle& node_handle);
        ~RepulsiveFields();
        void run();

    private:
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg); 
        void filterCloudRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_range, float max_range, float max_height, float min_height);

        void mavrosSetpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void mavrosRawSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr &msg);
        ros::Subscriber pointcloud2_subscriber_;
        ros::Subscriber odometry_subscriber_;

        ros::Subscriber mavros_setpoint_pose_local_sub_;
        ros::Subscriber mavros_setpoint_raw_sub_;

        ros::Publisher repulsive_vec3_;
        ros::Publisher repulsive_pcl_pub_;
        ros::Publisher repulsive_marker_pub_;

        ros::Publisher mavros_setpoint_pose_local_pub_;
        ros::Publisher mavros_setpoint_raw_pub_;

        nav_msgs::Odometry odometry_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_;

        geometry_msgs::PoseStamped input_mavros_setpoint_;
        mavros_msgs::PositionTarget input_mavros_raw_setpoint_;
        Vector3f sum_repulsive_force_;
};
