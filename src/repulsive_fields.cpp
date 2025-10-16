#include <repulsive_fields/repulsive_fields.h>

double distance(Vector3f v){
    return sqrt(pow(v(0), 2) + pow(v(1), 2));
}

double distance(pcl::PointXYZ p){
    return sqrt(pow(p.x, 2) + pow(p.y, 2) );
}

void dynamicReconfigureCallback(repulsive_fields::setRepulsiveFieldsConfig &config, uint32_t level){
    k_repulsive = config.k_repulsive;
    MAX_force = config.max_force;
    MAX_height = config.max_height;
    MIN_height = config.min_height;
    MAX_radius = config.max_radius;
}

RepulsiveFields::RepulsiveFields(ros::NodeHandle &node_handle){
    // Parameters
    node_handle.getParam("AGENT_radius", AGENT_radius);
    node_handle.getParam("MAX_radius", MAX_radius);
    node_handle.getParam("MAX_height", MAX_height);
    node_handle.getParam("MIN_height", MIN_height);
    node_handle.getParam("k_repulsive", k_repulsive);
    ROS_INFO_STREAM("[RepulsiveFields] AGENT_radius = " << MAX_height);
    ROS_INFO_STREAM("[RepulsiveFields] MAX_radius = " << MAX_height);
    ROS_INFO_STREAM("[RepulsiveFields] MAX_height = " << MAX_height);
    ROS_INFO_STREAM("[RepulsiveFields] MIN_height = " << MIN_height);
    ROS_INFO_STREAM("[RepulsiveFields] k_repulsive = " << k_repulsive);

    // Subscribers
    pointcloud2_subscriber_ = node_handle.subscribe("/simulated_pointcloud", 1, &RepulsiveFields::pointcloud2Callback, this);
    odometry_subscriber_ = node_handle.subscribe("/odometry", 1, &RepulsiveFields::odometryCallback, this);

    mavros_setpoint_pose_local_sub_ = node_handle.subscribe<geometry_msgs::PoseStamped>("/mavros/setpoint_position/raw_local", 10, &RepulsiveFields::mavrosSetpointCallback, this);
    mavros_setpoint_raw_sub_ = node_handle.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/raw_local", 10, &RepulsiveFields::mavrosRawSetpointCallback, this);

    // Publishers
    repulsive_vec3_ = node_handle.advertise<geometry_msgs::Vector3>("/repulsive_force", 1); // for debug
    repulsive_pcl_pub_ = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/repulsive_point_clouds", 1); // for debug
    repulsive_marker_pub_ = node_handle.advertise<visualization_msgs::Marker>("/repulsive_marker", 1);
    mavros_setpoint_raw_pub_ = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    pointcloud2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

RepulsiveFields::~RepulsiveFields(){
    ros::shutdown();
    exit(0);
}

void RepulsiveFields::run(){
    dynamic_reconfigure::Server<repulsive_fields::setRepulsiveFieldsConfig> server;
    dynamic_reconfigure::Server<repulsive_fields::setRepulsiveFieldsConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    ros::spin();
    // ros::Rate rate(100);
    // while(ros::ok())
    // {
    //     rate.sleep();
    //     ros::spinOnce();
    // }
}

void RepulsiveFields::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

// Filter pointcloud such that only points between AGENT_radius and MAX_range, min and max height and are kept.
void RepulsiveFields::filterCloudRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_range, float max_range, float max_height, float min_height)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    //ROS_INFO_STREAM("[RepulsiveFields] cloud->size() = " << cloud->size());

    for(int i = 0; i < cloud->size(); ++i)
    {
        float cloud_x = cloud->at(i).x;
        float cloud_y = cloud->at(i).y;
        float cloud_z = cloud->at(i).z;
        float odometry_x = odometry_.pose.pose.position.x;
        float odometry_y = odometry_.pose.pose.position.y;
        float odometry_z = odometry_.pose.pose.position.z;

        float xy_dist = sqrt(pow(cloud_x - odometry_x, 2) + pow(cloud_y - odometry_y, 2)); 
        if(xy_dist < min_range || xy_dist > max_range || cloud->at(i).z > max_height || cloud->at(i).z < min_height)
        {
            inliers->indices.push_back(i);
        }
    }

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}

void RepulsiveFields::mavrosSetpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    input_mavros_setpoint_ = *msg;
}

void RepulsiveFields::mavrosRawSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    input_mavros_raw_setpoint_ = *msg;
    input_mavros_raw_setpoint_.velocity.x += sum_repulsive_force_(0);
    input_mavros_raw_setpoint_.velocity.y += sum_repulsive_force_(1);

    mavros_setpoint_raw_pub_.publish(input_mavros_raw_setpoint_);
    // input_mavros_raw_setpoint_.velocity.z += sum_repulsive_force_.z;
    // msg.acceleration_or_force = 
}

void RepulsiveFields::pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *pointcloud2_);
    filterCloudRange(pointcloud2_, AGENT_radius, MAX_radius, MAX_height, MIN_height); 

    sum_repulsive_force_ = Vector3f(0,0,0);
    int points = 0;
    float odometry_x = odometry_.pose.pose.position.x;
    float odometry_y = odometry_.pose.pose.position.y;
    for(int i = 0; i < pointcloud2_->size(); ++i)
    {
        float cloud_x = pointcloud2_->at(i).x;
        float cloud_y = pointcloud2_->at(i).y;

        Vector3f obstacle_coords(pointcloud2_->points[i].x , pointcloud2_->points[i].y ,0); 
        Vector3f agent_coords(odometry_x , odometry_y ,0); 

        float obstacle_dist = sqrt(pow(cloud_x - odometry_x, 2) + pow(cloud_y - odometry_y, 2)); 

        Vector3f repulsive_force = (1/obstacle_dist) * ( (agent_coords - obstacle_coords) / obstacle_dist);
        sum_repulsive_force_ += repulsive_force;
        ++points;
    }

    sensor_msgs::PointCloud2 pcl2_msg;
    pcl::toROSMsg(*pointcloud2_, pcl2_msg);
    pcl2_msg.header.frame_id = "map";
    pcl2_msg.header.stamp = ros::Time::now();
    repulsive_pcl_pub_.publish(pcl2_msg); // Publish the filtered point cloud for debug

    if(points){ // normalise repulsive force
        sum_repulsive_force_ /= points;
    }

    sum_repulsive_force_ = k_repulsive * sum_repulsive_force_;

    if (distance(sum_repulsive_force_) > MAX_force){
        sum_repulsive_force_ = MAX_force * (sum_repulsive_force_ / distance(sum_repulsive_force_)); // normalize to MAX_force
    }
    if (distance(sum_repulsive_force_) < 0.1){
        sum_repulsive_force_ = Vector3f(0, 0, 0); 
    }

    ROS_INFO_STREAM("[RepulsiveFields] sum_repulsive_force_ = " << sum_repulsive_force_);

    geometry_msgs::Vector3 debug_msg;
    debug_msg.x = sum_repulsive_force_(0);
    debug_msg.y = sum_repulsive_force_(1);
    debug_msg.z = 0;
    repulsive_vec3_.publish(debug_msg);

    // Marker for visualization
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "arrow_marker";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // Arrow defined by 2 points: start and end
    geometry_msgs::Point start, end;
    start.x = odometry_x;
    start.y = odometry_y;
    start.z = 0.0;

    end.x = odometry_x + sum_repulsive_force_(0);
    end.y = odometry_y + sum_repulsive_force_(1);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    // Arrow dimensions
    arrow.scale.x = 0.05;  // shaft thickness
    arrow.scale.y = 0.1;   // head thickness
    arrow.scale.z = 0.2;   // head length

    // Color (red arrow)
    arrow.color.r = 1.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0f;
    arrow.lifetime = ros::Duration(1);
    repulsive_marker_pub_.publish(arrow);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "RepulsiveFields");
    ros::NodeHandle node_handle;
    RepulsiveFields* repulsive_fields = new RepulsiveFields(node_handle);
    cout << "[RepulsiveFields] RepulsiveFields node ..." << endl;
    repulsive_fields->run();
}
