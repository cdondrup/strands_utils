#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <ransac_primitives/sphere_primitive.h>
#include <ransac_primitives/cylinder_primitive.h>

#include "primitive_extraction/Primitive.h"
#include "primitive_extraction/PrimitiveArray.h"
#include "primitive_extraction/ExtractPrimitives.h"

#include <Eigen/Dense>

ros::Publisher pub;
double subsampling_voxel_size;
primitive_params params;
std::vector<base_primitive*> primitives;
double min_height;
double max_height;

void write_plane_msg(primitive_extraction::Primitive& msg, const Eigen::VectorXd& data, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points)
{
    msg.type = "plane";
    msg.pose.position.x = data(6);
    msg.pose.position.y = data(7);
    msg.pose.position.z = data(8);
    msg.pose.orientation.x = data(9);
    msg.pose.orientation.y = data(10);
    msg.pose.orientation.z = data(11);
    msg.pose.orientation.w = data(12);
    msg.params.resize(6);
    msg.params[0] = data(4);
    msg.params[1] = data(5);
    msg.params[2] = data(0);
    msg.params[3] = data(1);
    msg.params[4] = data(2);
    msg.params[5] = data(3);
    msg.points.resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        msg.points[i].x = points[i](0);
        msg.points[i].y = points[i](1);
        msg.points[i].z = points[i](2);
    }
}

void write_cylinder_msg(primitive_extraction::Primitive& msg, const Eigen::VectorXd& data)
{
    msg.type = "cylinder";
    msg.pose.position.x = data(3);
    msg.pose.position.y = data(4);
    msg.pose.position.z = data(5);
    Eigen::Vector3d x = data.segment<3>(0);
    Eigen::Vector3d y(-x(1), x(0), 0);
    y.normalize();
    Eigen::Vector3d z = x.cross(y);
    z.normalize();
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    Eigen::Quaterniond quat(R);
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.params.resize(2);
    msg.params[0] = data(6); // radius
    msg.params[1] = data(7); // height
}

void write_sphere_msg(primitive_extraction::Primitive& msg, const Eigen::VectorXd& data)
{
    msg.type = "sphere";
    msg.pose.position.x = data(0);
    msg.pose.position.y = data(1);
    msg.pose.position.z = data(2);
    Eigen::Quaterniond quat;
    quat.setIdentity();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.params.resize(1);
    msg.params[0] = data(3); // radius
}

void extract(primitive_extraction::PrimitiveArray& msg_array, std::vector<base_primitive*>& extracted, 
             const sensor_msgs::PointCloud2& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(msg, *msg_cloud);
    
    ROS_INFO("Got a point cloud of size %lu", msg_cloud->size());
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(msg_cloud);
    sor.setLeafSize(subsampling_voxel_size, subsampling_voxel_size, subsampling_voxel_size);
    if (min_height != 0.0 || max_height != 0.0) {
        sor.setFilterFieldName("z");
        sor.setFilterLimits(min_height, max_height);
    }
    sor.filter(*cloud);
    ROS_INFO("Downsampled to %lu", cloud->size());

    ros::Time begin = ros::Time::now();
    primitive_extractor<pcl::PointXYZ> extractor(cloud, primitives, params, NULL);
    ROS_INFO("Primitive extraction started...");
    extractor.extract(extracted);
    ros::Time end = ros::Time::now();
    ros::Duration duration = end - begin;
    ROS_INFO("Algorithm finished after %f seconds...", duration.toSec());
    
    msg_array.primitives.resize(extracted.size());
    msg_array.camera_frame = msg.header.frame_id;
    
    for (size_t i = 0; i < extracted.size(); ++i) {
        primitive_extraction::Primitive primitive_msg;
        Eigen::VectorXd data;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
        extracted[i]->shape_data(data);
        switch (extracted[i]->get_shape()) {
        case base_primitive::PLANE: 
            extracted[i]->shape_points(points);
            write_plane_msg(primitive_msg, data, points);
            break;
        case base_primitive::CYLINDER:
            write_cylinder_msg(primitive_msg, data);
            break;
        case base_primitive::SPHERE:
            write_sphere_msg(primitive_msg, data);
            break;
        }
        msg_array.primitives[i] = primitive_msg;
    }
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    primitive_extraction::PrimitiveArray msg_array;
    std::vector<base_primitive*> extracted;
    extract(msg_array, extracted, *msg);
    for (size_t i = 0; i < extracted.size(); ++i) {
        delete extracted[i];
    }
    pub.publish(msg_array);
}

bool service_callback(primitive_extraction::ExtractPrimitives::Request& req,
                      primitive_extraction::ExtractPrimitives::Response& res)
{
    std::vector<base_primitive*> extracted;
    extract(res.primitives, extracted, req.pointcloud);
    res.indices.resize(extracted.size());
    for (size_t i = 0; i < extracted.size(); ++i) {
        res.indices[i].indices.resize(extracted[i]->supporting_inds.size());
        for (size_t j = 0; j < extracted[i]->supporting_inds.size(); ++j) {
            res.indices[i].indices[j] = extracted[i]->supporting_inds[j];
        }
        delete extracted[i];
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extraction");
	ros::NodeHandle n;
	
    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle pn("~");
    pn.param<int>("number_disjoint_subsets", params.number_disjoint_subsets, 20);
    pn.param<double>("subsampling_voxel_size", subsampling_voxel_size, 0.02);
    pn.param<double>("octree_leaf_size", params.octree_res, 0.5);
    pn.param<double>("normal_neigbourhood", params.normal_neigbourhood, 0.04);
    pn.param<double>("inlier_threshold", params.inlier_threshold, 0.04);
    pn.param<double>("angle_threshold", params.angle_threshold, 0.4);
    pn.param<double>("error_add_probability", params.add_threshold, 0.01);
    pn.param<int>("min_inliers", params.inlier_min, 2000);
    pn.param<int>("min_terminate", params.min_shape, 2000);
    pn.param<double>("connectedness_dist", params.connectedness_res, 0.03);
    pn.param<double>("distance_threshold", params.distance_threshold, 0.0);
    bool extract_planes, extract_cylinders, extract_spheres;
    pn.param<bool>("extract_planes", extract_planes, true);
    pn.param<bool>("extract_cylinders", extract_cylinders, true);
    pn.param<bool>("extract_spheres", extract_spheres, true);
    pn.param<double>("min_height", min_height, 0.0);
    pn.param<double>("max_height", max_height, 0.0);
    std::string input;
    pn.param<std::string>("input", input, std::string("/primitive_extraction/input"));
    std::string output;
    pn.param<std::string>("output", output, std::string("/primitive_extraction/primitives"));
    
    if (extract_planes) {
        primitives.push_back(new plane_primitive());
    }
    if (extract_cylinders) {
        primitives.push_back(new cylinder_primitive());
    }
    if (extract_spheres) {
        primitives.push_back(new sphere_primitive());
    }
	
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	pub = n.advertise<primitive_extraction::PrimitiveArray>(output, 1);
	ros::ServiceServer service = n.advertiseService("extract_primitives", &service_callback);
    
    ros::spin();
    
    return 0;
}

