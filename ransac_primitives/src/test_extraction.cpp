#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "ransac_primitives/primitive_core.h"
#include "ransac_primitives/plane_primitive.h"
#include "ransac_primitives/sphere_primitive.h"
#include "ransac_primitives/cylinder_primitive.h"


#include <pcl/features/normal_3d.h>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Please supply path to pointcloud to process..." << std::endl;
        return 0;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::string filename(argv[1]);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1)
    {
        std::cout << "Couldn't read file " << filename << std::endl;
        return 0;
    }

    std::vector<base_primitive*> primitives = { new plane_primitive() };
    // sphere_primitive and cylinder_primitive have not been ported to the new framework yet
    primitive_params params;
    params.number_disjoint_subsets = 20;
    params.octree_res = 0.5;
    params.normal_neigbourhood = 0.04;
    params.inlier_threshold = 0.04;
    params.angle_threshold = 0.4;
    params.add_threshold = 0.01;
    params.min_shape = 2000;
    params.inlier_min = params.min_shape;
    params.connectedness_res = 0.01;
    params.distance_threshold = 4.0;

    primitive_visualizer<pcl::PointXYZRGB> viewer;
    primitive_extractor<pcl::PointXYZRGB> extractor(cloud, primitives, params, &viewer);
    viewer.cloud = extractor.get_cloud();
    viewer.cloud_changed = true;
    viewer.cloud_normals = extractor.get_normals();
    viewer.normals_changed = true;
    viewer.create_thread();
    std::vector<base_primitive*> extracted;
    extractor.extract(extracted);

    std::cout << "The algorithm has finished..." << std::endl;

    viewer.join_thread();

    return 0;
}
