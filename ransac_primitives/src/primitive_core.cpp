#include "ransac_primitives/primitive_core.h"
#include <pcl/point_types.h>

template class primitive_extractor<pcl::PointXYZRGB>;
template class primitive_extractor<pcl::PointXYZ>;

template class primitive_visualizer<pcl::PointXYZRGB>;
template class primitive_visualizer<pcl::PointXYZ>;
