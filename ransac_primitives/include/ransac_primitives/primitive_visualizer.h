#ifndef PRIMITIVE_VISUALIZER_H
#define PRIMITIVE_VISUALIZER_H
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

template <typename Point>
class primitive_visualizer
{
public:
    typedef Point point_type;
    typedef pcl::PointCloud<point_type> cloud_type;
    typedef typename cloud_type::ConstPtr cloud_const_ptr_type;
private:
    pthread_mutex_t mutex;
    pthread_t my_viewer_thread;
public:
    bool cloud_changed;
    bool normals_changed;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    cloud_const_ptr_type cloud;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals;
    void create_thread();
    void join_thread();
    void lock();
    void unlock();
    void run_visualizer();
    primitive_visualizer() : viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
    {
        viewer->setBackgroundColor(0, 0, 0);
        // Starting visualizer
        //viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        cloud_changed = false;
        normals_changed = false;
        if (pthread_mutex_init(&mutex, NULL) != 0) {
            std::cout << "mutex init failed" << std::endl;
        }
    }
};

#include "primitive_visualizer.hpp"
#endif // PRIMITIVE_VISUALIZER_H
