#include "primitive_visualizer.h"

template <typename Point>
void primitive_visualizer<Point>::run_visualizer()
{
    // Wait until visualizer window is closed.
    while (!viewer->wasStopped())
    {
        lock();
        if (cloud_changed) {
            viewer->removePointCloud("cloud");
            pcl::visualization::PointCloudColorHandlerCustom<point_type> single_color(cloud, 0, 255, 0);
            viewer->addPointCloud<point_type>(cloud, single_color, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                     1, "cloud");
            cloud_changed = false;
        }
        if (normals_changed) {
            viewer->removePointCloud("normals");
            viewer->addPointCloudNormals<point_type, pcl::Normal>(cloud, cloud_normals, 100, 1e-2f, "normals");
            normals_changed = false;
        }
        viewer->spinOnce(100);
        unlock();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer->close();
}

template <>
void primitive_visualizer<pcl::PointXYZRGB>::run_visualizer()
{
    // Wait until visualizer window is closed.
    while (!viewer->wasStopped())
    {
        lock();
        if (cloud_changed) {
            viewer->removePointCloud("cloud");
            pcl::visualization::PointCloudColorHandlerRGBField<point_type> rgb(cloud);
            viewer->addPointCloud<point_type>(cloud, rgb, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                     1, "cloud");
            cloud_changed = false;
        }
        if (normals_changed) {
            viewer->removePointCloud("normals");
            viewer->addPointCloudNormals<point_type, pcl::Normal>(cloud, cloud_normals, 100, 1e-2f, "normals");
            normals_changed = false;
        }
        viewer->spinOnce(100);
        unlock();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer->close();
}

template <typename Point>
void primitive_visualizer<Point>::lock()
{
    pthread_mutex_lock(&mutex);
}

template <typename Point>
void primitive_visualizer<Point>::unlock()
{
    pthread_mutex_unlock(&mutex);
}

template <typename Point>
void* viewer_thread(void* ptr)
{
    ((primitive_visualizer<Point>*)ptr)->run_visualizer();
    pthread_exit(NULL);
}

template <typename Point>
void primitive_visualizer<Point>::create_thread()
{
    pthread_create(&my_viewer_thread, NULL, viewer_thread<point_type>, this);
}

template <typename Point>
void primitive_visualizer<Point>::join_thread()
{
    pthread_join(my_viewer_thread, NULL);
}
