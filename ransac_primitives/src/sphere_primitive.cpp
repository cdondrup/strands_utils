#include "ransac_primitives/sphere_primitive.h"

#include <iostream> // DEBUG
#include <Eigen/LU>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp> // DEBUG

using namespace Eigen;

int sphere_primitive::spheres_drawn = 0;

sphere_primitive::sphere_primitive()
{
    max_radius = 0.1;
}

int sphere_primitive::points_required()
{
    return 2;
}

bool sphere_primitive::construct(const MatrixXd& points, const MatrixXd& normals,
                                double inlier_threshold, double angle_threshold)
{
    Vector3d p1 = points.col(0);
    Vector3d p2 = points.col(1);
    Vector3d n1 = normals.col(0);
    Vector3d n2 = normals.col(1);

    /*Matrix2d A;
    A(0, 0) = n1.dot(n1);
    A(0, 1) = n1.dot(n2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = n2.dot(n2);
    Vector2d b;
    b(0) = n1.dot(p1 - p2);
    b(1) = n2.dot(p1 - p2);

    Vector2d x;
    //x = A.inverse()*b;
    x = A.lu().solve(b);*/

    // Try 2
    /*Vector3d R = (p2 - p1).cross(n1.cross(n2));
    Vector2d x;
    x(0) = R.dot(n2);
    x(1) = R.dot(n1);

    c = 0.5*(p1 + x(0)*n1 + p2 + x(1)*n2);
    r = 0.5*((p1 - c).norm() + (p2 - c).norm());*/

    // Try 3
    Vector3d n = p1 - p2;
    n.normalize();
    double d = -n.dot(0.5*(p1 + p2));
    double t1 = -(d + n.dot(p1))/n.dot(n1);
    double t2 = -(d + n.dot(p2))/n.dot(n2);

    c = 0.5*(p1 + t1*n1 + p2 + t2*n2);
    r = 0.5*((p1 - c).norm() + (p2 - c).norm());

    /*std::cout << "Radius: " << r << std::endl;
    std::cout << "C1: " << (p1 + t1*n1).transpose() << std::endl;
    std::cout << "C2: " << (p2 + t2*n2).transpose() << std::endl;*/

    // Try 4, without normals
    /*Matrix3d N;
    N.row(0) = p1 - p2;
    N.row(0).normalize();
    N.row(1) = p3 - p1;
    N.row(1).normalize();
    N.row(2) = p2 - p3;
    N.row(2).normalize();
    Vector3d d;
    d(0) = -N.row(0)*(0.5*(p1 + p2));
    d(1) = -N.row(1)*(0.5*(p3 + p1));
    d(2) = -N.row(2)*(0.5*(p2 + p3));
    c = -N.lu().solve(d);
    r = 1.0/3.0*((p1 - c).norm() + (p2 - c).norm() + (p3 - c).norm());*/

    if (isnan(r) || fabs(r) < 0.02 || r > max_radius) {
        return false;
    }

    /*if (isinf(x.sum()) || isnan(x.sum())) {
        return false;
    }*/

    Vector3d rad;
    double radnorm = rad.norm();
    for (int i = 0; i < 2; ++i) {
        rad = points.col(i) - c;
        if (fabs(rad.norm() - r) > inlier_threshold || acos(fabs(normals.col(i).dot(rad/radnorm))) > angle_threshold) {
            //std::cout << "Proximity: " << fabs((rad).norm() - r) << std::endl;
            //std::cout << "Angle: " << acos(fabs(normals.col(i).dot(rad))) << std::endl;
            /*if (isnan(acos(fabs(normals.col(i).dot(rad))))) {
                std::cout << "Angle NAN!" << std::endl;
                //exit(0);
            }*/
            return false;
        }
    }

    //std::cout << "Succeded" << std::endl;
    //std::cout << "c: " << c.transpose() << std::endl;
    //std::cout << "r: " << r << std::endl;

    return true;
}

void sphere_primitive::compute_inliers(std::vector<int>& inliers, const MatrixXd& points, const MatrixXd& normals,
                                       const std::vector<int>& inds, double inlier_threshold, double angle_threshold)
{
    // check for inliers to primitive
    Vector3d pt;
    Vector3d n;
    Vector3d rad;
    double rad_norm;
    double cos_threshold = cos(angle_threshold);
    for (const int& i : inds) {
        pt = points.col(i);
        n = normals.col(i);
        rad = pt - c;
        rad_norm = rad.norm();
        rad *= 1.0/rad_norm;
        if (fabs(n.dot(rad)) > cos_threshold &&
                fabs(rad_norm - r) < inlier_threshold) {
            inliers.push_back(i);
        }
    }
}

void sphere_primitive::largest_connected_component(std::vector<int>& inliers, const MatrixXd& points)
{
    Vector2i minpt;
    minpt << int(-r*M_PI/2.0/current_connectedness_res()), int(-r*M_PI/2.0/current_connectedness_res());
    Vector2i maxpt;
    maxpt << int(r*M_PI/2.0/current_connectedness_res()), int(r*M_PI/2.0/current_connectedness_res());

    int width = 1 + maxpt(0) - minpt(0);
    int height = 1 + maxpt(1) - minpt(1);

    cv::Mat binary = cv::Mat::zeros(2*height, width, CV_32SC1);
    //cv::Mat binary0 = cv::Mat::zeros(2*height, width, CV_32SC1);

    // check for inliers to primitive
    Vector2d pt;
    Vector2i pt2i;
    Vector3d rad;
    double rad_norm;
    std::vector<Vector2i, aligned_allocator<Vector2i> > pts;
    pts.resize(conforming_inds.size());
    int counter = 0;
    for (const int& i : conforming_inds) {
        rad = points.col(i) - c;
        rad_norm = rad.norm();
        rad *= 1.0/rad_norm;
        bool isupper = sphere_to_grid(pt, rad);
        pt2i = (1.0/current_connectedness_res()*pt).cast<int>() - minpt;
        if (isupper) {
            pt2i(1) += height;
        }
        pts[counter] = pt2i;
        binary.at<int>(pt2i(1), pt2i(0)) = 1;
        //binary0.at<int>(pt2i(1), pt2i(0)) = 65535;
        ++counter;
    }

    //cv::imshow("Binary0", binary0);
    //cv::waitKey(0);

    //cv::Mat binary2 = cv::Mat::zeros(2*height, width, CV_32SC1);

    inliers.reserve(pts.size());
    int largest = find_blobs(binary, true, true);
    counter = 0;
    for (const Vector2i& pp : pts) {
        if (binary.at<int>(pp(1), pp(0)) == largest) {
            inliers.push_back(conforming_inds[counter]);
            //binary2.at<int>(pp(1), pp(0)) = 65535;
        }
        ++counter;
    }

    //cv::imshow("Binary2", binary2);
    //cv::waitKey(0);
}

base_primitive::shape sphere_primitive::get_shape()
{
    return SPHERE;
}

base_primitive* sphere_primitive::instantiate()
{
    return new sphere_primitive();
}

void sphere_primitive::draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    if (false) {
        std::cout << "Radius: " << r << std::endl;
        exit(0);
    }
    pcl::PointXYZ p(c(0), c(1), c(2));
    viewer->addSphere<pcl::PointXYZ>(p, r, 0.0, 1.0, 0.0, std::string("sphere") + boost::lexical_cast<std::string>(spheres_drawn));
    ++spheres_drawn;
}

bool sphere_primitive::sphere_to_grid(Vector2d& gridpt, const Vector3d& spherept)
{
    Vector3d pt = spherept;
    bool isupper;
    if (pt(2) > 0) {
        isupper = true;
    }
    else {
        isupper = false;
        pt(1) *= -1.0; // fold it around
        pt(2) *= -1.0;
    }
    Vector2d circlept;
    double angle = acos(fabs(spherept(2))); // max 1
    circlept = angle/(M_PI/2.0)*spherept.segment<2>(0);
    circle_to_grid(gridpt, circlept);
    gridpt.array() -= 0.5; // change to [-1, 1] instead of [0, 1]
    gridpt *= 2.0;
    gridpt *= r*M_PI/2.0;
    return isupper;
}

double sphere_primitive::distance_to_pt(const Vector3d& pt)
{
    return fabs((pt - c).norm() - r);
}

void sphere_primitive::direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center)
{
    direction.setZero();
    center = c;
}

double sphere_primitive::shape_size()
{
    return r;
}

void sphere_primitive::shape_data(VectorXd& data)
{
    data.resize(4);
    data.segment<3>(0) = c;
    data(3) = r;
}

void sphere_primitive::shape_points(std::vector<Vector3d, aligned_allocator<Vector3d> >& points)
{
    points.clear();
}
