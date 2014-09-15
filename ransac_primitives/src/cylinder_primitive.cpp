#include "ransac_primitives/cylinder_primitive.h"

#include <iostream> // DEBUG
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> // DEBUG
#include <boost/lexical_cast.hpp>

using namespace Eigen;

int cylinder_primitive::cylinders_drawn = 0;

cylinder_primitive::cylinder_primitive()
{
    max_radius = 0.25;
}

int cylinder_primitive::points_required()
{
    return 3;
}

bool cylinder_primitive::construct(const MatrixXd& points, const MatrixXd& normals,
                                   double inlier_threshold, double angle_threshold)
{
    Vector3d p1 = points.col(0);
    Vector3d p2 = points.col(1);
    Vector3d n1 = normals.col(0);
    Vector3d n2 = normals.col(1);
    a = n1.cross(n2);
    a.normalize();

    basis.col(0) = a.cross(n1);
    basis.col(0).normalize();
    basis.col(1) = a.cross(basis.col(0));

    Matrix2d A;
    A.col(0) = -basis.transpose()*n1;
    A.col(1) = basis.transpose()*n2;

    Vector2d b = basis.transpose()*(p1 - p2);

    Vector2d t = A.inverse()*b;

    Vector3d c1 = p1 + t(0)*n1;
    Vector3d c2 = p2 + t(1)*n2;

    c = (c1 + c2)/2.0;

    r = (fabs(t(0)) + fabs(t(1)))/2.0;

    if (r > max_radius || r < 0.07) {
        return false;
    }

    //std::cout << "c1: " << c1.transpose() << std::endl;
    //std::cout << "c2: " << c2.transpose() << std::endl;
    //std::cout << "r: " << r << std::endl;

    Vector2d pt2;
    double sin_threshold = sin(angle_threshold);
    for (int i = 0; i < 3; ++i) {
        pt2 = basis.transpose()*(points.col(i) - c);
        if (fabs(pt2.norm() - r) > inlier_threshold || fabs(a.dot(normals.col(i))) > sin_threshold) {
            //std::cout << "Distance: " << fabs(pt2.norm() - r) << std::endl;
            return false;
        }
    }

    //exit(0);

    return true;
}

void cylinder_primitive::compute_inliers(std::vector<int>& inliers, const MatrixXd& points, const MatrixXd& normals,
                                         const std::vector<int>& inds, double inlier_threshold, double angle_threshold)
{
    Vector3d pt;
    Vector3d n;
    Vector2d rad;
    for (const int& i : inds) {
        pt = points.col(i);
        n = normals.col(i);
        rad = basis.transpose()*(pt - c);
        if (fabs(rad.norm() - r) < inlier_threshold && M_PI/2.0 - acos(fabs(a.dot(n))) < angle_threshold) {
            inliers.push_back(i);
        }
    }
}

void cylinder_primitive::largest_connected_component(std::vector<int>& inliers, const MatrixXd& points)
{
    Vector2i minpt;
    minpt << INT32_MAX, int(-r*M_PI/current_connectedness_res());
    Vector2i maxpt;
    maxpt << -INT32_MAX, int(r*M_PI/current_connectedness_res());

    Vector3d pt;
    Vector2d rad;
    Vector2i pt2;
    std::vector<Vector2i, aligned_allocator<Eigen::Vector2i> > pts;
    pts.resize(conforming_inds.size());
    int counter = 0;
    for (const int& i : conforming_inds) {
        pt = points.col(i);
        rad = basis.transpose()*(pt - c);
        pt2(0) = int(a.dot(pt - c)/current_connectedness_res());
        pt2(1) = int(r*atan2(rad(0), rad(1))/current_connectedness_res());
        if (pt2(0) < minpt(0)) {
            minpt(0) = pt2(0);
        }
        if (pt2(0) > maxpt(0)) {
            maxpt(0) = pt2(0);
        }
        pts[counter] = pt2;
        ++counter;
    }

    int width = 1 + maxpt(0) - minpt(0);
    int height = 1 + maxpt(1) - minpt(1);

    if (width < 10) {
        inliers = conforming_inds;
        return;
    }

    cv::Mat binary = cv::Mat::zeros(height, width, CV_32SC1);
    //cv::Mat binary0 = cv::Mat::zeros(height, width, CV_32SC1);

    for (Vector2i& pp : pts) {
        pp -= minpt;
        binary.at<int>(pp(1), pp(0)) = 1;
        //binary0.at<int>(pp(1), pp(0)) = 65535;
    }

    //cv::imshow("Binary0", binary0);
    //cv::waitKey(0);

    cv::Mat support = cv::Mat::zeros(width, height, CV_8UC1);
    //cv::Mat binary2 = cv::Mat::zeros(height, width, CV_32SC1);

    inliers.reserve(pts.size());
    int largest = find_blobs(binary, true);
    counter = 0;
    for (const Vector2i& pp : pts) {
        if (binary.at<int>(pp(1), pp(0)) == largest) {
            inliers.push_back(conforming_inds[counter]);
            support.at<unsigned char>(pp(0), pp(1)) = 1;
            //binary2.at<int>(pp(1), pp(0)) = 65535;
        }
        ++counter;
    }

    // check how large of an angle is occupied
    // should this be done afterwards instead?
    cv::Mat row_support = cv::Mat::zeros(1, height, CV_32SC1);
    cv::reduce(support, row_support, 0, CV_REDUCE_SUM, CV_32SC1);
    int nonzero = cv::countNonZero(row_support);

    if (inlier_refinement == 1 && double(nonzero)/double(height) < 0.4) { // make this a parameter
        inliers.clear();
        return;
    }

    //std::cout << "Non zero: " << nonzero << "/" << height << std::endl;
    //cv::imshow("support", binary2);
    //cv::waitKey(0);
}

base_primitive::shape cylinder_primitive::get_shape()
{
    return CYLINDER;
}

base_primitive* cylinder_primitive::instantiate()
{
    return new cylinder_primitive();
}

void cylinder_primitive::draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    /*pcl::ModelCoefficients mc;
    Vector3d tc = c + min2(0)*a;
    Vector3d ta = c + max2(0)*a - tc;
    mc.values.resize(7);
    mc.values[0] = tc(0);
    mc.values[1] = tc(1);
    mc.values[2] = tc(2);
    mc.values[3] = ta(0);
    mc.values[4] = ta(1);
    mc.values[5] = ta(2);
    mc.values[6] = r;
    viewer->addCylinder(mc, std::string("cylinder") + boost::lexical_cast<std::string>(cylinders_drawn));
    ++cylinders_drawn;*/
}

double cylinder_primitive::distance_to_pt(const Vector3d& pt)
{
    return fabs((basis.transpose()*(pt - c)).norm() - r);
}

void cylinder_primitive::direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center)
{
    if (a(2) > 0) {
        direction = -a;
    }
    else {
        direction = a;
    }
}

void cylinder_primitive::compute_shape_size(const MatrixXd& points)
{
    double minx = INFINITY;
    double maxx = -INFINITY;

    double x;
    for (const int& i : supporting_inds) {
        x = a.dot(points.col(i));
        if (x < minx) {
            minx = x;
        }
        if (x > maxx) {
            maxx = x;
        }
    }

    double xc = a.dot(c);
    c = c + ((minx + maxx)/2.0 - xc)*a; // drawing might not work so great after this
    h = fabs(maxx - minx);
}

double cylinder_primitive::shape_size()
{
    return r;
}

void cylinder_primitive::shape_data(VectorXd& data)
{
    data.resize(8);
    data.segment<3>(0) = a;
    data.segment<3>(3) = c;
    data(6) = r;
    data(7) = h;
}

void cylinder_primitive::shape_points(std::vector<Vector3d, aligned_allocator<Vector3d> >& points)
{
    points.clear();
}
