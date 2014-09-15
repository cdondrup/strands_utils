#include "ransac_primitives/base_primitive.h"

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;

int base_primitive::min_inliers = 100;
double base_primitive::margin = 0.03;
double base_primitive::connectedness_res = 0.01;
int base_primitive::number_disjoint_subsets = 20;

// compute the confidence interval for the estimate of the number of inliers on the whole point cloud
void base_primitive::inliers_estimate(double& mean, double& a, double& b, int set_size, std::vector<int>& total_set_size)
{
    if (!interval_cached) {
        if (inlier_refinement == number_disjoint_subsets) {
            mean_cached = double(supporting_inds.size());
            a_cached = mean_cached;
            b_cached = mean_cached;
        }
        else {
            double N = -2.0 - double(total_set_size[inlier_refinement-1]);
            double x = -2.0 - double(set_size);
            double n = -1.0 - double(supporting_inds.size());
            mean_cached = -1.0 - x*n/N;
            double dev = 1.0/N*sqrt(x*n*(N-x)*(N+n)/(N-1));
            a_cached = mean_cached + dev;
            b_cached = mean_cached - dev;
        }
        interval_cached = true;
    }

    mean = mean_cached;
    a = a_cached;
    b = b_cached;
}

// compute the estimate of the number of inliers on the whole point cloud
double base_primitive::inliers_mean_estimate(int set_size, std::vector<int>& total_set_size)
{
    if (interval_cached) {
        return mean_cached;
    }
    double mean;
    double t1, t2;
    inliers_estimate(mean, t1, t2, set_size, total_set_size);
    return mean;
}


// we need the inliers to be sorted when finding mutual inliers between
// primitives via are_contained
std::vector<int>& base_primitive::sorted_inliers()
{
    if (!sorted) {
        std::sort(supporting_inds.begin(), supporting_inds.end());
        sorted = true;
    }
    return supporting_inds;
}

// how many disjoint sets we evaluated on
int base_primitive::refinement_level() const
{
    return inlier_refinement;
}

// doesn't need to exist for all subclasses
void base_primitive::compute_shape_size(const MatrixXd& points)
{

}

// look if inliers are shared between primitives
bool base_primitive::are_contained(const std::vector<int>& other_inds)
{
    if (!sorted) {
        std::sort(supporting_inds.begin(), supporting_inds.end());
        sorted = true;
    }
    // check if the primitives share any inliers
    size_t counter1 = 0;
    size_t counter2 = 0;
    while (counter1 < supporting_inds.size() && counter2 < other_inds.size()) {
        if (supporting_inds[counter1] == other_inds[counter2]) {
            return true;
        }
        if (supporting_inds[counter1] < other_inds[counter2]) {
            ++counter1;
        }
        else {
            ++counter2;
        }
    }
    return false;
}

// extract the largest connected component from an image,
// optionally connected through the sides
int base_primitive::find_blobs(cv::Mat& label_image, bool wrap_height, bool wrap_sides)
{
    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    int label_count = 2; // starts at 2 because 0,1 are used already
    std::vector<int> counts;

    for(int y = 0; y < label_image.rows; y++) {
        int* row = (int*)label_image.ptr(y);
        for(int x = 0; x < label_image.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, 4);

            int count = 0;

            for(int i = rect.y; i < (rect.y + rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for (int j = rect.x; j < (rect.x + rect.width); j++) {
                    if (row2[j] != label_count) {
                        continue;
                    }

                    ++count;
                }
            }

            counts.push_back(count);

            label_count++;
        }
    }

    if (wrap_height) { // connect blobs through top and bottom of image
        for (int x = 0; x < label_image.cols; ++x) {
            int top_value = label_image.at<int>(0, x);
            int bottom_value = label_image.at<int>(label_image.rows-1, x);
            if (top_value == 0 || bottom_value == 0 || top_value == bottom_value) {
                continue;
            }
            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x, label_image.rows-1), top_value, &rect, 0, 0, 4);
            counts[top_value-2] += counts[bottom_value-2];
            counts[bottom_value-2] = 0;
        }
    }

    // connect folded sides, first pixel on side connect with last on the same side,
    // progressing towards the middle
    if (wrap_sides) {
        for (int y = 0; y < label_image.rows/2; ++y) {
            int left_top_value = label_image.at<int>(y, 0);
            int left_bottom_value = label_image.at<int>(label_image.rows-1-y, 0);
            if (left_top_value != 0 && left_bottom_value != 0 && left_top_value != left_bottom_value) {
                cv::Rect rect;
                cv::floodFill(label_image, cv::Point(0, y), left_bottom_value, &rect, 0, 0, 4);
                counts[left_bottom_value-2] += counts[left_top_value-2];
                counts[left_top_value-2] = 0;
            }
            int right_top_value = label_image.at<int>(y, label_image.cols-1);
            int right_bottom_value = label_image.at<int>(label_image.rows-1-y, label_image.cols-1);
            if (right_top_value != 0 && right_bottom_value != 0 && right_top_value != right_bottom_value) {
                cv::Rect rect;
                cv::floodFill(label_image, cv::Point(label_image.cols-1, y), right_bottom_value, &rect, 0, 0, 4);
                counts[right_bottom_value-2] += counts[right_top_value-2];
                counts[right_top_value-2] = 0;
            }
        }
    }

    std::vector<int>::iterator result;
    result = std::max_element(counts.begin(), counts.end());

    return std::distance(counts.begin(), result) + 2;
}

// transform points on a circle to a grid coordinate system
// this is for discretization into an image
void base_primitive::circle_to_grid(Vector2d& rtn, const Vector2d onDisk)
{
    double r = onDisk.norm();
    double phi = atan2(onDisk(1) , onDisk(0));
    if (phi < -M_PI/4.0) { // in range [-pi/4,7pi/4]
        phi += 2*M_PI;
    }
    double a;
    double b;
    if (phi < M_PI/4.0) { // region 1
        a = r;
        b = phi*a/(M_PI/4.0);
    }
    else if (phi < 3.0/4.0*M_PI) { // region 2
        b = r;
        a = -(phi - M_PI/2.0)*b/(M_PI/4.0);
    }
    else if (phi < 5.0/4.0*M_PI) { // region 3
        a = -r;
        b = (phi - M_PI)*a/(M_PI/4.0);
    }
    else { // region 4
        b = -r;
        a = -(phi - 3.0*M_PI/2.0)*b/(M_PI/4.0);
    }
    rtn(0) = 0.5*(a + 1.0);
    rtn(1) = 0.5*(b + 1.0);
}

// compute the connectedness_res for given number of subsets
// that we have evaluated the primitive on
double base_primitive::current_connectedness_res()
{
    return double(number_disjoint_subsets)/double(inlier_refinement)*connectedness_res;
}

// write the inliers to a file or screen
void base_primitive::write_indices_to_stream(std::ostream& o)
{
    for (const int& ind : supporting_inds) {
        o << ind << " ";
    }
}
