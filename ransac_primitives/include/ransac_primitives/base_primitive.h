#ifndef BASE_PRIMITIVE_H
#define BASE_PRIMITIVE_H

#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/visualization/pcl_visualizer.h>

class base_primitive
{
private:
    double a_cached; // lower bound of inliers confidence interval
    double b_cached; // upper bound of inliers confidence interval
    double mean_cached; // mean estimate of inliers
    bool interval_cached;
public:
    // the different kinds of primitives that we are treating, only used for coloring
    enum shape {PLANE, SPHERE, CYLINDER, TORUS, CONE};
    std::vector<int> supporting_inds; // the inliers, i.e. points matching formula and connected
    std::vector<int> conforming_inds; // the points matching the analytic formula of the primitive
    int inlier_refinement; // the number of subsets that the primitive has been evaluated on
    bool sorted; // is supporting_inds sorted? needed for checking common inliers
    u_char red, green, blue; // colors used when displaying the primitive
    static int number_disjoint_subsets;
    static int min_inliers; // the minimum inliers required to check connectedness, skip primitive otherwise
    static double margin; // the margin used when discarding octree nodes by looking at the primitives
    static double connectedness_res; // the resolution in the discretized image where we check for connectedness
    int get_inliers() { return supporting_inds.size(); }
    // use the octree for this also, since they are all from one node... can do that as a previous step
    bool are_contained(const std::vector<int>& other_inds);
    int find_blobs(cv::Mat& label_image, bool wrap_height = false, bool wrap_sides = false);
    void circle_to_grid(Eigen::Vector2d& rtn, const Eigen::Vector2d onDisk);
    double current_connectedness_res();
    // output the indices in the point cloud contained in the primitive
    void write_indices_to_stream(std::ostream& o);
    // create a primitive, points_required points and normals needed for the operation
    virtual bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                           double inlier_threshold, double angle_threshold) = 0;
    /*void refine_inliers(std::vector<primitive_octree>& octrees, Eigen::MatrixXd& points,
                        Eigen::MatrixXd& normals, double inlier_threshold, double angle_threshold);
    void final_inliers(primitive_octree& octree, Eigen::MatrixXd& points, Eigen::MatrixXd& normals,
                       double inlier_threshold, double angle_threshold);*/
    int refinement_level() const;
    virtual void largest_connected_component(std::vector<int>& inliers, const Eigen::MatrixXd& points) = 0;
    std::vector<int>& sorted_inliers();
    void inliers_estimate(double& mean, double& a, double& b, int set_size, std::vector<int>& total_set_size);
    double inliers_mean_estimate(int set_size, std::vector<int>& total_set_size);
    // check for primitives, takes all points and normals considered and the indices that are still unoccupied by primitives
    virtual void compute_inliers(std::vector<int>& inliers, const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                                 const std::vector<int>& inds, double inlier_threshold, double angle_threshold) = 0;
    // the number of points needed for construct
    virtual int points_required() = 0;
    // check if the shape is on either side of xyz, the center of a box with side length l,
    // used for effective search for primitive inliers in an octree
    virtual double distance_to_pt(const Eigen::Vector3d& pt) = 0;
    // used to find the relative angles between primitives
    // TODO: change direction to orientation with Matrix3d instead
    virtual void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center) = 0;
    virtual double shape_size() = 0;
    virtual void shape_data(Eigen::VectorXd& data) = 0;
    virtual void shape_points(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points) = 0;
    virtual void compute_shape_size(const Eigen::MatrixXd& points);
    // returns the shape type, only used for coloring atm
    virtual shape get_shape() = 0;
    // draw the shape in a pcl visualizer window
    virtual void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) = 0;
    // create a new primitive of the subclass
    virtual base_primitive* instantiate() = 0;
    virtual ~base_primitive() {}
    base_primitive() : inlier_refinement(0) {}

    // compute the number of inliers on the whole point cloud
    template <typename Octree>
    void final_inliers(Octree& octree, Eigen::MatrixXd& points, Eigen::MatrixXd& normals,
                       double inlier_threshold, double angle_threshold)
    {
        if (inlier_refinement == number_disjoint_subsets) {
            return;
        }
        interval_cached = false;
        inlier_refinement = number_disjoint_subsets;
        std::vector<int> inds;
        octree.find_potential_inliers(inds, this, 0.01);
        conforming_inds.clear();
        compute_inliers(conforming_inds, points, normals, inds, inlier_threshold, angle_threshold);
        supporting_inds.clear();
        std::vector<int> temp;
        largest_connected_component(temp, points);
        supporting_inds.swap(temp);
        std::sort(supporting_inds.begin(), supporting_inds.end());
        sorted = true;
    }

    template <typename Octree>
    void refine_inliers(std::vector<Octree>& octrees, Eigen::MatrixXd& points,
                        Eigen::MatrixXd& normals, double inlier_threshold, double angle_threshold)
    {
        if (inlier_refinement == number_disjoint_subsets) {
            return;
        }
        interval_cached = false;
        ++inlier_refinement; // check if larger than disjoint sets?
        std::vector<int> inds;
        int n = inlier_refinement - 1;
        octrees[n].find_potential_inliers(inds, this, 0.01);
        std::vector<int> inliers;
        compute_inliers(inliers, points, normals, inds, inlier_threshold, angle_threshold);
        if (inliers.size() < 40) {
            return;
        }
        conforming_inds.insert(conforming_inds.end(), inliers.begin(), inliers.end());
        /*if (conforming_inds.size() < min_inliers) {

        }*/
        if (inlier_refinement == 1 || inlier_refinement < number_disjoint_subsets/2) {
            supporting_inds = conforming_inds;
        }
        else {
            inliers.clear();
            largest_connected_component(inliers, points);
            supporting_inds.swap(inliers);
        }
        sorted = false;
    }
};

#endif // BASE_PRIMITIVE_H
