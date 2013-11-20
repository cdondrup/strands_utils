
#include <move_back_recovery/move_back_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(move_back_recovery, MoveBackRecovery, move_back_recovery::MoveBackRecovery, nav_core::RecoveryBehavior)

namespace move_back_recovery {
    MoveBackRecovery::MoveBackRecovery(): global_costmap_(NULL), local_costmap_(NULL),
        tf_(NULL), initialized_(false), world_model_(NULL) {}

    void MoveBackRecovery::initialize(std::string name, tf::TransformListener* tf,
                                      costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
        if(!initialized_){
            name_ = name;
            tf_ = tf;
            global_costmap_ = global_costmap;
            local_costmap_ = local_costmap;

            //get some parameters from the parameter server
            ros::NodeHandle private_nh("~/" + name_);
            ros::NodeHandle planner_nh("~/TrajectoryPlannerROS");

            //we'll simulate every degree by default
            private_nh.param("sim_granularity", sim_granularity_, 0.1);
            private_nh.param("frequency", frequency_, 20.0);

            planner_nh.param("distance", dist_, 20.0);
            //    planner_nh.param("max_trans_vel", max_trans_vel_, 1.0);
            //    planner_nh.param("min_trans_vel", min_trans_vel_, 0.4);
            //    planner_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

            local_costmap_->getCostmapCopy(costmap_);
            world_model_ = new base_local_planner::CostmapModel(costmap_);
            tpr_ = new base_local_planner::TrajectoryPlannerROS("TPR", tf_, local_costmap_);

            initialized_ = true;
        }
        else{
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    MoveBackRecovery::~MoveBackRecovery(){
        delete world_model_;
    }

    void MoveBackRecovery::runBehavior(){
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        if(global_costmap_ == NULL || local_costmap_ == NULL){
            ROS_ERROR("The costmaps passed to the MoveBackRecovery object cannot be NULL. Doing nothing.");
            return;
        }
        ROS_WARN("Move back recovery behavior started.");

        ros::Rate r(frequency_);
        ros::NodeHandle n;
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        tf_->waitForTransform("/base_link", "/odom",
                              ros::Time(0), ros::Duration(1.0));

        tf::Stamped<tf::Pose> global_pose;
        local_costmap_->getRobotPose(global_pose);
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;
        tf_->lookupTransform("/base_link", "/odom",
                             ros::Time(0), start_transform);


        //  double current_angle = -1.0 * M_PI;

        //  bool got_180 = false;
        //  bool moved_back = false;

        //  double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
        while(n.ok()){
            local_costmap_->getRobotPose(global_pose);

            try {
                tf_->lookupTransform("/base_link", "/odom", ros::Time(0), current_transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                stopRobot(vel_pub);
                continue;
            }

            tf::Transform relative_transform = start_transform.inverse() * current_transform;
            double dist_moved = relative_transform.getOrigin().length();

            //compute the distance left to rotate
            double dist_left = dist_moved - dist_;

            //update the costmap copy that the world model holds
            local_costmap_->getCostmapCopy(costmap_);

            //check if that velocity is legal by forward simulating
            double sim_dist = 0.0;
            while(sim_dist < dist_left){
                std::vector<geometry_msgs::Point> oriented_footprint;
                double theta = tf::getYaw(global_pose.getRotation());

                geometry_msgs::Point position;
                position.x = global_pose.getOrigin().x() + sim_dist * cos(theta + M_PI);
                position.y = global_pose.getOrigin().y() + sim_dist * sin(theta + M_PI);

                local_costmap_->getOrientedFootprint(position.x, position.y, theta, oriented_footprint);

                //make sure that the point is legal, if it isn't... we'll abort
                double footprint_cost = world_model_->footprintCost(position, oriented_footprint, local_costmap_->getInscribedRadius(), local_costmap_->getCircumscribedRadius());
                if(footprint_cost < 0.0){
                    ROS_ERROR("Move back recovery can't rotate in place because there is a potential collision. Cost: %.2f", footprint_cost);
                    return;
                }

                sim_dist += sim_granularity_;
            }

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = -0.25;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;

            vel_pub.publish(cmd_vel);



            if(dist_moved > dist_) {
                stopRobot(vel_pub);
                return;
            }

            r.sleep();
        }
    }

    void MoveBackRecovery::stopRobot(ros::Publisher &vel_pub) {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        vel_pub.publish(cmd_vel);
    }
};
