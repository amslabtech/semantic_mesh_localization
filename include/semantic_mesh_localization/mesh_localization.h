#ifndef __MESH_LOCALIZATION_H
#define __MESH_LOCALIZATION_H

#define PI 3.141592

#include"semantic_mesh_localization/pointcloud_to_mesh.h"

#include<random>
#include<fstream>
#include<stdlib.h>

#include"geometry_msgs/PoseArray.h"
#include"geometry_msgs/PoseStamped.h"



namespace semloam{

    class MeshLocalization : public PcToMesh{

        public:

            MeshLocalization(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            virtual ~MeshLocalization();

            void segmented_image_callback(const sensor_msgs::ImageConstPtr& msg);

            void odometry_callback(const nav_msgs::OdometryConstPtr& msg);

            pos_trans get_relative_trans(nav_msgs::Odometry odom, nav_msgs::Odometry last_odom);

            bool setup_mesh_localization(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void build_mesh_map();

            void spin_mesh_localization();

            void wait_for_bag_data();

            void init_odometry_process();

            void particle_filter();

            void motion_update();

            double rand_delta(double ave, double dev);

            void update_likelihood();

            double get_likelihood(geometry_msgs::Pose pose);

            void estimate_current_pose();

            geometry_msgs::PoseStamped max_likelihood_approach();

            void resampling_particle();

            void publish_result();

        private:
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub;

            ros::Subscriber sub_odometry;
            ros::Publisher pub_particle;
            ros::Publisher pub_pose;

            tf::TransformListener listener;
            tf::TransformBroadcaster br;
            geometry_msgs::TransformStamped car_state;

            //cv_bridge::CvImagePtr segimage;
            cv::Mat segimage;

            nav_msgs::Odometry odom_data;
            nav_msgs::Odometry last_odom_data;
            pos_trans odom_trans;
            bool first_odom_checker = false;

            pcl::visualization::PCLVisualizer viewer;

            int particlenumber = 50;//launch particlenumber

            geometry_msgs::PoseStamped estimated_pose; //estimated vehicle pose
            geometry_msgs::PoseArray particle; // particle
            std::vector<double> likelihood;

            //for motion_update() function
            double dx_dev = 0.1; //launch xdev
            double dy_dev = 0.1; //launch ydev
            double dz_dev = 0.1; //launch zdev

            double droll_dev = 0.01; //launch rolldev
            double dpitch_dev = 0.01; //launch pitchdev
            double dyaw_dev = 0.01; //launch yawdev

            //Image size data
            int image_height = 376;
            int image_width = 1241;


    };

}

#endif
