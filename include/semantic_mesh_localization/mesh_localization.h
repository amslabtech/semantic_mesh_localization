#ifndef __MESH_LOCALIZATION_H
#define __MESH_LOCALIZATION_H

#define PI 3.141592

#include"semantic_mesh_localization/pointcloud_to_mesh.h"

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

        private:
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub;

            ros::Subscriber sub_odometry;

            cv_bridge::CvImagePtr segimage;

            nav_msgs::Odometry odom_data;
            nav_msgs::Odometry last_odom_data;
            pos_trans odom_trans;
            bool first_odom_checker = false;

            pcl::visualization::PCLVisualizer viewer;

            int particlenumber = 50;//launch particlenumber

            geometry_msgs::PoseStamped estimated_pose; //estimated vehicle pose
            geometry_msgs::PoseArray particle; // particle
            std::vector<double> likelihood;
    };

}

#endif
