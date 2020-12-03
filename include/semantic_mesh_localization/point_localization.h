#ifndef __POINT_LOCALIZATION_H
#define __POINT_LOCALIZATION_H

#include "semantic_mesh_localization/mesh_localization.h"

namespace semlocali{

    class PointLocalization : public MeshLocalization{

        public:
            PointLocalization( ros::NodeHandle& node, ros::NodeHandle& privateNode);

            virtual ~PointLocalization();
            
            bool setup_point_localization( ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void spin_point_localization();

        private:
            //a
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub;

            ros::Subscriber sub_odometry;
            ros::Publisher pub_particle;
            ros::Publisher pub_pose;

            tf::TransformListener listener;
            tf::TransformBroadcaster br;
            geometry_msgs::TransformStamped car_state;

            //cv_bridge::CvImagePtr segimage;
            cv::Mat segimage; //Segmented Image from Camera data
            cv::Mat mapimage; //Image from Semantic Mesh Map

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
            int image_height = 1241;
            int image_width = 376;

            bool imheight_checker = false;
            bool imwidth_checker = false;

            //Image Downsize ratio
            double image_down_width = 1.0;
            double image_down_height = 1.0;

            bool publish_csv_checker = false;
            std::string groundtruth_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/groundtruth.csv";
            std::string odometry_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/odometry.csv";
            std::string estimated_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/estimated.csv";

            tf::StampedTransform map_to_groundtruth;

            bool read_polygon_checker = false;
            std::string polygon_data_path = "/home/amsl/Polygon_data/sequence00/";
            std::string polygon_file_name_base = "polygon_mesh_";

            bool add_bias_checker = false;
            bool step_pebbles_checker = false; //pebbles means small stone
            bool stack_muddy_checker = false; //muddy means a ground coverd with mud, and it is called Nukarumi in Japanese
            double bias_XYZ = 0.01; //10m進むごとに10cm狂うというInfantのデータをもとに
            double bias_RPY = 0.01; //1度回転するごとに0.01度のズレが発生、この値は暫定値

            double pebbles_bias_XYZ = 0.01;
            double pebbles_bias_RPY = 0.01;
            int pebbles_counter = 79;

            double get_image_vtk_time = 0.0;
            Time start_vtk;
            Time end_vtk;

            double compare_time = 0.0;
            Time start_cmp;
            Time end_cmp;

            double camera_time = 0.0;
            Time start_camera;
            Time end_camera;

            double resampling_time = 0.0;
            Time start_re;
            Time end_re;

            Time start_csv;
            Time end_csv;

            Time start_mot;
            Time end_mot;
    };
}

#endif
