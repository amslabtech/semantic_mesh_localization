#ifndef __MESH_LOCALIZATION_H
#define __MESH_LOCALIZATION_H

#define PI 3.141592

#include"semantic_mesh_localization/pointcloud_to_mesh.h"

#include<random>
#include<iostream>
#include<fstream>
#include<stdlib.h>

#include"geometry_msgs/PoseArray.h"
#include"geometry_msgs/PoseStamped.h"



namespace semlocali{

    class MeshLocalization : public PcToMesh{

        public:

            MeshLocalization(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            virtual ~MeshLocalization();

            void segmented_image_callback(const sensor_msgs::ImageConstPtr& msg);

            void odometry_callback(const nav_msgs::OdometryConstPtr& msg);

            void add_bias_to_odometry(pos_trans& odom_trans);

            double add_bias_XYZ(double dt, int random_value);

            double add_bias_XY(double dt, int random_value);

            double add_bias_Z(double dt, int random_value);

            double add_bias_RPY(double dt, int random_value);

            pos_trans get_relative_trans(nav_msgs::Odometry odom, nav_msgs::Odometry last_odom);

            bool setup_mesh_localization(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void build_mesh_map();

            void read_mesh_map();

            void load_PLY(pcl::visualization::PCLVisualizer& viewer);

            void load_semantic_polygon(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, int blue, int green, int red);

            void spin_mesh_localization();

            void wait_for_bag_data();

            void init_odometry_process();

            void particle_filter();

            void motion_update();

            double rand_delta_XYZ(double ave, double dev);

            double rand_delta_RPY(double ave, double dev);

            void update_likelihood();

            double get_likelihood(geometry_msgs::Pose pose);

            void change_camera_position_for_particle(geometry_msgs::Pose pose);

            void get_image_from_pcl_visualizer();

            double compare_image_pixelwise();

            void estimate_current_pose();

            geometry_msgs::PoseStamped max_likelihood_approach();

            void resampling_particle();

            void publish_result();

            void publish_as_csv(std::ofstream& groundtruth_csv, std::ofstream& odometry_csv, std::ofstream& estimated_csv, std::ofstream& biased_odom_csv);

            void save_image();

        public:

            vtkSmartPointer<vtkRenderWindow> render;
            cv::Mat tmpimage;

            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub;

            ros::Subscriber sub_odometry;
            ros::Publisher pub_particle;
            ros::Publisher pub_pose;

            ros::Publisher pub_biased_odometry;
            nav_msgs::Odometry biased_odom;

            tf::TransformListener listener;
            tf::TransformBroadcaster br;
            geometry_msgs::TransformStamped car_state;

            bool build_polygon_checker = true;
            bool read_polygon_checker = false;

            pcl::visualization::PCLVisualizer viewer;

            //Image size data
            int image_height = 1241;
            int image_width = 376;

            bool imheight_checker = false;
            bool imwidth_checker = false;

            //Image Downsize ratio
            double image_down_width = 1.0;
            double image_down_height = 1.0;

            //cv_bridge::CvImagePtr segimage;
            cv::Mat segimage; //Segmented Image from Camera data
            cv::Mat mapimage; //Image from Semantic Mesh Map

            nav_msgs::Odometry odom_data;
            nav_msgs::Odometry last_odom_data;
            pos_trans odom_trans;
            bool first_odom_checker = false;

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

            bool publish_csv_checker = false;
            std::string groundtruth_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/groundtruth.csv";
            std::string odometry_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/odometry.csv";
            std::string estimated_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/estimated.csv";
            std::string biased_odom_path = "/home/amsl/catkin_ws/src/semantic_mesh_localization/output_data/biased_odom.csv";

            bool save_image_checker = false;
            std::string seg_image_path = "/home/amsl/Image_data/segimage/";
            std::string map_image_path = "/home/amsl/Image_data/mapimage/";
            int image_counter = 0;

            tf::StampedTransform map_to_groundtruth;

            std::string polygon_data_path = "/home/amsl/Polygon_data/sequence00/";
            std::string polygon_file_name_base = "polygon_mesh_";

            bool add_bias_checker = false;
            bool step_pebbles_checker = false; //pebbles means small stone
            bool stack_muddy_checker = false; //muddy means a ground coverd with mud, and it is called Nukarumi in Japanese

            double bias_XYZ = 0.01; //10m進むごとに10cm狂うというInfantのデータをもとに
            double bias_XY = 0.01;
            double bias_Z = 0.01;

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
