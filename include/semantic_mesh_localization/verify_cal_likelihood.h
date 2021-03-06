#ifndef __VERIFY_CAL_LIKELIHOOD_H
#define __VERIFY_CAL_LIKELIHOOD_H

#define PI 3.141592

#include"ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/common/common.h> 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include"pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"util.h"
#include"pcl/common/common.h"
#include"pcl/features/normal_3d.h"
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/gp3.h>
#include"sensor_msgs/PointCloud2.h"
#include"pcl_conversions/pcl_conversions.h"
#include"nav_msgs/Odometry.h"
#include "tf_conversions/tf_eigen.h"
#include"tf/transform_broadcaster.h"
#include"tf/transform_listener.h"
#include <vtkRenderWindow.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/image_transport.h>
#include"pcl/features/normal_3d_omp.h"
#include<random>
#include<iostream>
#include<fstream>
#include<stdlib.h>
#include"geometry_msgs/PoseArray.h"
#include"geometry_msgs/PoseStamped.h"
#include <pcl/console/parse.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

namespace semlocali{
    
    class VerCalLike{

        public:
            VerCalLike();

            virtual ~VerCalLike();

            bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void spin();

            void process();

            void load_place_data();

            void load_bgr_image();

            std::vector<std::string> split_v( std::string& input, char delimiter);

            void load_seg_image();

            void config_viewer_parameter( pcl::visualization::PCLVisualizer& viewer);

            void load_point_cloud( pcl::visualization::PCLVisualizer& viewer);

            void load_PCD();

            void classify_pointcloud();

            bool classify(pcl::PointXYZRGB& point, const color_data& color_id);

            void add_semantic_pc(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, const pcl::PointCloud<pcl::PointXYZRGB> semantic_cloud, int blue, int green, int red);

            void load_mesh( pcl::visualization::PCLVisualizer& viewer);

            void load_PLY(pcl::visualization::PCLVisualizer& viewer);

            void load_semantic_polygon(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, int blue, int green, int red);

            void get_map_image();

            cv::Mat get_image( pcl::visualization::PCLVisualizer& viewer, geometry_msgs::Pose pose, int counter, std::string file_dir);

            void verify_compare( std::vector<cv::Mat> segmented_image, std::vector<cv::Mat> image_row, std::string image_file_path, std::string type);

            cv::Mat compare_image( cv::Mat segimage, cv::Mat mapimage, std::string type);

            void save_csv();

            double cmp_pixel(double seg_b,double seg_g,double seg_r,double map_b,double map_g, double map_r);

            void create_integrated_image();

        public:

            pcl::visualization::PCLVisualizer point_viewer;
            pcl::visualization::PCLVisualizer mesh_viewer;

            std::string place_data_path = "/home/amsl/semantic_mesh_localization_data/sequence00/";
            std::string image_data_path = "/home/amsl/semantic_mesh_localization_data/sequence00_init/segimage/";
            std::string bgr_image_data_path = "/home/amsl/semantic_mesh_localization_data/sequence00_init/bgrimage/";
           std::string mesh_map_image_path = "/home/amsl/semantic_mesh_localization_data/sequence00/mesh_map_image/";
            std::string point_map_image_path = "/home/amsl/semantic_mesh_localization_data/sequence00/point_map_image/";
            std::string valued_mesh_map_image_path ="/home/amsl/semantic_mesh_localization/sequence00/valued_mesh_map_image/";
            std::string valued_point_map_image_path ="/home/amsl/semantic_mesh_localization/sequence00/valued_point_map_image/";
             std::string integrated_image_path ="/home/amsl/semantic_mesh_localization/sequence00/integrated_image/";
           std::string likelihood_csv_file_path = "/home/amsl/semantic_mesh_localization/sequence00/";

            std::string pcd_path = "/home/amsl/PCD_data/sequence00_12_05_1830/";
            std::string pcd_name = "semantic_mesh_loam_ascii";
            int load_pcd_counter = 0;

            std::string polygon_data_path = "/home/amsl/Polygon_data/sequence00_12_05_1830/";
            std::string polygon_file_name_base = "polygon_mesh_";

            std::vector<cv::Mat> seg_image;
            std::vector<cv::Mat> bgr_image;
            std::vector<cv::Mat> mesh_map_image;
            std::vector<cv::Mat> point_map_image;
            
            std::vector<geometry_msgs::Pose> agent_place;

            /*
            std::vector<double> seg_image_bgr;
            std::vector<double> mesh_image_bgr;
            std::vector<double> point_image_bgr;
            */

            int data_length = 0;

            int image_width = 0;
            int image_height = 0;

            pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;

            std::vector<double> likelihoods; //upper half:mesh lower half:pointcloud
            std::vector<double> pixels;
            size_t cmp_times = 0;

            pcl::PointCloud<pcl::PointXYZRGB> unlabeled;
            pcl::PointCloud<pcl::PointXYZRGB> outlier;
            pcl::PointCloud<pcl::PointXYZRGB> car;
            pcl::PointCloud<pcl::PointXYZRGB> bicycle;
            pcl::PointCloud<pcl::PointXYZRGB> bus;
            pcl::PointCloud<pcl::PointXYZRGB> motorcycle;
            pcl::PointCloud<pcl::PointXYZRGB> onrails;
            pcl::PointCloud<pcl::PointXYZRGB> truck;
            pcl::PointCloud<pcl::PointXYZRGB> othervehicle;
            pcl::PointCloud<pcl::PointXYZRGB> person;
            pcl::PointCloud<pcl::PointXYZRGB> bicyclist;
            pcl::PointCloud<pcl::PointXYZRGB> motorcyclist;
            pcl::PointCloud<pcl::PointXYZRGB> road;
            pcl::PointCloud<pcl::PointXYZRGB> parking;
            pcl::PointCloud<pcl::PointXYZRGB> sidewalk;
            pcl::PointCloud<pcl::PointXYZRGB> otherground;
            pcl::PointCloud<pcl::PointXYZRGB> building;
            pcl::PointCloud<pcl::PointXYZRGB> fence;
            pcl::PointCloud<pcl::PointXYZRGB> otherstructure;
            pcl::PointCloud<pcl::PointXYZRGB> lanemarking;
            pcl::PointCloud<pcl::PointXYZRGB> vegetation;
            pcl::PointCloud<pcl::PointXYZRGB> trunk;
            pcl::PointCloud<pcl::PointXYZRGB> terrain;
            pcl::PointCloud<pcl::PointXYZRGB> pole;
            pcl::PointCloud<pcl::PointXYZRGB> trafficsign;

            double point_size = 1.0;

    };
}

#endif
