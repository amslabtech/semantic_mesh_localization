#ifndef __VERIFY_LOCALIZATION_H
#define __VERIFY_LOCALIZATION_H

#define PI 3.141592

#include "ros/ros.h"
#include<random>
#include<iostream>
#include<fstream>
#include<stdlib.h>

#include"util_verify_localization.h"

namespace semlocali{

    class VerLocali{

        public:
            VerLocali();

            virtual ~VerLocali();

            bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void spin();

            void load_csv_file();

            std::vector<std::string> split_v( std::string& input, char delimiter);

            void calc_difference();

            void write_difference( std::vector<csv_data> pose_data, std::vector<csv_data> ground_truth, std::vector<diff_place>& diff_vector);

            void take_statistics();

            void make_stat(std::vector<diff_place> diff_data, diff_stat& stat_data);

            void publish_as_csv();

            void publish_result(diff_stat stat, std::string file_name);

        private:
            std::string data_path = "/home/amsl/";

            std::string biased_odom_file;
            std::string estimated_pose_file;
            std::string ground_truth_file; //odometry.csv

            std::vector<csv_data> biased_odom_data;
            std::vector<csv_data> estimated_pose_data;
            std::vector<csv_data> ground_truth_data;

            std::vector<diff_place> est_to_gt;//Difference between estimated pose and ground truth
            std::vector<diff_place> bio_to_gt;//Difference between biased odom and ground truth

            diff_stat est_to_gt_stat;
            diff_stat bio_to_gt_stat;
    };

}

#endif
