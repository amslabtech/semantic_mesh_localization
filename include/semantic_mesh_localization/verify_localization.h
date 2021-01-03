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

        private:
            std::string data_path = "/home/amsl/";

            std::string biased_odom_file;
            std::string estimated_pose_file;
            std::string ground_truth_file; //odometry.csv

            std::vector<csv_data> biased_odom_data;
            std::vector<csv_data> estimated_pose_data;
            std::vector<csv_data> ground_truth_data;





    };

}

#endif
