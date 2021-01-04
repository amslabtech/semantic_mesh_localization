#include "semantic_mesh_localization/verify_localization.h"

namespace semlocali{

    VerLocali::VerLocali(){
        //Do nothing
    }

    VerLocali::~VerLocali(){}

    bool VerLocali::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

        std::string sparam;

        if(privateNode.getParam("DataPath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                data_path = sparam;
            }
        }


        biased_odom_file = data_path + "biased_odom.csv";
        ground_truth_file = data_path + "odometry.csv";
        estimated_pose_file = data_path + "estimated.csv";

        return true;
    }

    void VerLocali::spin(){

        std::cout << "Load CSV file" << std::endl;
        load_csv_file();

        std::cout << "Calc difference between ground truth and estimated pose" << std::endl;
        calc_difference();

        std::cout << "Take statistics" << std::endl;
        take_statistics();

        std::cout << "Publish difference data as csv file" << std::endl;
        publish_as_csv();

    }

    std::vector<std::string> VerLocali::split_v( std::string& input, char delimiter){

        std::istringstream stream(input);
        std::string field;

        std::vector<std::string> result;

        while( getline(stream, field, delimiter) ){
            result.push_back(field);
        }

        return result;
    }

    void VerLocali::load_csv_file(){

        std::string line;
        csv_data tmp_csv_data;

        std::ifstream ifs_biased_odom( biased_odom_file );
        std::ifstream ifs_ground_truth( ground_truth_file );
        std::ifstream ifs_estimated_pose( estimated_pose_file );

        while( getline( ifs_biased_odom , line ) ){
            std::vector<std::string> strvec = split_v( line, ',');

            tmp_csv_data.time = std::stod(strvec[0]);
            
            tmp_csv_data.x    = std::stod(strvec[1]);
            tmp_csv_data.y    = std::stod(strvec[2]);
            tmp_csv_data.z    = std::stod(strvec[3]);

            tmp_csv_data.qx   = std::stod(strvec[4]);
            tmp_csv_data.qy   = std::stod(strvec[5]);
            tmp_csv_data.qz   = std::stod(strvec[6]);
            tmp_csv_data.qw   = std::stod(strvec[7]);

            biased_odom_data.push_back(tmp_csv_data);
        }

        while( getline( ifs_ground_truth , line ) ){
            std::vector<std::string> strvec = split_v( line, ',');

            tmp_csv_data.time = std::stod(strvec[0]);
            
            tmp_csv_data.x    = std::stod(strvec[1]);
            tmp_csv_data.y    = std::stod(strvec[2]);
            tmp_csv_data.z    = std::stod(strvec[3]);

            tmp_csv_data.qx   = std::stod(strvec[4]);
            tmp_csv_data.qy   = std::stod(strvec[5]);
            tmp_csv_data.qz   = std::stod(strvec[6]);
            tmp_csv_data.qw   = std::stod(strvec[7]);

            ground_truth_data.push_back(tmp_csv_data);
        }

        while( getline( ifs_estimated_pose , line ) ){
            std::vector<std::string> strvec = split_v( line, ',');

            tmp_csv_data.time = std::stod(strvec[0]);
            
            tmp_csv_data.x    = std::stod(strvec[1]);
            tmp_csv_data.y    = std::stod(strvec[2]);
            tmp_csv_data.z    = std::stod(strvec[3]);

            tmp_csv_data.qx   = std::stod(strvec[4]);
            tmp_csv_data.qy   = std::stod(strvec[5]);
            tmp_csv_data.qz   = std::stod(strvec[6]);
            tmp_csv_data.qw   = std::stod(strvec[7]);

            estimated_pose_data.push_back(tmp_csv_data);
        }
    }

    void VerLocali::write_difference( std::vector<csv_data> pose_data, std::vector<csv_data> ground_truth, std::vector<diff_place>& diff_vector){

        double dx2, dy2, dz2;
        double in_pro; //naiseki
        double diff_angle;

        diff_place diff;

        for(size_t i=0; i<pose_data.size(); i++){
            dx2 = (pose_data[i].x - ground_truth[i].x) * (pose_data[i].x - ground_truth[i].x);
            dy2 = (pose_data[i].y - ground_truth[i].y) * (pose_data[i].y - ground_truth[i].y);
            dz2 = (pose_data[i].z - ground_truth[i].z) * (pose_data[i].z - ground_truth[i].z);

            diff.pose = std::sqrt( dx2 + dy2 + dz2 );

            in_pro = pose_data[i].qx*ground_truth[i].qx + pose_data[i].qy*ground_truth[i].qy + pose_data[i].qz*ground_truth[i].qz + pose_data[i].qw*ground_truth[i].qw;

            diff.angle = std::acos(in_pro) * 180.0 / PI;

            diff_vector.push_back(diff);
        }
        /*
        std::cout << diff_vector[47].pose << std::endl;
        std::cout << diff_vector[47].angle << std::endl;
        */
    }

    void VerLocali::calc_difference(){
        if(     estimated_pose_data.size() == ground_truth_data.size() &&
                biased_odom_data.size() == ground_truth_data.size() ){
            
            write_difference(estimated_pose_data, ground_truth_data, est_to_gt);
            write_difference(   biased_odom_data, ground_truth_data, bio_to_gt);
            /*
            std::cout << est_to_gt[47].pose << std::endl;
            std::cout << est_to_gt[47].angle << std::endl;
            */
        }
        else{
            ROS_ERROR("Size is different");
        }
    }

    void VerLocali::make_stat(std::vector<diff_place> diff_data, diff_stat& stat_data){
        
        stat_data.scantimes = int(diff_data.size());

        for(size_t i=0; i<diff_data.size(); i++){
            if(diff_data[i].pose < 0.5){
                stat_data.time_XYZ_u05 += 1;
            }
            else if(diff_data[i].pose < 1.0){
                stat_data.time_XYZ_u10 += 1;
            }
            else if(diff_data[i].pose < 1.5){
                stat_data.time_XYZ_u15 += 1;
            }
            else if(diff_data[i].pose < 2.0){
                stat_data.time_XYZ_u20 += 1;
            }
            else if(diff_data[i].pose < 2.5){
                stat_data.time_XYZ_u25 += 1;
            }
            else if(diff_data[i].pose < 3.0){
                stat_data.time_XYZ_u30 += 1;
            }
            else if(diff_data[i].pose < 3.5){
                stat_data.time_XYZ_u35 += 1;
            }
            else if(diff_data[i].pose < 4.0){
                stat_data.time_XYZ_u40 += 1;
            }
            else{
                stat_data.time_XYZ_o40 += 1;
            }

            if(diff_data[i].angle < 1.0){
                stat_data.time_RPY_u01 += 1;
            }
            else if(diff_data[i].angle < 5.0){
                stat_data.time_RPY_u05 += 1;
            }
            else if(diff_data[i].angle < 10.0){
                stat_data.time_RPY_u10 += 1;
            }
            else if(diff_data[i].angle < 15.0){
                stat_data.time_RPY_u15 += 1;
            }
            else if(diff_data[i].angle < 20.0){
                stat_data.time_RPY_u20 += 1;
            }
            else if(diff_data[i].angle < 25.0){
                stat_data.time_RPY_u25 += 1;
            }
            else if(diff_data[i].angle < 30.0){
                stat_data.time_RPY_u30 += 1;
            }
            else{
                stat_data.time_RPY_o30 += 1;
            }
        }

        double t = double(stat_data.scantimes);

        stat_data.ratio_XYZ_u05 = double(stat_data.time_XYZ_u05)/t;
        stat_data.ratio_XYZ_u10 = double(stat_data.time_XYZ_u10)/t;
        stat_data.ratio_XYZ_u15 = double(stat_data.time_XYZ_u15)/t;
        stat_data.ratio_XYZ_u20 = double(stat_data.time_XYZ_u20)/t;
        stat_data.ratio_XYZ_u25 = double(stat_data.time_XYZ_u25)/t;
        stat_data.ratio_XYZ_u30 = double(stat_data.time_XYZ_u30)/t;
        stat_data.ratio_XYZ_u35 = double(stat_data.time_XYZ_u35)/t;
        stat_data.ratio_XYZ_u40 = double(stat_data.time_XYZ_u40)/t;
        stat_data.ratio_XYZ_o40 = double(stat_data.time_XYZ_o40)/t;

        stat_data.ratio_RPY_u01 = double(stat_data.time_RPY_u01)/t;
        stat_data.ratio_RPY_u05 = double(stat_data.time_RPY_u05)/t;
        stat_data.ratio_RPY_u10 = double(stat_data.time_RPY_u10)/t;
        stat_data.ratio_RPY_u15 = double(stat_data.time_RPY_u15)/t;
        stat_data.ratio_RPY_u20 = double(stat_data.time_RPY_u20)/t;
        stat_data.ratio_RPY_u25 = double(stat_data.time_RPY_u25)/t;
        stat_data.ratio_RPY_u30 = double(stat_data.time_RPY_u30)/t;
        stat_data.ratio_RPY_o30 = double(stat_data.time_RPY_o30)/t;
    }

    void VerLocali::take_statistics(){
        
        make_stat(est_to_gt, est_to_gt_stat);
        make_stat(bio_to_gt, bio_to_gt_stat);

    }

    void VerLocali::publish_result(diff_stat stat, std::string file_name){
        std::string file_path = data_path + file_name + ".csv";

        std::ofstream csv_publisher(file_path);
        csv_publisher << "XYZ u 0.5m" << "," << stat.ratio_XYZ_u05 << "," << std::endl;
        csv_publisher << "XYZ u 1.0m" << "," << stat.ratio_XYZ_u10 << "," << std::endl;
        csv_publisher << "XYZ u 1.5m" << "," << stat.ratio_XYZ_u15 << "," << std::endl;
        csv_publisher << "XYZ u 2.0m" << "," << stat.ratio_XYZ_u20 << "," << std::endl;
        csv_publisher << "XYZ u 2.5m" << "," << stat.ratio_XYZ_u25 << "," << std::endl;
        csv_publisher << "XYZ u 3.0m" << "," << stat.ratio_XYZ_u30 << "," << std::endl;
        csv_publisher << "XYZ u 3.5m" << "," << stat.ratio_XYZ_u35 << "," << std::endl;
        csv_publisher << "XYZ u 4.0m" << "," << stat.ratio_XYZ_u40 << "," << std::endl;
        csv_publisher << "XYZ o 4.0m" << "," << stat.ratio_XYZ_o40 << "," << std::endl;

        csv_publisher <<  "RPY u 1.0 deg" << "," << stat.ratio_RPY_u01 << "," << std::endl;
        csv_publisher <<  "RPY u 5.0 deg" << "," << stat.ratio_RPY_u05 << "," << std::endl;
        csv_publisher << "RPY u 10.0 deg" << "," << stat.ratio_RPY_u10 << "," << std::endl;
        csv_publisher << "RPY u 15.0 deg" << "," << stat.ratio_RPY_u15 << "," << std::endl;
        csv_publisher << "RPY u 20.0 deg" << "," << stat.ratio_RPY_u20 << "," << std::endl;
        csv_publisher << "RPY u 25.0 deg" << "," << stat.ratio_RPY_u25 << "," << std::endl;
        csv_publisher << "RPY u 30.0 deg" << "," << stat.ratio_RPY_u30 << "," << std::endl; 
        csv_publisher << "RPY o 30.0 deg" << "," << stat.ratio_RPY_o30 << "," << std::endl;

       csv_publisher.close();
    }

    void VerLocali::publish_as_csv(){
        
        publish_result(est_to_gt_stat, "estimated_pose_to_ground_truth");
        publish_result(bio_to_gt_stat, "biased_odom_to_ground_truth");
        
    }


}
