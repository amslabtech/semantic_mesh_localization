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

    void VerLocali::calc_difference(){
        //a

    }

    void VerLocali::take_statistics(){
        //a

    }

    void VerLocali::publish_as_csv(){
        //a
        
    }


}
