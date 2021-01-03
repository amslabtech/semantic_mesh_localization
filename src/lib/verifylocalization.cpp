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
        //a

    }

}
