#include"semantic_mesh_localization/verify_robust_change.h"

namespace semlocali{

    VerRobust::VerRobust(){
        //
    }

    VerRobust::~VerRobust(){}

    bool VerRobust::setup_robust( ros::NodeHandle& node, ros::NodeHandle& privateNode){

        if( !setup( node, privateNode)){
            ROS_ERROR("Error in VerfyCalcLikelihood");
            return false;
        }



        return true;
    }

    void VerRobust::spin_robust(){
        //


    }
}
