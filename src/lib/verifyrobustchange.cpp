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
        process_robust();
    }

    void VerRobust::process_robust(){

        std::cout << "Loading place data" << std::endl;
        load_place_data();

        std::cout << "Loading segmented image data" << std::endl;
        load_seg_image();

        std::cout << "Config Viewer" << std::endl;
        config_viewer_parameter( point_viewer);
        config_viewer_parameter( mesh_viewer);

        std::cout << "Load Point Cloud Data" << std::endl;
        load_point_cloud( point_viewer );

        std::cout << "Load Mesh Data" << std::endl;
        load_mesh( mesh_viewer);

        std::cout << "Get Map Image" << std::endl;
        //get_map_image_robust();

        std::cout << "Verify Robust in mesh map" << std::endl;
        //

        std::cout << "Verify Robust in point map" << std::endl;
        //



    }

}
