#include "semantic_mesh_localization/mesh_localization.h"

namespace semloam{

    MeshLocalization::MeshLocalization(ros::NodeHandle& node, ros::NodeHandle& privateNode)
        : it_(node){

            image_sub = it_.subscribe
                ("/image_view/output" , 1, &MeshLocalization::segmented_image_callback, this);

        }

    MeshLocalization::~MeshLocalization(){}

    void MeshLocalization::segmented_image_callback(const sensor_msgs::ImageConstPtr& msg){
        //a
    }

    bool MeshLocalization::setup_mesh_localization(ros::NodeHandle& node, ros::NodeHandle& privateNode){

        return true;
    }

    void MeshLocalization::spin_mesh_localization(){
        //a
    }

}


