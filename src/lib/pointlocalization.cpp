#include "semantic_mesh_localization/point_localization.h"

namespace semlocali{

    PointLocalization::PointLocalization( ros::NodeHandle& node, ros::NodeHandle& privateNode)
        : it_(node){

            image_sub = it_.subscribe
                ("/bonnet/color_mask" , 1, &MeshLocalization::segmented_image_callback, this);
            
            sub_odometry = node.subscribe<nav_msgs::Odometry>
                ("/odom_pose", 1, &MeshLocalization::odometry_callback, this);

            pub_pose = node.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
            pub_particle = node.advertise<geometry_msgs::PoseArray>("/particle", 1);

            estimated_pose.header.frame_id = "map";
            particle.header.frame_id = "map";

            car_state.header.frame_id = "map";
            car_state.child_frame_id = "estimated_pose";

        }

    PointLocalization::~PointLocalization{}

    bool PointLocalization::setup_point_localization(ros::Nodehandle& node, ros::NodeHandle& privateNode){
        if( !setup( node, privateNode)){
            ROS_ERROR("Point Map Setup has problem");
            return false;
        }

        int iparam;
        double bparam;
        bool bparam;
        std::string sparam;

        std::cout << "Point Cloud Map's setup has done" << std::endl;






        return true;
    }
 





}
