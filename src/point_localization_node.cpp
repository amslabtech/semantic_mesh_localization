#include "semantic_mesh_localization/point_localization.h"

int main(int argc, char** argv){

    ros::init( argc, argv, "point_localization");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    semlocali::PointLocalization poiloca(node, privateNode);

    if( poiloca.setup_point_localization( node, privateNode ) ){

        poiloca.spin_point_localization();

    }

    return 0;
}
