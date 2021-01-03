#include "semantic_mesh_localization/verify_localization.h"

int main(int argc, char** argv){

    ros::init( argc, argv, "verify_localization");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    semlocali::VerLocali verlocali;

    if( verlocali.setup( node, privateNode)){

        verlocali.spin();

    }

    return 0;
}
