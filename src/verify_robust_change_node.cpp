#include"semantic_mesh_localization/verify_robust_change.h"

int main(int argc, char** argv){

    ros::init( argc, argv, "verify_robust_change");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    semlocali::VerRobust verrobust;

    if( verrobust.setup_robust( node, privateNode)){

        verrobust.spin_robust();

    }

    return 0;
}
