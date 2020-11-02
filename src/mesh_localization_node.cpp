#include "semantic_mesh_localization/mesh_localization.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "mesh_localization");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    semloam::MeshLocalization meshloca(node, privateNode);

    if( meshloca.setup_mesh_localization(node, privateNode) ){

        meshloca.spin_mesh_localization();

    }

    return 0;
}
