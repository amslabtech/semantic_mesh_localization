#include "semantic_mesh_localization/verify_cal_likelihood.h"

int main(int argc, char** argv){

    ros::init( argc, argv, "verify_cal_likelihood");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    semlocali::VerCalLike vericallike;

    if( vericallike.setup( node, privateNode)){

        vericallike.spin();

    }

    return 0;
}
