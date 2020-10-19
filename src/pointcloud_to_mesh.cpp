#include"semantic_mesh_localization/pointcloud_to_mesh.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "pointcloud_to_mesh");

	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");

	semloam::PcToMesh pctomesh;

	if( pctomesh.setup(node, privateNode) ){

		pctomesh.spin();

	}

	return 0;
}
