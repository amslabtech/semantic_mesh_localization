#include"semantic_mesh_localization/pointcloud_to_mesh.h"

namespace semlocali{

	PcToMesh::PcToMesh()
    {

		std::cout << "Set up PcToMesh class" << std::endl;

	}

    PcToMesh::~PcToMesh(){}

	void PcToMesh::odometry_callback(const nav_msgs::OdometryConstPtr& odom){

		odom_data = *odom;

        double qx = odom_data.pose.pose.orientation.x;
        double qy = odom_data.pose.pose.orientation.y;
        double qz = odom_data.pose.pose.orientation.z;
        double qw = odom_data.pose.pose.orientation.w;

        double ql = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);

        qx = qx/ql;
        qy = qy/ql;
        qz = qz/ql;
        qw = qw/ql;

        odom_data.pose.pose.orientation.x = qx;
        odom_data.pose.pose.orientation.y = qy;
        odom_data.pose.pose.orientation.z = qz;
        odom_data.pose.pose.orientation.w = qw;

	}

	bool PcToMesh::setup(ros::NodeHandle& node,ros::NodeHandle& privateNode){

		float fparam;
		double dparam;
		int iparam;
		bool bparam;
		std::string strparam;

		//_pubroad = node.advertise<sensor_msgs::PointCloud2>("/road",2);
		//_pubcar = node.advertise<sensor_msgs::PointCloud2>("/car",2);

		//Image Transformerをこれから使う
        //_pub_test_image = node.advertise<sensor_msgs::Image>("test_image", 1);

        /*
		_sub_odometry = node.subscribe<nav_msgs::Odometry>
			("/odom_pose", 1, &PcToMesh::odometry_callback, this);
            */

		if( privateNode.getParam("NormalSearchRadius", dparam) ){
			if(dparam < 0.0001){
				ROS_ERROR("Invalid NormalSearchRadius parameter");
				return false;
			}
			else{
				normal_search_radius = dparam;
			}
		}

        if( privateNode.getParam("polygonpathpc", strparam) ){
            if(strparam.length() < 1){
                ROS_ERROR("Invalid polygon file path in pctomesh");
                return false;
            }
            else{
                polygon_path = strparam;
            }
        }

        if( privateNode.getParam("polygonnamepc", strparam) ){
            if( strparam.length() < 1 ){
                ROS_ERROR("Invalid polygon file name");
                return false;
            }
            else{
                polygon_name = strparam;
            }
        }

		if( privateNode.getParam("filepath", strparam) ){
			if(strparam.length() < 1 ){
				ROS_ERROR("Invalid file path");
				return false;
			}
			else{
				file_path = strparam;
			}
		}

		if( privateNode.getParam("filename", strparam)){
			if(strparam.length() < 1){
				ROS_ERROR("Invalid file name");
				return false;
			}
			else{
				file_name = strparam;
			}
		}

		if( privateNode.getParam("filetype", strparam) ){
			if(strparam=="ascii" || strparam=="binary"){
				file_type = strparam;
			}
			else{
				ROS_ERROR("Invalid file type %s, You must describe file type ascii or binary in child charactor", strparam);
				return false;
			}
		}

		if( privateNode.getParam("PCLvisualizer", bparam) ){
			if( bparam==true || bparam==false ){
				pcl_mesh_visualize_checker = bparam;
			}
			else{
				ROS_ERROR("Invalid pcl_mesh_visualizer_checker parameter");
				return false;
			}
		}

		if( privateNode.getParam("ROSvisualizer", bparam) ){
			if(bparam==true || bparam==false){
				ros_mesh_visualize_checker = bparam;
			}
			else{
				ROS_ERROR("Invalid ros_mesh_visualizer_checker parameter");
				return false;
			}
		}


		std::cout << "Init set up is done" << std::endl;
        
        unlabeled.reserve(pc_size_min);
        outlier.reserve(pc_size_min);
        car.reserve(pc_size_mid);
        bicycle.reserve(pc_size_min);
        motorcycle.reserve(pc_size_min);
        onrails.reserve(pc_size_min);
        truck.reserve(pc_size_mid);
        othervehicle.reserve(pc_size_mid);
        //person.reserve(pc_size_min);
        //bicyclist.reserve(pc_size_min);
        //motorcyclist.reserve(pc_size_min);
        road.reserve(pc_size_big);
        parking.reserve(pc_size_mid);
        sidewalk.reserve(pc_size_mid);
        otherground.reserve(pc_size_mid);
        building.reserve(pc_size_big);
        fence.reserve(pc_size_big);
        otherstructure.reserve(pc_size_min);
        lanemarking.reserve(pc_size_min);
        vegetation.reserve(pc_size_mid);
        trunk.reserve(pc_size_min);
        terrain.reserve(pc_size_mid);;
        pole.reserve(pc_size_min);
        trafficsign.reserve(pc_size_min);

        std::cout << "Reserve memory" << std::endl;

		return true;
	}

	void PcToMesh::spin(){

		process();

	}

	void PcToMesh::process(){

		load_PCD();

		pcl::visualization::PCLVisualizer viewer;//create visualizer

		init_config_viewer_parameter(viewer);

		std::cout << "generate mesh" << std::endl;
		generate_mesh(viewer);

		config_tmp_viewer_parameter(viewer);
		
		ros::Rate rate(10);

		while( ros::ok() ){

			//ros::spinOnce();//catch pose data

			//change_camera_data(viewer);

            //test_publish_image_data(viewer);

			viewer.spinOnce();

			rate.sleep();

		}

	}

	void PcToMesh::publish_rosmsg(){
		sensor_msgs::PointCloud2 car_pc2, road_pc2;

		pcl::toROSMsg(car, car_pc2);
		pcl::toROSMsg(road, road_pc2);

		car_pc2.header.frame_id = "map";
		road_pc2.header.frame_id = "map";

		car_pc2.header.stamp = ros::Time::now();
		road_pc2.header.stamp = ros::Time::now();

		_pubroad.publish(road_pc2);
		_pubcar.publish(car_pc2);
	}

	void PcToMesh::load_PCD(){
		
		bool load_checker = true;

		while(load_checker==true /* && load_counter < 2 */ ){
			std::string number = std::to_string(load_counter);

			std::string file = file_path + file_name + "_" + file_type + number + ".pcd";

			int load_file = pcl::io::loadPCDFile(file, cloud_tmp);

			if(load_file == 0){//0 means loading pcd file is successfull
				load_counter += 1;

				classify_pointcloud();
				
				//cloud += cloud_tmp;
				cloud_tmp.clear();

				std::cout << "Loading " << file << " is successfull" << std::endl;

			}
			else{
				cloud_tmp.clear();
				std::cout << file << " is not found" << std::endl;

				load_checker = false;
			}
		}

		std::cout << "Loading PCD file has done" << std::endl;

	}

	void PcToMesh::classify_pointcloud(){
		size_t cloud_size = cloud_tmp.size();

		pcl::PointXYZRGB point;

		for(int i=0; i<cloud_size; i++){

			point.x = cloud_tmp[i].x;
			point.y = cloud_tmp[i].y;
			point.z = cloud_tmp[i].z;

			point.r = cloud_tmp[i].r;
			point.g = cloud_tmp[i].g;
			point.b = cloud_tmp[i].b;

			color_data color_id;
			color_id.r = int(point.r);
			color_id.g = int(point.g);
			color_id.b = int(point.b);

			if(!classify(point, color_id) ){
				//
			}
		}
	}

	bool PcToMesh::classify(pcl::PointXYZRGB& point, const color_data& color_id){
		if(color_id.r==0 && color_id.g==0 && color_id.b==0){
			//unlabeled.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
			//outlier.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==150 && color_id.b==245){
            point.b = 142;
            point.g = 0;
            point.r = 0;
			car.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==230 && color_id.b==245){
            point.b = 32;
            point.g = 11;
            point.r = 119;
			bicycle.push_back(point);
		}
		else if(color_id.r==100 && color_id.g==80 && color_id.b==250){
            point.b = 100;
            point.g = 60;
            point.r = 0;
			bus.push_back(point);
		}
		else if(color_id.r==30 && color_id.g==60 && color_id.b==150){
            point.b = 230;
            point.g = 0;
            point.r = 0;
			motorcycle.push_back(point);
		}
		else if(color_id.r==0 && color_id.g==0 && color_id.b==255){
			onrails.push_back(point);
		}
		else if(color_id.r==80 && color_id.g==30 && color_id.b==180){
            point.b = 70;
            point.g = 0;
            point.r = 0;
			truck.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==30 && color_id.b==30){
			//person.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==40 && color_id.b==200){
			//bicyclist.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==30 && color_id.b==90){
			//motorcyclist.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==255){
            point.b = 128;
            point.g = 64;
            point.r = 128;
			road.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==150 && color_id.b==255){
			parking.push_back(point);
		}
		else if(color_id.r==75 && color_id.g==0 && color_id.b==75){
            point.b = 232;
            point.g = 35;
            point.r = 244;
			sidewalk.push_back(point);
		}
		else if(color_id.r==175 && color_id.g==0 && color_id.b==75){
			otherground.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==200 && color_id.b==0){
            point.b = 70;
            point.g = 70;
            point.r = 70;
			building.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==120 && color_id.b==50){
            point.b = 153;
            point.g = 153;
            point.r = 190;
			fence.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==150 && color_id.b==0){
			otherstructure.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==255 && color_id.b==170){
			lanemarking.push_back(point);
		}
		else if(color_id.r==0 && color_id.g==175 && color_id.g==0){
            point.b = 35;
            point.g = 142;
            point.r = 107;
			vegetation.push_back(point);
		}
		else if(color_id.r==135 && color_id.g==60 && color_id.b==0){
			trunk.push_back(point);
		}
		else if(color_id.r==150 && color_id.g==240 && color_id.b==80){
            point.b = 152;
            point.g = 251;
            point.r = 152;
			terrain.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==240 && color_id.b==150){
            point.b = 153;
            point.g = 153;
            point.r = 153;
			pole.push_back(point);
		}
		else if(color_id.r==255 && color_id.g==0 && color_id.b==0){
            point.b = 0;
            point.g = 220;
            point.r = 220;
			trafficsign.push_back(point);
		}

		return true;
	}

	void PcToMesh::init_config_viewer_parameter(pcl::visualization::PCLVisualizer& viewer){

		std::cout << "Config viewer init parameter except place" << std::endl;

        viewer.setSize( 620.0, 188.0);
        viewer.initCameraParameters();

	}

	void PcToMesh::generate_mesh(pcl::visualization::PCLVisualizer& viewer){

		std::cout << "Generate Triangle Mesh" << std::endl;
		
		if(car.size() > 10){
			generate_semantic_mesh(viewer, car, "car", 0.60, 700);
		}
		
		if(bicycle.size() > 10 ){
			generate_semantic_mesh(viewer, bicycle, "bicycle", 0.15, 200);
		}
		
		if(bus.size() > 10 ){
			generate_semantic_mesh(viewer, bus, "bus", 0.2, 300);
		}

		if(motorcycle.size() > 10 ){
			generate_semantic_mesh(viewer, motorcycle, "motorcycle", 0.25, 500);
		}
		
		if(onrails.size() > 10 ){
			generate_semantic_mesh(viewer, onrails, "onrails", 0.2, 400);
		}

		if(truck.size() > 10){
			generate_semantic_mesh(viewer, truck, "truck", 0.3, 600);
		}

		if(othervehicle.size() > 10 ){
			generate_semantic_mesh(viewer, othervehicle, "othervehicle", 0.2, 400);
		}

		if(parking.size() > 10 ){
			generate_semantic_mesh(viewer, parking, "parking", 0.55, 600);
		}

		if(sidewalk.size() > 10){
			generate_semantic_mesh(viewer, sidewalk, "sidewalk", 0.90, 600);
		}

		if(otherground.size() > 10){
			generate_semantic_mesh(viewer, otherground, "otherground", 0.35, 700);
		}

		if(building.size() > 10){
			generate_semantic_mesh(viewer, building, "building", 0.90, 1000);
		}

		if(fence.size() > 10){
			generate_semantic_mesh(viewer, fence, "fence", 0.35, 900);
		}

		if(otherstructure.size() > 10 ){
			generate_semantic_mesh(viewer, otherstructure, "otherstructure", 0.35, 900);
		}

		if(lanemarking.size() > 10){
			generate_semantic_mesh(viewer, lanemarking, "lanemarking", 0.10, 200);
		}

		if(vegetation.size() > 10 ){
			generate_semantic_mesh(viewer, vegetation, "vegetation", 0.5, 600);
		}

		if(trunk.size() > 10){
			generate_semantic_mesh(viewer, trunk, "trunk", 0.4, 600);
		}

		if(terrain.size() > 10){
			generate_semantic_mesh(viewer, terrain, "terrain", 0.6, 500);
		}

		if(pole.size() > 10){
			generate_semantic_mesh(viewer, pole, "pole", 0.3, 500);
		}

		if(trafficsign.size() > 10){
			generate_semantic_mesh(viewer, trafficsign, "trafficsign", 0.10, 500);
		}

		if(road.size() > 10 ){
			generate_semantic_mesh(viewer, road, "road", 0.90, 1000);
		}

	}

	void PcToMesh::generate_semantic_mesh(pcl::visualization::PCLVisualizer& viewer,const pcl::PointCloud<pcl::PointXYZRGB>& semantic_cloud, const std::string semantic_name, double search_radius, double max_neighbor){
		
		std::cout << "Create semantic mesh of " << semantic_name << " " << std::endl;
		std::cout << "compute normal of " << semantic_name << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZRGB>(semantic_cloud));
		//create smart pointer because normal estimation and greedytriangulation need smart pointer

		pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

		kdtree->setInputCloud(cloudin);

		//Set search method
		ne.setSearchMethod(kdtree);

		//Set normal search radius
		ne.setRadiusSearch(normal_search_radius);

		//表裏を決める部分
		ne.setViewPoint(0, 0, 0);

		ne.setInputCloud(cloudin);

		ne.compute(*cloud_normal);
		
		//アホみたいな話だがPointNormalで受け取ってもXYZの情報は0のままなので写してやる必要がある
		for (int i = 0; i < cloud_normal->points.size(); i++){

			cloud_normal->points[i].x = cloudin->points[i].x;
			cloud_normal->points[i].y = cloudin->points[i].y;
			cloud_normal->points[i].z = cloudin->points[i].z;

		}

		/*
		std::cout << cloudin->points[0].x << std::endl;
		std::cout << cloudin->points[0].y << std::endl;
		std::cout << cloudin->points[0].z << std::endl;
		*/

		std::cout << "Compute polygon mesh of " << semantic_name << std::endl;

		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh triangles;

		//微調整が必要なパラメータ
		gp3.setSearchRadius(search_radius);
		gp3.setMu(5.0);
		gp3.setMaximumNearestNeighbors(max_neighbor);
		gp3.setMaximumSurfaceAngle(PI/2.0);
		gp3.setMinimumAngle(-PI);
		gp3.setMaximumAngle(PI);
		gp3.setNormalConsistency(false);

		gp3.setInputCloud(cloud_normal);
		
		//Create new KdTree for PointNormal
		pcl::search::KdTree<pcl::PointNormal>::Ptr treePN(new pcl::search::KdTree<pcl::PointNormal>);
		treePN->setInputCloud(cloud_normal);
		gp3.setSearchMethod(treePN);

		//COmpute triangles
		gp3.reconstruct(triangles);

		viewer.addPolygonMesh(triangles, semantic_name);
		
		
		//Convert RGB data 0~255 to 0.0~1.0
		float color_r = float(cloudin->points[0].r)/255.0;
		float color_g = float(cloudin->points[0].g)/255.0;
		float color_b = float(cloudin->points[0].b)/255.0;

		//viewer.addPointCloud<pcl::PointXYZRGB>(cloudin, semantic_name, v1);
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, semantic_name, v1);

		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b, semantic_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, semantic_name);

		std::cout << "Semantic mesh " << semantic_name << " was added to PCL visualizer" << std::endl;

        if(save_polygon_data == true){
            std::string polygon_file_name = polygon_path + polygon_name + semantic_name + ".ply";

            pcl::io::savePLYFile(polygon_file_name, triangles);

            std::cout << "Polygon " << polygon_file_name << "was saved as .ply file" << std::endl;


        }

	}

	void PcToMesh::config_tmp_viewer_parameter(pcl::visualization::PCLVisualizer& viewer){

		viewer.setCameraPosition(0.0,0.0,0.0, 0.0,0.0,0.0);// x, y, z, roll, pitch, yaw

	}

	void PcToMesh::change_camera_data(pcl::visualization::PCLVisualizer& viewer){

		viewer.resetCameraViewpoint();

		double roll, pitch, yaw;

		tf::Quaternion quat_tf;

		quaternionMsgToTF( odom_data.pose.pose.orientation , quat_tf );
        quat_tf.normalize();
        tf::Matrix3x3( quat_tf ).getRPY( roll, pitch, yaw);

		viewer.setCameraPosition(
				odom_data.pose.pose.position.x,
				odom_data.pose.pose.position.y,
				odom_data.pose.pose.position.z,
				roll,
				pitch,
				yaw   );


	}

    void PcToMesh::test_publish_image_data(pcl::visualization::PCLVisualizer& viewer){

        vtkSmartPointer<vtkRenderWindow> render = viewer.getRenderWindow();

        std::unique_ptr<uchar> pixels( render->GetRGBACharPixelData( 0, 0, render->GetSize()[0] - 1, render->GetSize()[1] - 1, 1 ) );

        cv::Mat cvimage = cv::Mat(render->GetSize()[1], render->GetSize()[0], CV_8UC4, &pixels.get()[0] );
        cv::cvtColor( cvimage , cvimage , cv::COLOR_RGBA2BGRA);

        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvimage).toImageMsg();

        ros_image->header.stamp = ros::Time::now();

        _pub_test_image.publish( ros_image );
		//cv::imshow("image", cvimage);

    }

}
