#include "semantic_mesh_localization/verify_cal_likelihood.h"

namespace semlocali{

    VerCalLike::VerCalLike(){
        //a
    }

    VerCalLike::~VerCalLike(){}

    bool VerCalLike::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode){

        std::string sparam;

        if( privateNode.getParam("PlaceDataPath", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                place_data_path = sparam;
            }
        }

        if( privateNode.getParam("ImageDataPath", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                image_data_path = sparam;
            }
        }

        if( privateNode.getParam("MeshMapImagePath", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                mesh_map_image_path = sparam;
            }
        }

        if( privateNode.getParam("PointMapImagePath", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                point_map_image_path = sparam;
            }
        }

        if( privateNode.getParam("PCDPath", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                pcd_path = sparam;
            }
        }

        if( privateNode.getParam("PCDName", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file name");
                return false;
            }
            else{
                pcd_name = sparam;
            }
        }

        if( privateNode.getParam("PolygonDataPath", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                polygon_data_path = sparam;
            }
        }

        if( privateNode.getParam("PolygonFileNameBase", sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file name");
                return false;
            }
            else{
                polygon_file_name_base = sparam;
            }
        }

        std::cout << "Init set up has done" << std::endl;
        return true;
    }

    void VerCalLike::spin(){
        
        process();

    }

    void VerCalLike::process(){

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
        get_map_image();

        while(1){
            point_viewer.spin();
            mesh_viewer.spin();
        }

    }

    std::vector<std::string> VerCalLike::split_v( std::string& input, char delimiter){
        
        std::istringstream stream(input);
        std::string field;

        std::vector<std::string> result;

        while( getline(stream, field, delimiter) ){
            result.push_back(field);
        }

        return result;
    }

    void VerCalLike::load_place_data(){
        
        std::string csv_path = place_data_path + "odometry.csv";
        std::ifstream ifs_csv_file( csv_path );

        std::string line;
        while( getline( ifs_csv_file, line) ){

            geometry_msgs::Pose pose_tmp;

            std::vector<std::string> strvec = split_v( line, ',');

            pose_tmp.position.x = std::stod( strvec[1] );
            pose_tmp.position.y = std::stod( strvec[2] );
            pose_tmp.position.z = std::stod( strvec[3] );
            pose_tmp.orientation.x = std::stod( strvec[4] );
            pose_tmp.orientation.y = std::stod( strvec[5] );
            pose_tmp.orientation.z = std::stod( strvec[6] );
            pose_tmp.orientation.w = std::stod( strvec[7] );

            agent_place.push_back( pose_tmp );

        }

        data_length = agent_place.size();

        std::cout << "Loading place data has done" << std::endl;
    }

    void VerCalLike::load_seg_image(){

        int counter = 0;

        for( int i=0; i < data_length; i++){

            std::string pic_number = std::to_string(counter);
            std::string image_path = image_data_path + "image" + pic_number + ".jpg";
            cv::Mat image_tmp = cv::imread(image_path);
            seg_image.push_back( image_tmp );

            //Get image size
            image_width = image_tmp.size().width;
            image_height = image_tmp.size().height;
        }

        /*
        std::cout << "Width " << image_width << std::endl;
        std::cout << "height " << image_height << std::endl;
        */

        std::cout << "Loading Segmented Image has done" << std::endl;
    }

    void VerCalLike::config_viewer_parameter( pcl::visualization::PCLVisualizer& viewer){

        viewer.setSize( image_width, image_height);
        viewer.initCameraParameters();

        std::cout << "Viewer Parameter set" << std::endl;

    }

    void VerCalLike::load_point_cloud( pcl::visualization::PCLVisualizer& viewer ){

        load_PCD();

        add_semantic_pc( viewer,     "road",         road, 128,  64, 128);
        add_semantic_pc( viewer, "sidewalk",     sidewalk, 232,  35, 244);
        add_semantic_pc( viewer, "building",     building,  70,  70,  70);
        //add_semantic_pc( viewer,     "wall",    wall, 156, 102, 102);
        add_semantic_pc( viewer,    "fence",        fence, 153, 153, 190);
        //add_semantic_pc( viewer,     "pole",    pole, 153, 153, 153);
        add_semantic_pc( viewer,"trafficsign",trafficsign,   0, 220, 220);
        add_semantic_pc( viewer, "vegetation", vegetation,  35, 142, 107);
        add_semantic_pc( viewer,  "terrain",      terrain, 152, 251, 152);
        add_semantic_pc( viewer,      "car",          car, 142,   0,   0);
        add_semantic_pc( viewer,    "truck",        truck,  70,   0,   0);
        add_semantic_pc( viewer,      "bus",          bus, 100,  60,   0);
        add_semantic_pc( viewer,"motorcycle",  motorcycle, 230,   0,   0);
        add_semantic_pc( viewer,  "bicycle",      bicycle,  32,  11, 119);

        viewer.setSize( image_width, image_height);
    }

    void VerCalLike::load_PCD(){
        bool load_checker = true;

        while(load_checker==true /* && load_counter < 2 */ ){
            std::string number = std::to_string(load_pcd_counter);

            std::string pcd_file = pcd_path + pcd_name + number + ".pcd";

            int load_file = pcl::io::loadPCDFile(pcd_file, cloud_tmp);

            if(load_file == 0){//0 means loading pcd file is successfull
                load_pcd_counter += 1;

                classify_pointcloud();

                cloud_tmp.clear();

                std::cout << "Loading " << pcd_file << " is successfull" << std::endl;

            }
            else{
                cloud_tmp.clear();
                std::cout << pcd_file << " is not found" << std::endl;
                load_checker = false;
            }
        }

        std::cout << "Loading PCD file has done" << std::endl;
    }

    void VerCalLike::classify_pointcloud(){

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

    bool VerCalLike::classify(pcl::PointXYZRGB& point, const color_data& color_id){
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
    
    void VerCalLike::add_semantic_pc(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, const pcl::PointCloud<pcl::PointXYZRGB> semantic_cloud, int blue, int green, int red){
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>(semantic_cloud));

        double color_r = double(red)/255.0;
        double color_g = double(green)/255.0;
        double color_b = double(blue)/255.0;

        viewer.addPointCloud( cloudin, semantic_name);

        viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b, semantic_name);
        viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, semantic_name);
        viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, semantic_name);

        std::cout << "Setting " << semantic_name << "'s point cloud has done" << std::endl;
    }

    void VerCalLike::load_mesh( pcl::visualization::PCLVisualizer& viewer){

        load_PLY( viewer );

        viewer.setSize( image_width, image_height );

    }

    void VerCalLike::load_PLY(pcl::visualization::PCLVisualizer& viewer){

        load_semantic_polygon( viewer,     "road", 128,  64, 128);
        load_semantic_polygon( viewer, "sidewalk", 232,  35, 244);
        load_semantic_polygon( viewer, "building",  70,  70,  70);
        load_semantic_polygon( viewer,     "wall", 156, 102, 102);
        load_semantic_polygon( viewer,    "fence", 153, 153, 190);
        load_semantic_polygon( viewer,     "pole", 153, 153, 153);
        load_semantic_polygon( viewer,"trafficsign", 0, 220, 220);
        load_semantic_polygon( viewer, "vegetation",35, 142, 107);
        load_semantic_polygon( viewer,  "terrain", 152, 251, 152);
        load_semantic_polygon( viewer,      "car", 142,   0,   0);
        load_semantic_polygon( viewer,    "truck",  70,   0,   0);
        load_semantic_polygon( viewer,      "bus", 100,  60,   0);
        load_semantic_polygon( viewer,"motorcycle",230,   0,   0);
        load_semantic_polygon( viewer,  "bicycle",  32,  11, 119);

        std::cout << "Loading semantic mesh polygon is done" << std::endl;
    }

    void VerCalLike::load_semantic_polygon(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, int blue, int green, int red){

        pcl::PolygonMesh semantic_mesh;

        std::string polygon_file_string = polygon_data_path + polygon_file_name_base + semantic_name + ".ply";

        int load_ply_success_checker = pcl::io::loadPLYFile( polygon_file_string, semantic_mesh);

        if( load_ply_success_checker != 0 ){
            std::cout << "loading " << polygon_file_string << " was failed" << std::endl;
        }
        else{
            std::cout << "loading " << polygon_file_string << " was successfull" << std::endl;
            viewer.addPolygonMesh( semantic_mesh, semantic_name);

            double color_r = double(red)/255.0;
            double color_g = double(green)/255.0;
            double color_b = double(blue)/255.0;

            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b, semantic_name);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0  , semantic_name);

            std::cout << semantic_name << "'s polygon mesh is added to PCL Visualizer" << std::endl  ;
        }
    }

    void VerCalLike::get_map_image(){

        cv::Mat tmp_image;

        for(int i=0; i<agent_place.size(); i++){

            tmp_image = get_image( point_viewer, agent_place[i], i, "point_map_image");
            point_map_image.push_back(tmp_image);

            tmp_image = get_image( mesh_viewer, agent_place[i], i, "mesh_map_image");
            mesh_map_image.push_back(tmp_image);

        }

        std::cout << "Get Map Image" << std::endl;
    }

    cv::Mat VerCalLike::get_image( pcl::visualization::PCLVisualizer& viewer, geometry_msgs::Pose pose, int counter, std::string file_dir){
        
        //Set Camera position
        tf::Quaternion quat_tf;
        quaternionMsgToTF( pose.orientation , quat_tf );

        tf::Matrix3x3 rota;
        rota.setRotation( quat_tf );
        tf::Vector3 place;
        place.setValue( pose.position.x , pose.position.y , pose.position.z );

        Eigen::Matrix4f particle_pos;
        particle_pos << rota[0][0], rota[0][1] , rota[0][2], place[0],
                        rota[1][0], rota[1][1] , rota[1][2], place[1],
                        rota[2][0], rota[2][1] , rota[2][2], place[2],
                               0.0,        0.0 ,        0.0,      1.0;
        
        Eigen::Vector4f slide(0.2 , 0.0, 0.0, 1.0);
        Eigen::Vector4f new_pos = particle_pos * slide;
        viewer.setCameraPosition(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                new_pos[0],
                new_pos[1],
                new_pos[2],
                0.0,
                0.0,
                1.0);

        //Get Image as cv::Mat

        cv::Mat mapimage;

        vtkSmartPointer<vtkRenderWindow> render = viewer.getRenderWindow();
        std::unique_ptr<uchar> pixels( render->GetRGBACharPixelData( 0, 0, render->GetSize()[0]-1, render->GetSize()[1]-1, 1) );

        cv::Mat tmpimage = cv::Mat(render->GetSize()[1], render->GetSize()[0], CV_8UC4, &pixels.get()[0] );
        cv::cvtColor( tmpimage , mapimage , cv::COLOR_RGBA2BGRA);
        //Kaiten
        cv::flip(mapimage, mapimage, -1);
        cv::flip(mapimage, mapimage,  1);

        std::string number = std::to_string(counter);

        std::string file_path;
        if(file_dir=="point_map_image"){
            file_path = point_map_image_path + "image" + number + ".jpg";
            cv::imwrite(file_path, mapimage);
        }
        else if(file_dir=="mesh_map_image"){
            file_path = mesh_map_image_path + "image" + number + ".jpg";
            cv::imwrite(file_path, mapimage);
        }
        else{
            ROS_ERROR("Error in saving %s image",file_dir);
        }
       
        return mapimage;
    }

}
