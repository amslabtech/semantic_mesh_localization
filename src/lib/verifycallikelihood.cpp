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

        if( privateNode.getParam("ValuedMeshMapImagePath", sparam) ){
            if( sparam.length() < 1 ){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                valued_mesh_map_image_path = sparam;
            }
        }

        if( privateNode.getParam("ValuedPointMapImagePath", sparam) ){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid file path");
                return false;
            }
            else{
                valued_point_map_image_path = sparam;
            }
        }

        if( privateNode.getParam("CSVPath" , sparam) ){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid csv file path");
                return false;
            }
            else{
                likelihood_csv_file_path = sparam;
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

        std::cout << "Compara image and save verification result" << std::endl;
        std::cout << "Verify mesh image" << std::endl;
        verify_compare( seg_image, mesh_map_image, valued_mesh_map_image_path, "mesh");
        std::cout << "Verify point image" << std::endl;
        verify_compare( seg_image, point_map_image, valued_point_map_image_path, "point");

        std::cout << "Save likelihood as csv file" << std::endl;
        save_csv();

        /*
        std::cout << "Viewer" << std::endl;
        
        while(1){
            point_viewer.spin();
            mesh_viewer.spin();
        }
        */
        

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
            cv::Mat tmp_image(image_tmp.size().height, image_tmp.size().width, CV_8UC4);
            cv::cvtColor(image_tmp,tmp_image, CV_BGR2BGRA);
            seg_image.push_back( tmp_image );

            //Get image size
            image_width = tmp_image.size().width;
            image_height = tmp_image.size().height;
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
        //add_semantic_pc( viewer,     "road",         road, 120, 120, 120);
        
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
        //load_semantic_polygon( viewer,     "road", 120, 120, 120);

        
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
            //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, semantic_name);

            viewer.setRepresentationToSurfaceForAllActors();

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

    void VerCalLike::verify_compare( std::vector<cv::Mat> segmented_image, std::vector<cv::Mat> image_row, std::string image_file_path, std::string type){
        
        int compare_counter = 0;

        cmp_times = segmented_image.size() - 1;

        for( int i=0; i<cmp_times; i++){
            cv::Mat cmp_image = compare_image( segmented_image[i], image_row[ i ] , type);

            std::string number = std::to_string( compare_counter );
            std::string file_path = image_file_path + "image" + number + ".jpg";

            cv::imwrite( file_path, cmp_image );
            compare_counter += 1;

        }

    }

    cv::Mat VerCalLike::compare_image( cv::Mat segimage, cv::Mat mapimage , std::string type){

        //cv::Mat verified_image( cv::Size( image_width, image_height), CV_8UC4, cv::Scalar(0,0,0,0));

        cv::Mat verified_image( image_height, image_width, CV_8UC4, cv::Scalar(0,0,0,0) );

        std::cout << "segimage mat type" << segimage.type() << std::endl;
        std::cout << "mapimage mat type" << mapimage.type() << std::endl;

        double map_r, map_g, map_b;
        double seg_r, seg_g, seg_b;
        double diff_r, diff_g, diff_b;

        /*
        while(1){
            cv::imshow("image1" , segimage);
            cv::imshow("image2" , mapimage);
            cv::waitKey(0);
        }
        */

        /*
        std::cout << "verified_image row " << verified_image.rows << std::endl;
        std::cout << "seg_image row " << segimage.rows << std::endl;
        std::cout << "map_image row " << mapimage.rows << std::endl;
        std::cout << "Image row " << image_height << std::endl;
        */

        double likelihood = 0;
        double pixel = 0;

        for( int y=0; y<verified_image.rows; y++){

            cv::Vec4b* ver_ptr = verified_image.ptr<cv::Vec4b>( y );

            cv::Vec4b* seg_ptr = segimage.ptr<cv::Vec4b>( y );
            cv::Vec4b* map_ptr = mapimage.ptr<cv::Vec4b>( y );

            for( int x=0; x<verified_image.cols; x++){

                cv::Vec4b ver_bgr = ver_ptr[ x ];
                cv::Vec4b map_bgr = map_ptr[ x ];
                cv::Vec4b seg_bgr = seg_ptr[ x ];

                double b = ver_bgr[0];

                /*
                map_b = mapimage.at<cv::Vec4b>(y,x)[0];
                map_g = mapimage.at<cv::Vec4b>(y,x)[1];
                map_r = mapimage.at<cv::Vec4b>(y,x)[2];
                */

                map_b = map_bgr[0];
                map_g = map_bgr[1];
                map_r = map_bgr[2];

                double map_b_r = std::abs(map_b - map_r);
                double map_r_g = std::abs(map_r - map_g);
                double map_g_b = std::abs(map_g - map_b);

                /*
                seg_b = segimage.at<cv::Vec4b>(y,x)[0];
                seg_g = segimage.at<cv::Vec4b>(y,x)[1];
                seg_r = segimage.at<cv::Vec4b>(y,x)[2];
                */

                seg_b = seg_bgr[0];
                seg_g = seg_bgr[1];
                seg_r = seg_bgr[2];
                    
                    /*
                    if( seg_b==128 && seg_g==64 && seg_r==128){
                        seg_b = 120;
                        seg_g = 120;
                        seg_r = 120;
                    }*/

                double seg_b_r = std::abs(seg_b - seg_r);
                double seg_r_g = std::abs(seg_r - seg_g);
                double seg_g_b = std::abs(seg_g - seg_b);

                    
                if( map_b==0 && map_g==0 && map_r==0) continue;

                /*
                double map_bgr = map_b * 1000.0 * 1000.0 + map_g * 1000.0 + map_r;
                double seg_bgr = seg_b * 1000.0 * 1000.0 + seg_g * 1000.0 + seg_r;
                */

                    /*
                    if(type == "mesh"){
                        mesh_image_bgr.push_back(map_bgr);
                        //seg_image_bgr.push_back(seg_bgr);
                    }
                    else if(type == "point"){
                        point_image_bgr.push_back(map_bgr);
                        //seg_image_bgr.push_back(seg_bgr);
                    }
                    else{
                        seg_image_bgr.push_back(seg_bgr);
                    }
                    */

                diff_r = std::abs( seg_r - map_r );
                diff_g = std::abs( seg_g - map_g );
                diff_b = std::abs( seg_b - map_b );

                double diff_b_r = std::abs( map_b_r - seg_b_r );
                double diff_r_g = std::abs( map_r_g - seg_r_g );
                double diff_g_b = std::abs( map_g_b - seg_g_b );


                    //std::cout << diff_r << std::endl;

                    /*
                    if( diff_r < 30 && diff_g < 30 && diff_b < 30 ){
                        ver_ptr[ x ] = cv::Vec4b( 255.0, 255.0, 255.0, 0);
                    }
                    */
                double match_checker = cmp_pixel(seg_b,seg_g,seg_r,map_b,map_g,map_r);

                    /*
                    if(     (diff_b_r < 10.0 && diff_r_g < 10.0 && diff_g_b < 10.0) || 
                            ( diff_r < 30.0 && diff_g < 30.0 && diff_b < 30.0 )
                            ){
                        ver_ptr[ x ] = cv::Vec4b( map_b, map_g, map_r, 255);
                        //ver_ptr[ x ] = cv::Vec4b( 255.0, 255.0, 255.0, 0);
                        likelihood += 1.0;
                    }
                    */

                if( match_checker > 0){
                    ver_ptr[ x ] = cv::Vec4b( map_b, map_g, map_r, 0.0);
                    //ver_ptr[ x ] = cv::Vec4b( 255.0, 255.0, 255.0, 0);
                    likelihood += match_checker;
                    pixel += 1.0;
                }
            }

            /*
            std::cout << map_b << std::endl;
            std::cout << seg_b << std::endl;
            std::cout << diff_b << std::endl << std::endl;
            */
        }

        pixels.push_back( pixel );
        likelihoods.push_back( likelihood );

        return verified_image;
    }

    double VerCalLike::cmp_pixel(double seg_b,double seg_g,double seg_r,double map_b,double map_g, double map_r){
        //

        if(map_r==0 && map_g==0 && map_b==0) return 0.0;

        double diff_r = std::abs( seg_r - map_r );
        double diff_g = std::abs( seg_g - map_g );
        double diff_b = std::abs( seg_b - map_b );
        if( diff_r < 10 && diff_g < 10 && diff_b < 10 ){
            return 1.0;
        }

        if( (std::abs(seg_b - 128.0) < 5.0) && (std::abs(seg_g - 64.0) < 5.0) &&  (std::abs(seg_r - 128.0) < 5.0) ){//road
            if( std::abs(map_r - map_b) < 10.0  && map_r/(map_g+1) < 4.0 && map_b/(map_g+1) < 4.0) return 0.7;
        }

        if( (std::abs(seg_b - 232.0) < 5.0) && (std::abs(seg_g - 35.0) < 5.0) &&  (std::abs(seg_r - 244.0) < 5.0) ){//sidewalk
            if( std::abs(map_r - map_b) < 15.0  && map_r/map_g > 4.0 && map_b/map_g > 4.0) return 1.5;
        }

        if( (std::abs(seg_b - 142.0) < 5.0) && (std::abs(seg_g - 0.0) < 5.0) &&  (std::abs(seg_r - 0.0) < 5.0) ){//car
            if( map_g < 10.0 && map_r < 10.0 && map_b > 10.0) return 10.0;
        }

        if( (std::abs(seg_b - 70.0) < 5.0) && (std::abs(seg_g - 70.0) < 5.0) &&  (std::abs(seg_r - 70.0) < 5.0) ){//building
            if( std::abs(map_r - map_g) < 5.0 && std::abs(map_g - map_b) < 5.0 && std::abs(map_b - map_r) < 5.0){
                return 0.7;
            }
        }

        if( (std::abs(seg_b - 152.0) < 10.0) && (std::abs(seg_g - 251.0) < 10.0) &&  (std::abs(seg_r - 152.0) < 10.0) ){//terrain (shibahu)
            
            if( std::abs(map_r-map_b) < 5.0 && map_g>map_b && map_g>map_r ) return 2.0;

        }

        return 0.0;
    }

    void VerCalLike::save_csv(){

        std::string csv_file_name = likelihood_csv_file_path + "likelihood.csv";
        //std::ofstream csv_key( csv_file_name );

        /*
        std::string mesh_bgr_file_name = likelihood_csv_file_path + "mesh_bgr.csv";
        std::ofstream mesh_bgr_key( mesh_bgr_file_name );

        std::string point_bgr_file_name = likelihood_csv_file_path + "point_bgr.csv";
        std::ofstream point_bgr_key( point_bgr_file_name );
        */

        //csv_key << "Mesh" << "," << "Point" << std::endl;

        size_t like_size = likelihoods.size()/2;

        std::cout << std::endl;
        std::cout << "Likelihood" << std::endl;

        for( size_t i=0; i<cmp_times; i++){
            std::cout << "Image" << i << ": " << "Mesh Map: " << likelihoods[i] << " Point Map: " << likelihoods[like_size + i] << " Diff: " << std::abs(likelihoods[i]-likelihoods[i+like_size]) <<std::endl;

            //csv_key << likelihoods[i] << "," << likelihoods[i + like_size] << std::endl;
        }

        std::cout << std::endl;
        std::cout << "Pixel" << std::endl;

        for( size_t i=0; i<cmp_times; i++){
            std::cout << "Image" << i << ": " << "Mesh Map: " << pixels[i] << " Point Map: " << pixels[like_size + i] << " Diff: " << std::abs(pixels[i]-pixels[i+like_size]) <<std::endl;

            //csv_key << pixels[i] << "," << pixels[i + like_size] << std::endl;
        }

        std::cout << std::endl;


        /*
        for(size_t i=0; i<mesh_image_bgr.size(); i++){
            mesh_bgr_key << mesh_image_bgr[i] << std::endl;
        }

        for(size_t i=0; i<point_image_bgr.size(); i++){
            point_bgr_key << point_image_bgr[i] << std::endl;
        }*/

        //csv_key.close();
        /*
        mesh_bgr_key.close();
        point_bgr_key.close();
        */

    }


}


