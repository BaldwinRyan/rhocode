#include "color_blob_detector/object_modeling_distribution.h"
//--------------------------------------------------------------------
// 1. Subscribe filtered_points
// (1). Call pointCloudCallback


namespace enc = sensor_msgs::image_encodings;
using namespace cv;

ObjectModelingDistribution::ObjectModelingDistribution(std::string node_name, ros::NodeHandle nh){
	

	nh_ = nh;
	
	//----Load default value for parameters----
	// handler_name.param<data type>("parameter name in the launch file", variable name, default value);	
	nh_.param<int>("mean_color", mean_color_, DEFAULT_MEAN_COLOR);
	nh_.param<int>("window_size", window_size_, DEFAULT_WINDOW_SIZE);
	nh_.param<int>("blur_kernel", blur_kernel_, DEFAULT_BLUR_KERNEL);
	nh_.param<int>("max_cluster_size", max_cluster_size_, DEFAULT_MAX_CLUSTER_SIZE);
	nh_.param<int>("min_cluster_size", min_cluster_size_, DEFAULT_MIN_CLUSTER_SIZE);
	nh_.param<int>("max_num_features", max_num_features_, DEFAULT_MAX_NUM_FEATURES);
	nh_.param<int>("update_rate", update_rate_, DEFAULT_UPDATE_RATE);	
	nh_.param<double>("within_cluster_distance", within_cluster_distance_, DEFAULT_WITHIN_CLUSTER_DISTANCE);
	nh_.param<double>("cluster_window", cluster_window_, DEFAULT_CLUSTER_WINDOW);
	nh_.param<double>("variance_coeff", variance_coeff_, DEFAULT_VARIANCE_COEFF); //takeshi	
	nh_.param<double>("hue_var_coeff", hue_var_coeff_, DEFAULT_HUE_VAR_COEFF); //takeshi	
	nh_.param<double>("sat_var_coeff", sat_var_coeff_, DEFAULT_SAT_VAR_COEFF); //takeshi	
	nh_.param<double>("val_var_coeff", val_var_coeff_, DEFAULT_VAL_VAR_COEFF); //takeshi	
	nh_.param<bool>("display_image", display_image_, DEFAULT_DISPLAY_IMAGE);
	nh_.param<bool>("publish_markers", publish_markers_, DEFAULT_PUBLISH_MARKERS);
	nh_.param<std::string>("feature_type", feature_type_, DEFAULT_FEATURE_TYPE);
	nh_.param<std::string>("feature_name", feature_name_, DEFAULT_FEATURE_NAME);
	nh_.param<std::string>("source_identifier", source_identifier_, DEFAULT_SOURCE_IDENTIFIER);
	nh_.param<std::string>("output_identifier", output_identifier_, DEFAULT_OUTPUT_IDENTIFIER);
	nh_.param<std::string>("marker_topic", marker_topic_, DEFAULT_MARKER_TOPIC);
	nh_.param<std::string>("base_frame", base_frame_, DEFAULT_BASE_FRAME);
	
	//----Load parameters form a launch file.----
	//----create a map of any int or double parameter values----
	parameter_map_.insert(make_pair(std::string("mean_color"), (double)mean_color_));
	parameter_map_.insert(make_pair(std::string("window_size"), (double)window_size_));
	parameter_map_.insert(make_pair(std::string("blur_kernel"), (double)blur_kernel_));
	parameter_map_.insert(make_pair(std::string("max_cluster_size"), (double)max_cluster_size_));
	parameter_map_.insert(make_pair(std::string("min_cluster_size"), (double)min_cluster_size_));
	parameter_map_.insert(make_pair(std::string("within_cluster_distance"), (double)within_cluster_distance_));
	parameter_map_.insert(make_pair(std::string("cluster_window"), (double)cluster_window_));
	parameter_map_.insert(make_pair(std::string("display_image"), (double)display_image_));
	parameter_map_.insert(make_pair(std::string("publish_markers"), (double)publish_markers_));
	parameter_map_.insert(make_pair(std::string("max_num_features"), (double)max_num_features_));
	parameter_map_.insert(make_pair(std::string("update_rate"), (double)update_rate_));
	parameter_map_.insert(make_pair(std::string("variance_coeff"), (double)variance_coeff_)); //takeshi
	parameter_map_.insert(make_pair(std::string("hue_var_coeff"), (double)hue_var_coeff_)); //takeshi
	parameter_map_.insert(make_pair(std::string("sat_var_coeff"), (double)sat_var_coeff_)); //takeshi
	parameter_map_.insert(make_pair(std::string("val_var_coeff"), (double)val_var_coeff_)); //takeshi


	//----get attributes (optional)----
	//----first get attribute names----
	XmlRpc::XmlRpcValue attribute_list;
	std::vector <std::string> att_names;
	if(nh_.getParam("attribute_names", attribute_list)){
		ROS_ASSERT(attribute_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
		  
		for (int i = 0; i < (int)attribute_list.size(); i++) 
		  {
			ROS_ASSERT(attribute_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			std::string temp_att_name = static_cast<std::string>(attribute_list[i]);
			att_names.push_back(temp_att_name);
		  }
	}


	//============next get attribute values from lauch file===============
	XmlRpc::XmlRpcValue attribute_dict;
	if(nh_.getParam("attribute_values", attribute_dict)){
		ROS_ASSERT(attribute_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
		  
		for (std::vector<std::string>::const_iterator att_name = att_names.begin(); att_name != att_names.end(); att_name++){      
			if (attribute_dict.hasMember(*att_name)){
				XmlRpc::XmlRpcValue att_value_list = attribute_dict[*att_name];
				ROS_ASSERT(att_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
				std::vector <double> att_vals;
				for (int i = 0; i < (int)att_value_list.size(); i++) 
				{
					ROS_INFO("ERROR i=%d",i);
					ROS_ASSERT(att_value_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
					std::string temp_val = static_cast<std::string>(att_value_list[i]);
					ROS_INFO("attribute name: %s, with value from param: %s", att_name->c_str(), temp_val.c_str());
					if(parameter_map_.find(temp_val) != parameter_map_.end()){
						att_vals.push_back(parameter_map_[temp_val]);
						ROS_INFO("value: %4.3f", parameter_map_[temp_val]);
					}
					else
						ROS_ERROR("NO value found for attribute %s", temp_val.c_str());
				}
				attributes_.insert(make_pair(*att_name, att_vals));
			}
			else
			ROS_ERROR("attribute_values parameter not specified correctly."); 
		}
	}


	//----initial settings----
	new_data_ = false;
	updated_  = true;

	threshold_image_.reset(new Mat); // boost::shared_ptr<cv::Mat> threshold_image_
	rgb_image_.reset(new Mat);
	cluster_image_.reset(new Mat);
	binary_image_.reset(new Mat);
	//binary_image2_.reset(new Mat);
	hue_image_.reset(new Mat);
	sat_image_.reset(new Mat); //takeshi
	val_image_.reset(new Mat); //takeshi
	pcl_in_.reset(new PointCloudRGB);
	pcl_out1_.reset(new PointCloudRGB); //takeshi

	color_indices_.reset(new pcl::PointIndices);

	within_cluster_max_ = .2;   //maximum we would ever like the clusters to be
	within_cluster_scale_ = .001;   //scale for cluster distance ( 1mm )
	within_cluster_scaled_ = (int)(within_cluster_distance_ / within_cluster_scale_);

	node_name_ = node_name;
	window_name_ = node_name_ + " Window";
	//------- pop up a window to control values -------
	//if (display_image_) {
		//namedWindow(window_name_, CV_WINDOW_NORMAL || CV_WINDOW_KEEPRATIO);
		////namedWindow(window_name_, CV_WINDOW_AUTOSIZE);
		//createTrackbar("Mean Color", window_name_, &mean_color_, 179, NULL);
		//createTrackbar("Blur Kernel", window_name_, &blur_kernel_, 25, NULL);
		//createTrackbar("Window Size", window_name_, &window_size_, 40, NULL);
		//createTrackbar("Cluster Distance", window_name_, &within_cluster_scaled_, (int)(within_cluster_max_ / within_cluster_scale_), NULL, this); 

		////necessary to have time to initialize display window correctly
		//waitKey(50); //wait 50 milseconds
	//}

	//----- Subscriber -----
	pointcloud_sub_ = nh_.subscribe(source_identifier_, 1, &ObjectModelingDistribution::pointCloudCallback, this); //subscribe filtered_points"
	//----- Publisher -----
	blob_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_,1);
	moments_pub_ = nh_.advertise<action_msgs::GaussianPointDistributionArray>(output_identifier_,1);
	new_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("object_marker", 10);
	new_marker2_pub_ = nh_.advertise<visualization_msgs::Marker>("object2_marker", 10);
	
	new_filtered_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("new_filtered_points", 1);

	//----- Timer ----- call "update" periodically
	timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &ObjectModelingDistribution::update, this);
	
  
	//---------------------Takeshi added ---------------------------------
	//Initialize variables
	log_count_=0;//Takeshi 
	for (int i=0;i<3;i++){
		glob_im_mean_[i]=0.0;
		glob_obj_mean_[i]=0.0;
		for (int j=0;j<3;j++){		
			glob_im_cov_[i][j]=0.0;
			glob_obj_cov_[i][j]=0.0;
		}
		for (int k=0;k<256;k++){		
			glob_im_hist_[i][k]=0;
		}
	}  
	for (int i=0;i<3;i++){	//3--- 1-norm, 2-norm and frob-norm
		for (int k=0;k<100;k++){		
			glob_norm_hist_[i][k]=0.0;		
		}
	}
	
	//--- load objectsmodel--- TO DO
	//loadObjectsModel();
	
	
	//mean_color_sub_ = nh_.subscribe("/uBot/hue_mean_color_", 1, &ObjectModelingDistribution::getMeanColorCallback, this);
	//no_blob_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/camera/rgb/feature/blue/no_marker",1);
	// -------------------------------------------------------------------

	//register the goal and feedback callbacks
	ROS_INFO("Hue Monitor %s started", node_name_.c_str());
}
////------------- Takeshi added----------------------
//void ObjectModelingDistribution::getMeanColorCallback(const affordance_msgs::trackConstPtr& msg)
//{
	//mean_color_=(int)(msg->mean[0]);
	//ROS_INFO("====== Subscribe mean color===============");
	//ROS_INFO("=========Subscribe mean_color = %d=========",mean_color_);
	////sleep(0.5);
//}
//// -------------------------------------------------------------------

void ObjectModelingDistribution::loadObjectsModel(){
	std::ifstream ifs("/home/takeshi/UMassROS/umass_servers/ImagePipeline/color_blob_detector/model_data.txt");
	std::string buf;
	getline(ifs,floor_model_.sensor_);
	//unsigned int loc = floor_model_.sensor_.find("sensor",0);
	//std::cout << "the loc is " << loc << std::endl;
	//unsigned int loc = floor_model_.sensor_.find(":",0);
	boost::erase_all(floor_model_.sensor_," ");
	boost::erase_all(floor_model_.sensor_,"/t");
	std::cout << floor_model_.sensor_<< std::endl;

	std::vector<std::string> strs;
	boost::split(strs,floor_model_.sensor_, boost::is_any_of(":"));
	std::cout << "str1" << strs[0] << std::endl;
	std::cout << "str2" << strs[1] << std::endl;
	//boost::iterator_range<char*> result = boost::algorithm::find_first(floor_model_.sensor_,"hue");
	//std::cout << "the loc is " << result << std::endl;
	
	std::cout << floor_model_.sensor_ << std::endl;
	getline(ifs,floor_model_.objective_);
	std::cout << floor_model_.objective_ << std::endl;
	getline(ifs,floor_model_.effector_);
	std::cout << floor_model_.effector_ << std::endl;
	getline(ifs,floor_model_.frame_);
	std::cout << floor_model_.frame_ << std::endl;
	getline(ifs, buf);
	
	floor_model_.eigval1_mean_=std::strtod(buf.c_str(),0); //string to double //boost::lexical_cast<double>(input) ???
	std::cout << floor_model_.eigval1_mean_ << std::endl;

	
	floor_model_.eigval1_mean_ = 2.0;
	floor_model_.signal_mean_ = Eigen::VectorXf::Ones(5);
	floor_model_.signal_mean_ = Eigen::VectorXf::Ones(2);

	std::cout << "test =" << floor_model_.signal_mean_ << std::endl;
}


void ObjectModelingDistribution::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if(!new_data_ && updated_){    

    new_data_ = true;

    sensor_msgs::Image img;
    PointCloudRGB cloud_in;
    boost::scoped_ptr<Mat> hsv_image;	     // hsv image from cameras
    hsv_image.reset(new Mat);
    
    pcl::fromROSMsg (*msg, *pcl_in_);
    waitKey(5);//required wait period to convert to pcl
    pcl::toROSMsg (*pcl_in_, img);

	//----cv_bridge is in ROS. cv_bridge is a bridge between opencv and ROC
	//----toCvCopy convert a seonsor_msgs::Image message to an OpenCV-compatible CVimage, copying the image data.
    rgb_image_ptr_ = cv_bridge::toCvCopy(img, "bgr8");  								
    rgb_image_->release();
    rgb_image_ptr_->image.copyTo(*rgb_image_);

	//----convert rgb_image in BGR space to hsv_image in HSV space ----
    cvtColor(*rgb_image_, *hsv_image, CV_BGR2HSV); 
    
    //----split the image into separate color planes-----
    vector <Mat> planes;
    split(*hsv_image, planes); //hsv_image in HSV space will be divided into each plane?
    
    //---Initialize planes---
    *hue_image_ = Mat::zeros(hue_image_->size(), CV_8UC1); //create a matrix with type CV_8UC1. CV_8UC1 means 80bit single-channel matrix
    planes[0].copyTo(*hue_image_); //plane[0] has hue data, and the data will be copied to hue_image_
    *sat_image_ = Mat::zeros(sat_image_->size(), CV_8UC1); //create a matrix with type CV_8UC1. CV_8UC1 means 80bit single-channel matrix
    planes[1].copyTo(*sat_image_); //plane[0] has hue data, and the data will be copied to hue_image_    
    *val_image_ = Mat::zeros(val_image_->size(), CV_8UC1); //create a matrix with type CV_8UC1. CV_8UC1 means 80bit single-channel matrix
    planes[2].copyTo(*val_image_); //plane[0] has hue data, and the data will be copied to hue_image_

    //============= compute mean and variace of the model==============
	int num_cols[3], num_rows[3], im_size[3], num_elements;
    double im_mean[3], im_variance[3], im_sum[3];
   	double cov_sum[3][3], im_cov[3][3];
   	int im_hist[3][256];
    int is_computing_floor=false;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    //---- get the number of columns and rows, and the size of the data. ---
    for(int i=0;i<3;i++){
		num_cols[i]=planes[i].cols;
		num_rows[i]=planes[i].rows;
		im_size[i]=num_cols[i]*num_rows[i];
		im_sum[i]=0;
		for(int j=0;j<256;j++){
			im_hist[i][j]=0;
		}		
	}	
	
	//------ Compute mean and histogram -----------------
	if(is_computing_floor==true){
		num_elements=0;	
		for(int i=0;i<im_size[0];i++){	
			if (planes[2].data[i]>0){ // if the intensity is zero, this means that filtered point doesn't have any information about that point.
				num_elements = num_elements+ 1;			
				for (int k=0;k<3;k++){
					im_sum[k] = im_sum[k] + planes[k].data[i]; // for mean				
					im_hist[k][planes[k].data[i]] = im_hist[k][planes[k].data[i]]+1; //for histogram								
				}
			}
		}	
		if(num_elements > 0){
			for (int k=0;k<3;k++){
				im_mean[k] = im_sum[k]/num_elements;			
			}
		}else{
			for (int k=0;k<3;k++){
				im_mean[k] = 0;
			}
		}

		//---------------compute covariance----------------
		for (int i=0;i<3;i++){
			for (int j=0;j<3;j++){
				cov_sum[i][j]=0.0;
			}
		}	
		for(int i=0;i<im_size[0];i++){	
			//im_sum=im_sum+pow((planes[hsv_j].data[i]-im_mean[hsv_j]),2);		
			if (planes[2].data[i]>0){		
				cov_sum[0][0]=cov_sum[0][0] + (planes[0].data[i]-im_mean[0])*(planes[0].data[i]-im_mean[0]);
				cov_sum[1][0]=cov_sum[1][0] + (planes[1].data[i]-im_mean[1])*(planes[0].data[i]-im_mean[0]);
				cov_sum[2][0]=cov_sum[2][0] + (planes[2].data[i]-im_mean[2])*(planes[0].data[i]-im_mean[0]);
				
				cov_sum[0][1]=cov_sum[0][1] + (planes[0].data[i]-im_mean[0])*(planes[1].data[i]-im_mean[1]);
				cov_sum[1][1]=cov_sum[1][1] + (planes[1].data[i]-im_mean[1])*(planes[1].data[i]-im_mean[1]);			
				cov_sum[2][1]=cov_sum[2][1] + (planes[2].data[i]-im_mean[2])*(planes[1].data[i]-im_mean[1]);
				
				cov_sum[0][2]=cov_sum[0][2] + (planes[0].data[i]-im_mean[0])*(planes[2].data[i]-im_mean[2]);
				cov_sum[1][2]=cov_sum[1][2] + (planes[1].data[i]-im_mean[1])*(planes[2].data[i]-im_mean[2]);			
				cov_sum[2][2]=cov_sum[2][2] + (planes[2].data[i]-im_mean[2])*(planes[2].data[i]-im_mean[2]);
			}
		}	
		for (int i=0;i<3;i++){
			for (int j=0;j<3;j++){
				if(num_elements>0){
					im_cov[i][j]=cov_sum[i][j]/num_elements;
				}else{
					im_cov[i][j]=0.0;
				}
			}
		}
		//---- Compute the mean of the mean and covariance ----
		for (int i=0;i<3;i++){
			glob_im_mean_[i]=glob_im_mean_[i]+im_mean[i];
			for (int j=0;j<3;j++){
				glob_im_cov_[i][j]=glob_im_cov_[i][j]+im_cov[i][j];
			}
			for (int k=0;k<256;k++){
				glob_im_hist_[i][k]=glob_im_hist_[i][k]+im_hist[i][k];
			}
		}
		log_count_=log_count_+1;	
		if(log_count_==100){
			for (int i=0;i<3;i++){
				glob_im_mean_[i]=glob_im_mean_[i]/log_count_;
				for (int j=0;j<3;j++){
					glob_im_cov_[i][j]=glob_im_cov_[i][j]/log_count_;
				}
				for(int k=0;k<256;k++){
					glob_im_hist_[i][k]=glob_im_hist_[i][k];
				}
			}					
			printf("=================================================\n");
			printf("-------------count = %d ----------\n",log_count_);
			printf("hue: mean=%6.4f, variance=%6.4f\n",glob_im_mean_[0],glob_im_cov_[0][0]);
			printf("sat: mean=%6.4f, variance=%6.4f\n",glob_im_mean_[1],glob_im_cov_[1][1]);
			printf("val: mean=%6.4f, variance=%6.4f\n",glob_im_mean_[2],glob_im_cov_[2][2]);
			
			printf("cov: %6.4f %6.4f %6.4f\n",glob_im_cov_[0][0],glob_im_cov_[0][1],glob_im_cov_[0][2]);
			printf("   : %6.4f %6.4f %6.4f\n",glob_im_cov_[1][0],glob_im_cov_[1][1],glob_im_cov_[1][2]);
			printf("   : %6.4f %6.4f %6.4f\n",glob_im_cov_[2][0],glob_im_cov_[2][1],glob_im_cov_[2][2]);
			printf("=================================================\n");
			printf("========== hue hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					printf("%d ",glob_im_hist_[0][8*l+n]);
				}
				printf("\n");
			}
			printf("========== sat hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					printf("%d ",glob_im_hist_[1][8*l+n]);
				}
				printf("\n");
			}
			printf("========== val hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					printf("%d ",glob_im_hist_[2][8*l+n]);
				}
				printf("\n");
			}		
		}else if(log_count_<100){
			printf("-------------log_count=%d----------\n",log_count_);
			printf("hue: mean=%6.4f, variance=%6.4f\n",im_mean[0],im_cov[0][0]);
			printf("sat: mean=%6.4f, variance=%6.4f\n",im_mean[1],im_cov[1][1]);
			printf("val: mean=%6.4f, variance=%6.4f\n",im_mean[2],im_cov[2][2]);
			
			printf("cov: %6.4f %6.4f %6.4f\n",im_cov[0][0],im_cov[0][1],im_cov[0][2]);
			printf("   : %6.4f %6.4f %6.4f\n",im_cov[1][0],im_cov[1][1],im_cov[1][2]);
			printf("   : %6.4f %6.4f %6.4f\n",im_cov[2][0],im_cov[2][1],im_cov[2][2]);
		}
	}

	//===============segment out the floor =============================
	//--------floor model ----------
	double floor_mean[3] = {59.2996,6.3939,226.6608};
	double floor_var[3]  = {2129.8503,57.6890,256.8162};
	double floor_hue_kmeans[2]={30.4602, 113.3986};
	double floor_hue_kmeans_var[2]={210.5612, 357.8212};	
	double floor_hue_boundary=(floor_hue_kmeans[1]-floor_hue_kmeans[0])/2.0+floor_hue_kmeans[0];
	
	double o_mean[2][3] = {{48.9113,146.3329,88.4433},{166.8488, 152.6717,177.9121}};
	double o_var[2][3] = {{57.3365,2513.3059,172.5225},{273.4618, 283.4092, 451.0874}};
	
	double floor_std[3];
	int is_floor;
	int object_class;
	double obj_mean[3], obj_variance[3];
	double obj_sum[3] = {0.0,0.0,0.0};
	double obj_cov_sum[3][3];
	double obj_cov[3][3];
	//double sig_coeff=variance_coeff_;
//hue_var_coeff_
	pcl_out1_.reset(new PointCloudRGB); //takeshi
	pcl_out1_->header=pcl_in_->header;
	if(is_computing_floor==false){		
		floor_std[0]=sqrt(floor_var[0]);
		floor_std[1]=sqrt(floor_var[1]);
		floor_std[2]=sqrt(floor_var[2]);
		//------------------------------
		
		//Prepare markers
		visualization_msgs::Marker points;
		points.header.frame_id = base_frame_;
		points.header.stamp =ros::Time::now();
		points.ns =  "points";
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		points.scale.x = 0.003; 		//// POINTS markers use x and y scale for width/height respectively
		points.scale.y = 0.003;
		points.color.g = 1.0f; 		// Points are green
		points.color.a = 1.0;
		geometry_msgs::Point p;
		//Prepare markers 2
		visualization_msgs::Marker points2;
		points2.header.frame_id = base_frame_;
		points2.header.stamp =ros::Time::now();
		points2.ns =  "points";
		points2.id = 0;
		points2.type = visualization_msgs::Marker::POINTS;
		points2.scale.x = 0.003; 		//// POINTS markers use x and y scale for width/height respectively
		points2.scale.y = 0.003;
		points2.color.r = 1.0f; 		// Points are red
		points2.color.a = 1.0;
		geometry_msgs::Point p2;


		num_elements=0;
		for(int i=0;i<im_size[0];i++){	
			if (planes[2].data[i]>0){ // if the intensity is zero, this means that filtered point doesn't have any information about that point.
				is_floor=false;
				//if(((floor_mean[0]-hue_var_coeff_*floor_std[0])<planes[0].data[i])&&((floor_mean[0]+hue_var_coeff_*floor_std[0])>planes[0].data[i])){
					//if(((floor_mean[1]-sat_var_coeff_*floor_std[1])<planes[1].data[i])&&((floor_mean[1]+sat_var_coeff_*floor_std[1])>planes[1].data[i])){
						//if(((floor_mean[2]-val_var_coeff_*floor_std[1])<planes[2].data[i])&&((floor_mean[2]+val_var_coeff_*floor_std[2])>planes[2].data[i])){
							//is_floor=true;
						//}
					//}
				//}
				double x0=planes[0].data[i];
				double x1=planes[1].data[i];
				double x2=planes[2].data[i];
				//double multihypo[3];
				Eigen::Vector3f multihypo;
				Eigen::MatrixXf::Index maxind;
								
				if (x0<=floor_hue_boundary){										
					//p1=normdist(x,floor_hue_kmeans[0],floor_hue_kmeans_var[0]);
					//p2=normdist(x,floor_mean[1],floor_var[1]);
					//p3=normdist(x,floor_mean[2],floor_var[2]);
					//multihypo(0)=p1*p2*p3;
					//std::cout << "floor0 :x= " << x << "p1=" << p1 <<", p2=" << p2 <<", p3=" << p3 << std::endl;
					multihypo(0)=normdist(x0,floor_hue_kmeans[0],floor_hue_kmeans_var[0])*normdist(x1,floor_mean[1],floor_var[1])*normdist(x2,floor_mean[2],floor_var[2]);
				}else{
					//p1=normdist(x,floor_hue_kmeans[1],floor_hue_kmeans_var[1]);
					//p2=normdist(x,floor_mean[1],floor_var[1]);
					//p3=normdist(x,floor_mean[2],floor_var[2]);
					//multihypo(0)=p1*p2*p3;
					//std::cout << "floor1 :x= " << x << "p1=" << p1 <<", p2=" << p2 <<", p3=" << p3 << std::endl;
					multihypo(0)=normdist(x0,floor_hue_kmeans[1],floor_hue_kmeans_var[1])*normdist(x1,floor_mean[1],floor_var[1])*normdist(x2,floor_mean[2],floor_var[2]);					
				}
				//multihypo(0)=normdist(x,floor_mean[0],floor_var[0])*normdist(x,floor_mean[1],floor_var[1])*normdist(x,floor_mean[2],floor_var[2]);				
				multihypo(1)=normdist(x0,o_mean[0][0],o_var[0][0])*normdist(x1,o_mean[0][1],o_var[0][1])*normdist(x2,o_mean[0][2],o_var[0][2]);
				multihypo(2)=normdist(x0,o_mean[1][0],o_var[1][0])*normdist(x1,o_mean[1][1],o_var[1][1])*normdist(x2,o_mean[1][2],o_var[1][2]);				

				multihypo.maxCoeff(&maxind);				
				//std::cout << "floor_hue_boundary=" << floor_hue_boundary << "index= " << maxind <<",  hypo =" << multihypo(0) <<  ", " <<multihypo(1) <<  ", " <<multihypo(2) << std::endl;
				if (maxind==0){//  ((multihypo(0)>multihypo(1))&&((multihypo(0)>multihypo(2)))){
					is_floor=true;					
					
				}else{
					//std::cout << "hypo =" << multihypo << std::endl;
					object_class=maxind;
				}
				
				if (is_floor==false){
					if (object_class==1){ // green ball
						num_elements = num_elements+ 1;								
						//---marker---
						points.action = visualization_msgs::Marker::ADD;
						points.pose.orientation.w = 1.0;
						p.x=pcl_in_->points[i].x;
						p.y=pcl_in_->points[i].y;
						p.z=pcl_in_->points[i].z;
						points.points.push_back(p);
						
						//---pointcloud indices--
						inliers->indices.push_back(i);
						pcl_out1_->points.push_back(pcl_in_->points[i]);
					}else if (object_class==2){ //pink ball
						//---marker---
						points2.action = visualization_msgs::Marker::ADD;
						points2.pose.orientation.w = 1.0;
						p2.x=pcl_in_->points[i].x;
						p2.y=pcl_in_->points[i].y;
						p2.z=pcl_in_->points[i].z;
						points2.points.push_back(p2);
						
						//---pointcloud indices--
						inliers->indices.push_back(i);
						pcl_out1_->points.push_back(pcl_in_->points[i]);

					}
					
					//---histogram---
					for (int k=0;k<3;k++){								
						obj_sum[k] = obj_sum[k] + planes[k].data[i];				
						im_hist[k][planes[k].data[i]] = im_hist[k][planes[k].data[i]]+1; //for histogram				
					}
				}
				//if (is_floor==false){
					//num_elements = num_elements+ 1;								
					////---marker---
					//points.action = visualization_msgs::Marker::ADD;
					//points.pose.orientation.w = 1.0;
					//p.x=pcl_in_->points[i].x;
					//p.y=pcl_in_->points[i].y;
					//p.z=pcl_in_->points[i].z;
					//points.points.push_back(p);
					
					////---pointcloud indices--
					//inliers->indices.push_back(i);
					//pcl_out1_->points.push_back(pcl_in_->points[i]);
					
					////---histogram---
					//for (int k=0;k<3;k++){								
						//obj_sum[k] = obj_sum[k] + planes[k].data[i];				
						//im_hist[k][planes[k].data[i]] = im_hist[k][planes[k].data[i]]+1; //for histogram				
					//}
				//}
			}
		}
		//publish marker
		new_marker_pub_.publish(points);
		points.points.clear();
		new_marker2_pub_.publish(points2);
		points2.points.clear();		
		//publish point cloud
		sensor_msgs::PointCloud2 filtered_msg;      
		pcl::toROSMsg(*pcl_out1_, filtered_msg);
		filtered_msg.header.stamp = ros::Time::now();
		filtered_msg.header.frame_id = base_frame_;
		new_filtered_points_pub_.publish(filtered_msg);


		if(num_elements>0){
			obj_mean[0]=obj_sum[0]/num_elements;
			obj_mean[1]=obj_sum[1]/num_elements;
			obj_mean[2]=obj_sum[2]/num_elements;
		}else{
			obj_mean[0]=0;
			obj_mean[1]=0;
			obj_mean[2]=0;
		}

		//compute covariance
		for (int i=0;i<3;i++){
			for (int j=0;j<3;j++){
				obj_cov_sum[i][j]=0.0;
			}
		}	
		for(int i=0;i<im_size[0];i++){	
			//im_sum=im_sum+pow((planes[hsv_j].data[i]-im_mean[hsv_j]),2);		
			if (planes[2].data[i]>0){
				is_floor=false;
				//if(((floor_mean[0]-hue_var_coeff_*floor_std[0])<planes[0].data[i])&&((floor_mean[0]+hue_var_coeff_*floor_std[0])>planes[0].data[i])){
					//if(((floor_mean[1]-sat_var_coeff_*floor_std[1])<planes[1].data[i])&&((floor_mean[1]+sat_var_coeff_*floor_std[1])>planes[1].data[i])){
						//if(((floor_mean[2]-val_var_coeff_*floor_std[1])<planes[2].data[i])&&((floor_mean[2]+val_var_coeff_*floor_std[2])>planes[2].data[i])){
							//is_floor=true;
						//}
					//}
				//}
				double x=planes[2].data[i];
				//double multihypo[3];
				Eigen::Vector3f multihypo;
				Eigen::MatrixXf::Index maxind;
				
				if (x<=floor_hue_boundary){
					multihypo(0)=normdist(x,floor_hue_kmeans[0],floor_hue_kmeans_var[0])*normdist(x,floor_mean[1],floor_var[1])*normdist(x,floor_mean[2],floor_var[2]);
				}else{
					multihypo(0)=normdist(x,floor_hue_kmeans[1],floor_hue_kmeans_var[1])*normdist(x,floor_mean[1],floor_var[1])*normdist(x,floor_mean[2],floor_var[2]);
				}

				//multihypo(0)=normdist(x,floor_mean[0],floor_var[0])*normdist(x,floor_mean[1],floor_var[1])*normdist(x,floor_mean[2],floor_var[2]);
				multihypo(1)=normdist(x,o_mean[0][0],o_var[0][0])*normdist(x,o_mean[0][1],o_var[0][1])*normdist(x,o_mean[0][2],o_var[0][2]);
				multihypo(2)=normdist(x,o_mean[1][0],o_var[1][0])*normdist(x,o_mean[1][1],o_var[1][1])*normdist(x,o_mean[1][2],o_var[1][2]);				
				multihypo.maxCoeff(&maxind);
				if (maxind==0){
					is_floor=true;
				}
				
				if (is_floor==false){
		
					obj_cov_sum[0][0]=obj_cov_sum[0][0] + (planes[0].data[i]-obj_mean[0])*(planes[0].data[i]-obj_mean[0]);
					obj_cov_sum[1][0]=obj_cov_sum[1][0] + (planes[1].data[i]-obj_mean[1])*(planes[0].data[i]-obj_mean[0]);
					obj_cov_sum[2][0]=obj_cov_sum[2][0] + (planes[2].data[i]-obj_mean[2])*(planes[0].data[i]-obj_mean[0]);
					
					obj_cov_sum[0][1]=obj_cov_sum[0][1] + (planes[0].data[i]-obj_mean[0])*(planes[1].data[i]-obj_mean[1]);
					obj_cov_sum[1][1]=obj_cov_sum[1][1] + (planes[1].data[i]-obj_mean[1])*(planes[1].data[i]-obj_mean[1]);			
					obj_cov_sum[2][1]=obj_cov_sum[2][1] + (planes[2].data[i]-obj_mean[2])*(planes[1].data[i]-obj_mean[1]);
					
					obj_cov_sum[0][2]=obj_cov_sum[0][2] + (planes[0].data[i]-obj_mean[0])*(planes[2].data[i]-obj_mean[2]);
					obj_cov_sum[1][2]=obj_cov_sum[1][2] + (planes[1].data[i]-obj_mean[1])*(planes[2].data[i]-obj_mean[2]);			
					obj_cov_sum[2][2]=obj_cov_sum[2][2] + (planes[2].data[i]-obj_mean[2])*(planes[2].data[i]-obj_mean[2]);
				}
			}
		}
		
		for (int i=0;i<3;i++){
			for (int j=0;j<3;j++){
				if(num_elements>0){
					obj_cov[i][j]=obj_cov_sum[i][j]/num_elements;
				}else{
					obj_cov[i][j]=0.0;
				}
			}
		}

		for (int i=0;i<3;i++){
			glob_obj_mean_[i]=glob_obj_mean_[i]+obj_mean[i];
			for (int j=0;j<3;j++){
				glob_obj_cov_[i][j]=glob_obj_cov_[i][j]+obj_cov[i][j];
			}
			for (int k=0;k<256;k++){
				glob_im_hist_[i][k]=glob_im_hist_[i][k]+im_hist[i][k];
			}

		}
		


		log_count_=log_count_+1;	
		if(log_count_==100){
			for (int i=0;i<3;i++){
				glob_obj_mean_[i]=glob_obj_mean_[i]/log_count_;
				for (int j=0;j<3;j++){
					glob_obj_cov_[i][j]=glob_obj_cov_[i][j]/log_count_;
				}
			}
			//--------write data --------------------
			FILE *fp_hue_hist, *fp_sat_hist, *fp_val_hist;
			fp_hue_hist=fopen("/home/takeshi/UMass/ROS/data/vision/hue_hist.txt","w");
			fp_sat_hist=fopen("/home/takeshi/UMass/ROS/data/vision/sat_hist.txt","w");
			fp_val_hist=fopen("/home/takeshi/UMass/ROS/data/vision/val_hist.txt","w");			
			if (fp_hue_hist == NULL) {
				std::cerr<<"Can't open input file"<<std::endl;
				return;
			}
			if (fp_sat_hist == NULL) {
				std::cerr<<"Can't open input file"<<std::endl;						
				return;				
			}
			if (fp_val_hist == NULL) {				
				std::cerr<<"Can't open input file"<<std::endl;						
				return;				
			}
								
			printf("=================================================\n");
			printf("-------------count = %d ----------\n",log_count_);
			printf("hue: mean=%6.4f, variance=%6.4f\n",glob_obj_mean_[0],glob_obj_cov_[0][0]);
			printf("sat: mean=%6.4f, variance=%6.4f\n",glob_obj_mean_[1],glob_obj_cov_[1][1]);
			printf("val: mean=%6.4f, variance=%6.4f\n",glob_obj_mean_[2],glob_obj_cov_[2][2]);
			
			printf("cov: %6.4f %6.4f %6.4f\n",glob_obj_cov_[0][0],glob_obj_cov_[0][1],glob_obj_cov_[0][2]);
			printf("   : %6.4f %6.4f %6.4f\n",glob_obj_cov_[1][0],glob_obj_cov_[1][1],glob_obj_cov_[1][2]);
			printf("   : %6.4f %6.4f %6.4f\n",glob_obj_cov_[2][0],glob_obj_cov_[2][1],glob_obj_cov_[2][2]);
			
			printf("========== hue hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					printf("%d ",glob_im_hist_[0][8*l+n]);
					fprintf(fp_hue_hist, "%d ",glob_im_hist_[0][8*l+n]);
				}
				printf("\n");
				
			}
			fprintf(fp_hue_hist, "\n");
			printf("========== sat hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					printf("%d ",glob_im_hist_[1][8*l+n]);
					fprintf(fp_sat_hist, "%d ",glob_im_hist_[1][8*l+n]);
					
				}
				printf("\n");
			}
			printf("========== val hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					printf("%d ",glob_im_hist_[2][8*l+n]);
					fprintf(fp_val_hist, "%d ",glob_im_hist_[2][8*l+n]);					
				}
				printf("\n");
			}
			printf("=================================================\n");
			fclose(fp_hue_hist);
			fclose(fp_sat_hist);
			fclose(fp_val_hist);
		}else if(log_count_<100){
				printf("-------------log_count=%d----------\n",log_count_);
			printf("hue: mean=%6.4f, variance=%6.4f\n",obj_mean[0],obj_cov[0][0]);
			printf("sat: mean=%6.4f, variance=%6.4f\n",obj_mean[1],obj_cov[1][1]);
			printf("val: mean=%6.4f, variance=%6.4f\n",obj_mean[2],obj_cov[2][2]);
			
			printf("cov: %6.4f %6.4f %6.4f\n",obj_cov[0][0],obj_cov[0][1],obj_cov[0][2]);
			printf("   : %6.4f %6.4f %6.4f\n",obj_cov[1][0],obj_cov[1][1],obj_cov[1][2]);
			printf("   : %6.4f %6.4f %6.4f\n",obj_cov[2][0],obj_cov[2][1],obj_cov[2][2]);
		}
		//FILE *fp_mean, *fp_cov;
		//fp_mean=fopen("/home/takeshi/UMass/ROS/data/vision/mean.txt","w");

		//if (fp_mean == NULL) {
			//std::cerr<<"Can't open input file for rehab l"<<std::endl;
			//if (fp_mean == NULL) {
				//std::cerr<<"Can't open input file"<<std::endl;						
				//return;
			//}     
		//}
		
		//fprintf(fp_mean, "%d,%s\n",current_file_num, filename_used);
		
		//fp_cov=fopen("/home/takeshi/UMass/ROS/data/vision/cov.txt","w");
		//if (fp_cov == NULL) {
			//std::cerr<<"Can't open input file for rehab l"<<std::endl;
		
			//if (fp_cov == NULL) {
				//std::cerr<<"Can't open input file"<<std::endl;						
				//return;
			//}     
		//}
		//fprintf(fp_cov, "%d,%s\n",current_file_num, filename_used);
	}

	
    updated_ = false;
    
    //----new---
    //updated_ = true;
    //new_data_ = false; // added June 26 takeshi

  }
}

//void ObjectModelingDistribution::getPointIndices()
//{
	//Size s = threshold_image_->size();

	////take the points that are non-zero in threshold_image_ and add them as point_indices
	//color_indices_->indices.clear();

	//for(int i = 0; i < s.height; i++) {
		//for(int j = 0; j < s.width; j++) {
		  ////std::cout << " i: " << i << " j: " << j << std::endl;
		  ////std::cout << "point cloud val: " << pcl_in_->at(j,i) << std::endl;
		  ////std::cout << "image val: " << (int)(threshold_image_->at <uchar> (i, j))  << std::endl;
			//if((int)(binary_image_->at <uchar> (i, j)))
			//{ 
				//color_indices_->indices.push_back(i * s.width + j);
			//}
		//}
	//}
//}

double ObjectModelingDistribution::normdist(double x, double mu, double sigma2){
	double sigma=sqrt(sigma2);
	//double y = 1.0/sqrt(2.0*M_PI*pow(sigma,2))*exp(-pow((x-mu),2)/(2.0*pow(sigma,2)));

	//standerdized
	double z = (x - mu)/sigma;
	sigma=1.0;
	mu=0;
	double y = exp(-pow(z,2)/2.0);
	//double y = 1.0/(sqrt(2.0*M_PI*pow(sigma,2)))*exp(-pow((z-mu),2)/(2.0*pow(sigma,2)));
	return y;
}


void ObjectModelingDistribution::update(const ros::TimerEvent& e) {
  if(new_data_ && !updated_){

    updated_ = true;
    
    visualization_msgs::MarkerArray marker_array;

	if(pcl_out1_->points.size() > 0){
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid<pcl::PointXYZRGB> (*pcl_out1_,  centroid);
		Eigen::Matrix3f covariance_matrix;
		pcl::computeCovarianceMatrixNormalized<pcl::PointXYZRGB> (*pcl_out1_, centroid, covariance_matrix);

		//std::cout << "Covariance: " << covariance_matrix << std::endl;

		action_msgs::GaussianPointDistribution gaussian_point_dist;
		gaussian_point_dist.mean.x = centroid(0); 
		gaussian_point_dist.mean.y = centroid(1);
		gaussian_point_dist.mean.z = centroid(2); 

		gaussian_point_dist.covariance.cxx = covariance_matrix(0,0); 
		gaussian_point_dist.covariance.cxy = covariance_matrix(0,1); 
		gaussian_point_dist.covariance.cxz = covariance_matrix(0,2); 
		gaussian_point_dist.covariance.cyy = covariance_matrix(1,1); 
		gaussian_point_dist.covariance.cyz = covariance_matrix(1,2); 
		gaussian_point_dist.covariance.czz = covariance_matrix(2,2); 
		
		//Eigen::Matrix3f A(2,2);
		//A << 1,-2,
		//	-3, 4;
		
		if(log_count_<=100){
			std::cout << "Covariance: " <<std::endl;
			std::cout << covariance_matrix << std::endl;			
			glob_norm_hist_[0][log_count_-1] = covariance_matrix.cwiseAbs().colwise().sum().maxCoeff() ; //1-norm
			Eigen::JacobiSVD<Eigen::MatrixXf> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
			glob_norm_hist_[1][log_count_-1] = svd.singularValues().maxCoeff() ; //1-norm
			glob_norm_hist_[2][log_count_-1] = covariance_matrix.norm() ; //1-norm								
			//std::cout << "squarednorm=" << covariance_matrix.squaredNorm() <<std::endl;
			std::cout << "1-norm	= " << glob_norm_hist_[0][log_count_-1] <<std::endl; //1-norm . 1 Norm and norm infinity are the same because the cov matrix is symmetric.
			std::cout << "2-norm	= " << glob_norm_hist_[1][log_count_-1] <<std::endl;		
			std::cout << "Frob norm	= " << glob_norm_hist_[2][log_count_-1] <<std::endl; //frobenius norm
			//std::cout << "lpNorm<1>	= " << covariance_matrix.lpNorm<1>() <<std::endl;
			//std::cout << "lpNorm<2>=" << covariance_matrix.lpNorm<2>() <<std::endl;	
			//std::cout << "lpNorm squared=" << covariance_matrix.squaredNorm() <<std::endl;						
			//std::cout << "lpNorm squared=" << covariance_matrix.lpNorm<Eigen::Infinity> <<std::endl;		
		}
		if(log_count_==100){
			//---compute the mean of norms---
			double norm_mean[3]={0.0,0.0,0.0};
			for (int i=0;i<100;i++){
				norm_mean[0]=norm_mean[0]+glob_norm_hist_[0][i];
				norm_mean[1]=norm_mean[1]+glob_norm_hist_[1][i];
				norm_mean[2]=norm_mean[2]+glob_norm_hist_[2][i];
			}
			norm_mean[0]=norm_mean[0]/100.0;
			norm_mean[1]=norm_mean[1]/100.0;
			norm_mean[2]=norm_mean[2]/100.0;
			
			//---compute the variance of norms---
			double norm_var[3]={0.0,0.0,0.0};
			for (int i=0;i<100;i++){
				norm_var[0]=norm_var[0]+(glob_norm_hist_[0][i]-norm_mean[0])*(glob_norm_hist_[0][i]-norm_mean[0]);
				norm_var[1]=norm_var[1]+(glob_norm_hist_[1][i]-norm_mean[1])*(glob_norm_hist_[1][i]-norm_mean[1]);
				norm_var[2]=norm_var[2]+(glob_norm_hist_[2][i]-norm_mean[2])*(glob_norm_hist_[2][i]-norm_mean[2]);				
			}			
			norm_var[0]=norm_var[0]/100.0;
			norm_var[1]=norm_var[1]/100.0;
			norm_var[2]=norm_var[2]/100.0;
			printf("=================================================\n");
			printf("1-norm	: mean=%11.9f, variance=%11.9f\n" ,norm_mean[0],norm_var[0]);
			printf("2-norm	: mean=%11.9f, variance=%11.9f\n" ,norm_mean[1],norm_var[1]);
			printf("frob_norm:mean=%11.9f, variance=%11.9f\n" ,norm_mean[2],norm_var[2]);			
			printf("========== hue hist =============\n");
			for (int l=0;l<10;l++){
				for (int n=0;n<10;n++){
					std::cout << glob_norm_hist_[0][10*l+n] <<" ";
					//printf("%10.8f ",glob_norm_hist_[0][10*l+n]);
				}
				printf("\n");
			}
			printf("========== sat hist =============\n");
			for (int l=0;l<10;l++){
				for (int n=0;n<10;n++){
					std::cout << glob_norm_hist_[1][10*l+n] <<" ";
					//printf("%10.8f ",glob_norm_hist_[1][10*l+n]);
				}
				printf("\n");
			}
			printf("========== val hist =============\n");
			for (int l=0;l<10;l++){
				for (int n=0;n<10;n++){
					std::cout << glob_norm_hist_[2][10*l+n] <<" ";
					//printf("%10.8f ",glob_norm_hist_[2][10*l+n]);
				}
				printf("\n");
			}
			printf("=================================================\n");
			

		}

		
	  
	    int count=1;
		if(publish_markers_){
		  visualization_msgs::Marker h = getMarker(count, centroid, covariance_matrix);
		  marker_array.markers.push_back(h);
		}
		if((publish_markers_)&&(count>=1)){
		  ////ROS_INFO("publish marker_array");
		  blob_marker_pub_.publish(marker_array);
		}
	}










    //int rangeMin = mean_color_ - window_size_;
    //int rangeMax = mean_color_ + window_size_;
	    
    //Size ksize = Size(2 * blur_kernel_ + 1,2 * blur_kernel_ + 1);
    //GaussianBlur(*hue_image_, *hue_image_, ksize, -1, -1);
    ////blur(*hue_image_, *hue_image_, ksize, Point(-1, -1), BORDER_REPLICATE);
    ////dilate(*hue_image_, *hue_image_, Mat(), Point(-1, -1), BORDER_REPLICATE);	    
    
    //*binary_image_ = Mat::zeros(hue_image_->size(), CV_8UC1);
    ////*binary_image2_ = Mat::zeros(hue_image_->size(), CV_8UC1);

    //if((rangeMin >= 0) && (rangeMax <= 179)){
      //inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *binary_image_);
      ////inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *binary_image2_);
    //}
    //else if (rangeMin >= 0){
      //Mat temp_binary_image1(hue_image_->size(), CV_8UC1);
      //Mat temp_binary_image2(hue_image_->size(), CV_8UC1);

      //inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)179)), temp_binary_image1);
      //inRange(*hue_image_, Scalar((double)((uchar)0)),Scalar((double)((uchar)(rangeMax - 179))), temp_binary_image2);
      //bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image_);
      ////bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image2_);
    //}
    //else if (rangeMax <= 179){
      //Mat temp_binary_image1(hue_image_->size(), CV_8UC1);
      //Mat temp_binary_image2(hue_image_->size(), CV_8UC1);

      //inRange(*hue_image_, Scalar((double)((uchar)0)),Scalar((double)((uchar)rangeMax)), temp_binary_image1);
      //inRange(*hue_image_, Scalar((double)((uchar)(179 + rangeMin))),Scalar((double)((uchar)179)), temp_binary_image2);
      //bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image_);
      ////bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image2_);
    //}
    //else {
      //ROS_ERROR("Window size parameter is too large.  Decrease it!");
      //return;
    //}  

    //*threshold_image_ = Mat::zeros(hue_image_->size(), CV_8UC3);
    //rgb_image_->copyTo(*threshold_image_, *binary_image_);

    //getPointIndices();
    
    //if(color_indices_->indices.size() > 0){
    
      //pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster;	
      //stable_color_regions_indexes_.clear();
      //pcl::KdTree<pcl::PointXYZRGB>::Ptr clusters_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();

      //cluster.setClusterTolerance (within_cluster_distance_);
      //cluster.setMinClusterSize (min_cluster_size_);
      //cluster.setMaxClusterSize (max_cluster_size_);
      //cluster.setSearchMethod (clusters_tree);

      //// Cluster potential stable regions

      //cluster.setInputCloud (pcl_in_);
      //cluster.setIndices (color_indices_);
      //cluster.extract (stable_color_regions_indexes_);
     
      //*cluster_image_ = Mat::zeros(hue_image_->size(), CV_8UC1);
      
      ////array of clusters
      //action_msgs::GaussianPointDistributionArray dist_array;
      //visualization_msgs::MarkerArray marker_array;

      ////fill in header
      //dist_array.header.frame_id = base_frame_;
      //dist_array.header.stamp = ros::Time::now();
      //dist_array.feature_name = feature_name_;
      //dist_array.feature_type = feature_type_;

      ////fill in attributes
      //for (std::map<std::string, std::vector<double> >::iterator attrib = attributes_.begin(); attrib != attributes_.end(); attrib++){
		//action_msgs::Attribute temp_attribute;
		//temp_attribute.name = attrib->first;
		//temp_attribute.values = attrib->second;
		//dist_array.attributes.push_back(temp_attribute);
      //}

      //int count = 0;
      //for (std::vector<pcl::PointIndices>::const_iterator stable_color_region_index = stable_color_regions_indexes_.begin(); stable_color_region_index != stable_color_regions_indexes_.end(); stable_color_region_index++){

		////std::cout << "cluster: " << count << " size: " << stable_color_region_index->indices.size() << std::endl;

		//if(count <= max_num_features_) {

			//if(stable_color_region_index->indices.size() > 0){
				//Eigen::Vector4f centroid;
				//pcl::compute3DCentroid<pcl::PointXYZRGB> (*pcl_in_, stable_color_region_index->indices, centroid);
				//Eigen::Matrix3f covariance_matrix;
				//pcl::computeCovarianceMatrixNormalized<pcl::PointXYZRGB> (*pcl_in_, stable_color_region_index->indices, centroid, covariance_matrix);

				////std::cout << "Covariance: " << covariance_matrix << std::endl;

				//if (display_image_){
					//for (std::vector<int>::const_iterator pit = stable_color_region_index->indices.begin(); pit != stable_color_region_index->indices.end(); pit++){
						////std::cout << "index: " << (*pit) << std::endl;
						////std::cout << "x: " << (*pit)%(pcl_in_->width) << " y: " << (*pit)/(pcl_in_->width) << std::endl;
						////std::cout << "img val: " << ((int)threshold_image_->at<uchar>((*pit)%(pcl_in_->width), (*pit)/(pcl_in_->width))) << std::endl;
						//cluster_image_->at<uchar>((*pit)/(pcl_in_->width), (*pit)%(pcl_in_->width)) = ((uchar)255); 
					//}
				//}
				//action_msgs::GaussianPointDistribution gaussian_point_dist;
				//gaussian_point_dist.mean.x = centroid(0); 
				//gaussian_point_dist.mean.y = centroid(1);
				//gaussian_point_dist.mean.z = centroid(2); 

				//gaussian_point_dist.covariance.cxx = covariance_matrix(0,0); 
				//gaussian_point_dist.covariance.cxy = covariance_matrix(0,1); 
				//gaussian_point_dist.covariance.cxz = covariance_matrix(0,2); 
				//gaussian_point_dist.covariance.cyy = covariance_matrix(1,1); 
				//gaussian_point_dist.covariance.cyz = covariance_matrix(1,2); 
				//gaussian_point_dist.covariance.czz = covariance_matrix(2,2); 

				//dist_array.distributions.push_back(gaussian_point_dist);
			  
				//if(publish_markers_){
				  //visualization_msgs::Marker h = getMarker(count, centroid, covariance_matrix);
				  //marker_array.markers.push_back(h);
				//}

				//count++;
		
				//if (display_image_)
					//binary_image_->copyTo(*binary_image_, *cluster_image_);	
					
						  
			//}
		//}
      //}
      

      //if (display_image_){
		//*threshold_image_ = Mat::zeros(hue_image_->size(), CV_8UC3);
		//rgb_image_->copyTo(*threshold_image_, *binary_image_);	  
		////imshow(window_name_, *threshold_image_); //pop up a window which displays the image
		//////imshow(window_name_, *binary_image_);
      //}

      //moments_pub_.publish(dist_array);
      //if((publish_markers_)&&(count>=1)){
		  ////ROS_INFO("publish marker_array");
		  //blob_marker_pub_.publish(marker_array);
	  //}else{
		//ROS_INFO("Not publish marker array");
		//visualization_msgs::MarkerArray no_marker_array;

		//no_blob_marker_pub_.publish(no_marker_array);
		//ROS_INFO("Not publish marker array done");

	  //}
    //}
	////ROS_INFO("new_data_=false");

    new_data_ = false;
  }   
} //update

visualization_msgs::Marker ObjectModelingDistribution::getMarker(int marker_id, Eigen::Vector4f centroid, Eigen::Matrix3f covariance_matrix){

  //------- Compute Principal Componets for Marker publishing


  //Get the principal components and the quaternion
  Eigen::Matrix3f evecs;
  Eigen::Vector3f evals;
  //pcl::eigen33 (covariance_matrix, evecs, evals);
  Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
	
  evecs = es.eigenvectors().real();
  evals = es.eigenvalues().real();
	    
  Eigen::Matrix3f rotation;
  rotation.row (0) = evecs.col (0);
  rotation.row (1) = evecs.col (1);
  rotation.row (2) = rotation.row (0).cross (rotation.row (1));
	    
  rotation.transposeInPlace ();
  Eigen::Quaternion<float> qt (rotation);
  qt.normalize ();
	    
  //Publish Marker for cluster
  visualization_msgs::Marker marker;	
		
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time().now();
  marker.ns = "Perception";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = marker_id;	
  marker.lifetime = ros::Duration(1);	
		
  //centroid position
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];	
  marker.pose.orientation.x = qt.x();
  marker.pose.orientation.y = qt.y();
  marker.pose.orientation.z = qt.z();
  marker.pose.orientation.w = qt.w();			

  //std::cout << "e1: " << evals(0) << " e2: " << evals(1) << " e3: " << evals(2) << std::endl;

  marker.scale.x = sqrt(evals(0)) * 4;
  marker.scale.y = sqrt(evals(1)) * 4;
  marker.scale.z = sqrt(evals(2)) * 4;
	

  //give it some color!
  marker.color.a = 0.75;
  hueToRGB(marker.color.r, marker.color.g, marker.color.b);

  //std::cout << "marker being published" << std::endl;

  return marker;
}

void ObjectModelingDistribution::hueToRGB(float &r, float &g, float &b){

  //mean_color_ += 1;
  //ROS_INFO("mean_color=%d", mean_color_);

  if ((2.0 * (float)mean_color_) < 120.0) {
    r = (120.0 - (2.0 * (float)mean_color_)) / 120.0;
    g = (2.0 * (float)mean_color_) / 120.0;
    b = 0;
  } 
  else if ((2.0 * (float)mean_color_) < 240.0) {
    r = 0;
    g = (240.0 - (2.0 * (float)mean_color_)) / 120.0;
    b = ((2.0 * (float)mean_color_) - 120.0) / 120.0;
  } 
  else {
    r = ((2.0 *(float)mean_color_) - 240.0) / 120.0;
    g = 0;
    b = (360.0 - (2.0 * (float)mean_color_)) / 120.0;
  }

  float max_val = std::max( std::max(r,g) , b);
  if(max_val != 0){
    r /= max_val;
    g /= max_val;
    b /= max_val;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_modeling_distribution");
  ros::NodeHandle nh("~");
  
  ObjectModelingDistribution node(ros::this_node::getName(), nh);
  ros::spin();
    
  return 0;
}
  
