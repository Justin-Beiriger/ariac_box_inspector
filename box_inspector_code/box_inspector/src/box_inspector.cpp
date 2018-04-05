//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();
  
  // convert models wrt cam to models wrt world
  geometry_msgs::Pose cam_pose, model_pose_wrt_world;
  cam_pose = box_inspector_image_.pose;
  vector<osrf_gear::Model> models_seen_wrt_world;
  osrf_gear::Model model_seen_wrt_world;
  
  // convert logical camera output to coordinates wrt world frame
  for (int i=0; i<num_parts_seen;i++) {
      model_pose_wrt_world = compute_stPose(cam_pose, box_inspector_image_.models[i].pose).pose;
      model_seen_wrt_world.type = box_inspector_image_.models[i].type;
      model_seen_wrt_world.pose = model_pose_wrt_world;
      models_seen_wrt_world.push_back(model_seen_wrt_world);
  }
  
  osrf_gear::Model model_seen, model_desired;
  int num_parts_desired = desired_models_wrt_world.size();
  double translational_error, rotation_seen, rotation_desired, rotational_error, dx, dy, dz;
  // allowable translational error in m
  double translational_tolerance = 0.1;
  // allowable rotational error (rad)
  double rotational_tolerance = 0.1;
  
  
  vector<osrf_gear::Model> models_seen_not_properly_placed;
  vector<osrf_gear::Model> models_desired_not_accounted_for;
 
  bool isProperlyPlaced;
  vector<bool> isAccountedFor;
  // initialize vector 
  for (int i=0; i<num_parts_desired; i++) {
      isAccountedFor.push_back(false);
  }
  
  // loop through each part seen and compare to all desired parts of same type
  // this loop extracts the parts that are in the right place
  for (int i=0;i<num_parts_seen;i++) {
      model_seen = models_seen_wrt_world[i];
      
      // check to see if each seen part is properly placed
      isProperlyPlaced = false;
      
      for (int j = 0; j < num_parts_desired; j++) {
          model_desired = desired_models_wrt_world[j];
          if (model_seen.type == model_desired.type) {
              // compute translational and rotational error for each part
              dx = model_seen.pose.position.x - model_desired.pose.position.x;
              dy = model_seen.pose.position.y - model_desired.pose.position.y;
              dz = model_seen.pose.position.z - model_desired.pose.position.z;
              
              translational_error = dx*dx + dy*dy + dz*dz;
              
              rotation_seen = xformUtils_.convertPlanarQuat2Phi(model_seen.pose.orientation);
              rotation_desired = xformUtils_.convertPlanarQuat2Phi(model_desired.pose.orientation);
              
              rotational_error = rotation_seen - rotation_desired;
              if (rotational_error < 0) {
                  rotational_error = -1*rotational_error;
              }
              
              // if the part is where it should be, add it to the satisfied models list
              if (translational_error < (translational_tolerance*translational_tolerance) && (rotational_error < rotational_tolerance)) {
                  satisfied_models_wrt_world.push_back(model_seen);
                  isProperlyPlaced = true;
                  isAccountedFor[j] = true;
              }
          }
      }
      
      if (!isProperlyPlaced) {
          models_seen_not_properly_placed.push_back(model_seen);
      }
     
  }
  
  // create vector of models that have not been accounted for
  for (int i=0; i<num_parts_desired; i++) {
      // if part has not been accounted for, add it to the vector
      if (!isAccountedFor[i])
          models_desired_not_accounted_for.push_back(desired_models_wrt_world[i]);
  }
  
  int index_of_closest_seen_part;
  double error_of_closest_part = 100;
  bool found_misplaced_part; 
  vector<osrf_gear::Model> save_models_not_properly_placed;
  
  // at this point, all the models in correct poses are in the vector satisfied_models_wrt_world
  // and all the models that are seen that are not in correct poses are models_not_properly_placed
  // Now, find all misplaced parts
  for (int i=0; i<models_desired_not_accounted_for.size(); i++) {
      model_desired = models_desired_not_accounted_for[i];
      found_misplaced_part = false;
      error_of_closest_part = 100;
      
      for (int j=0; j<models_seen_not_properly_placed.size(); j++) {
          model_seen = models_seen_not_properly_placed[j];
          
          if (model_seen.type == model_desired.type) {
              // compute translational error for each part
              dx = model_seen.pose.position.x - model_desired.pose.position.x;
              dy = model_seen.pose.position.y - model_desired.pose.position.y;
              dz = model_seen.pose.position.z - model_desired.pose.position.z;
              
              translational_error = dx*dx + dy*dy + dz*dz;
              
              if (j == 0) {
                  error_of_closest_part = translational_error;
                  found_misplaced_part = true;
                  index_of_closest_seen_part = j;
              }
              else if (translational_error < error_of_closest_part) {
                  index_of_closest_seen_part = j;
                  error_of_closest_part = translational_error;
              }
              
          }
      }
      
      // remove the closest part from models_seen_not_properly_placed vector 
      if (found_misplaced_part) {
          save_models_not_properly_placed = models_seen_not_properly_placed;
          models_seen_not_properly_placed.clear();
          for (int k=0; k<save_models_not_properly_placed.size(); k++) {
              if (k != index_of_closest_seen_part) {
                  models_seen_not_properly_placed.push_back(save_models_not_properly_placed[k]);
              }
              else {
                  misplaced_models_actual_coords_wrt_world.push_back(save_models_not_properly_placed[k]);
                  misplaced_models_desired_coords_wrt_world.push_back(model_desired);
              }
          }
      }
      else {
          missing_models_wrt_world.push_back(model_desired);
      }
      
  }
  
  
  // at this point, all parts are in the correct vectors except orphan parts
  // orphan parts are in the vector models_seen_not_properly_placed
  for (int i=0; i<models_seen_not_properly_placed.size(); i++) {
      orphan_models_wrt_world.push_back(models_seen_not_properly_placed[i]);
  }
  

}

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
