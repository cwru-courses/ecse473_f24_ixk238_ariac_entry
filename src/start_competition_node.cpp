#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "ik_service/PoseIK.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h" // L6
#include "trajectory_msgs/JointTrajectoryPoint.h" // L6
#include "actionlib/client/simple_action_client.h" // L7 -Action Server Headers
#include "actionlib/client/terminal_state.h" // L7- Action Sever Headers
#include "control_msgs/FollowJointTrajectoryAction.h" // L7 - Action Server "message type"

#define GREEN_TEXT "\033[32m"
#define RESET_TEXT "\033[0m"
#define BLUE_TEXT  "\033[34m"

// Declare the variable in this way where necessary in the code.
std_srvs::Trigger begin_comp;
// Declare an order vector
std::vector<osrf_gear::Order> order_vector;
// Declare the vector for the logical cameras (There are 3 types of them)
std::vector<osrf_gear::LogicalCameraImage> logical_cameras_bin(6);
std::vector<osrf_gear::LogicalCameraImage> logical_cameras_agv(2);
std::vector<osrf_gear::LogicalCameraImage> logical_cameras_quality(2);

// Declare the transformation buffer to maintain a list of transformations
tf2_ros::Buffer tfBuffer;

// Declare the found product pose vector
std::vector<geometry_msgs::Pose> found_product_pose;

//Declare an ik_service::PoseIK variable
ik_service::PoseIK ik_pose;

// L6- Declare a variable for storing the joint names of the robot.
std::vector<std::string> joint_names;

//L6-  Declare a variable for storing the current state of the joints of the robot.
sensor_msgs::JointState joint_states;

// L6- Publisher for joint trajectory
ros::Publisher joint_trajectory_publisher;

// L6- Joint trajectory header count
int joint_trajectory_header_count = 0;

//L6-
std::vector<geometry_msgs::Pose> specific_poses;

trajectory_msgs::JointTrajectoryPoint original_joint_trajectory_point;

// L7 - initialize a goal number and sequence number for the header
int goal_num = 0;
static int sequence_num = 0;
static int found_product_index = 0;

// This function is written to get the appropriate camera data according to given storage unit id
const osrf_gear::LogicalCameraImage* getCameraData(
    const std::string& storage_unit_id,
    const std::vector<osrf_gear::LogicalCameraImage>& logical_cameras_bin,
    const std::vector<osrf_gear::LogicalCameraImage>& logical_cameras_agv,
    const std::vector<osrf_gear::LogicalCameraImage>& logical_cameras_quality) {
    
    // Determine the camera type based on the storage unit ID prefix
    if (storage_unit_id.find("bin") == 0) {
        // It's a bin camera
        int bin_index = std::stoi(storage_unit_id.substr(3)) - 1;
        if (bin_index >= 0 && bin_index < logical_cameras_bin.size()) {
            return &logical_cameras_bin[bin_index];
        } else {
            ROS_WARN_STREAM("Invalid bin index for storage unit: " << storage_unit_id);
        }
    } else if (storage_unit_id.find("agv") == 0) {
        // It's an AGV camera
        int agv_index = std::stoi(storage_unit_id.substr(3)) - 1;
        if (agv_index >= 0 && agv_index < logical_cameras_agv.size()) {
            return &logical_cameras_agv[agv_index];
        } else {
            ROS_WARN_STREAM("Invalid AGV index for storage unit: " << storage_unit_id);
        }
    } else if (storage_unit_id.find("quality") == 0) {
        // It's a quality control camera
        int quality_index = std::stoi(storage_unit_id.substr(7)) - 1;
        if (quality_index >= 0 && quality_index < logical_cameras_quality.size()) {
            return &logical_cameras_quality[quality_index];
        } else {
            ROS_WARN_STREAM("Invalid quality control index for storage unit: " << storage_unit_id);
        }
    } else {
        ROS_WARN_STREAM("Unknown storage unit prefix: " << storage_unit_id);
    }
    
    return nullptr; // Return nullptr if no valid camera data found
}

//This function is written to debug what logical camera data looks like in the callbacks
void printlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg) {
    ROS_INFO("Received LogicalCameraImage with %zu models", msg->models.size());

    for (const auto& model : msg->models) {
        // Print model type
        ROS_INFO("Model Type: %s", model.type.c_str());

        // Print position
        ROS_INFO("Position - x: %.2f, y: %.2f, z: %.2f",
                 model.pose.position.x,
                 model.pose.position.y,
                 model.pose.position.z);

        // Print orientation
        ROS_INFO("Orientation - x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                 model.pose.orientation.x,
                 model.pose.orientation.y,
                 model.pose.orientation.z,
                 model.pose.orientation.w);
    }
}

// L6- Callback to receive the state of the joints of the robot
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
  joint_states = *msg;
  static bool was_moving = false;
  bool is_moving = std::any_of(msg->velocity.begin(), msg->velocity.end(), [](double vel) { return fabs(vel) > 0.001; });

  if (is_moving && !was_moving) {
    ROS_INFO("%sThe arm is moving",GREEN_TEXT);
  } else if (!is_moving && was_moving) {
    ROS_INFO("%sThe arm is not in motion",GREEN_TEXT);
  }

  was_moving = is_moving;
}


// Callback function to subscribe to logical cameras
// agv Cameras
void  logicagv1CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  //printlogicalCameraCallback(msg);
  logical_cameras_agv[0] = *msg;
}
void  logicagv2CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 //printlogicalCameraCallback(msg);
  logical_cameras_agv[1] = *msg;
}
// Quality cameras callback
void logicQuality1CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 //printlogicalCameraCallback(msg);
  logical_cameras_quality[0] = *msg;
}
void logicQuality2CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 //printlogicalCameraCallback(msg);
  logical_cameras_quality[1] = *msg;
}
 //Bin Cameras
 void logicbin1CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 // printlogicalCameraCallback(msg);
  logical_cameras_bin[0] = *msg;
}
 void logicbin2CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 //printlogicalCameraCallback(msg);
  logical_cameras_bin[1] = *msg;
}
 void logicbin3CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 //printlogicalCameraCallback(msg);
  logical_cameras_bin[2] = *msg;
}
 void logicbin4CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
//printlogicalCameraCallback(msg);
  logical_cameras_bin[3] = *msg;
}
 void logicbin5CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 //printlogicalCameraCallback(msg);
  logical_cameras_bin[4] = *msg;
}
 void logicbin6CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
//printlogicalCameraCallback(msg);
  logical_cameras_bin[5] = *msg;
}


//Callback function to subscribe the orders
void orderCallback(const osrf_gear::Order::ConstPtr& orders){
 // Add order to the end of the vector
  order_vector.push_back(*orders);
}

void logJointAngles(const auto& ik_pose) {
  for(int i = 0; i < ik_pose.num_sols; i++) {
    ROS_INFO("Solution Num: %d", i + 1);
    for(int j = 0; j < 6; j++) {
      ROS_INFO("Joint angle [%d]: %.4f ", j + 1, ik_pose.joint_solutions[i].joint_angles[j]);
    }
  }
}

// this function choose the solution according to the position of part wrt camera too
int chooseSolution(const auto& ik_pose, geometry_msgs::PoseStamped& part_pose) {
  const double tolerance = 0.1; // Add a small tolerance to wrist_2_joint angle

  for(int i = 0; i < ik_pose.num_sols; i++) {
    // Restrict the solution for shoulder_lift_joint to a region of 90 degrees to reduce solutions
    double shoulder_lift_angle = ik_pose.joint_solutions[i].joint_angles[1];
      // Further restrict based on wrist_2_joint angle to pick only one solution
      double wrist_2_angle = ik_pose.joint_solutions[i].joint_angles[4];
      if ((wrist_2_angle >= (1.5708 - tolerance) && wrist_2_angle <= (1.5708 + tolerance)) ||
          (wrist_2_angle >= (3 * 1.5708 - tolerance) && wrist_2_angle <= (3 * 1.5708 + tolerance))) {
         double wrist_3_angle = ik_pose.joint_solutions[i].joint_angles[5];
         if(part_pose.pose.position.y > 0 ){
            // select solution num 2 
            if(wrist_3_angle <= 1.5708){
               return 2;
            }
            else if(wrist_3_angle<=3*1.5708){ // select solution num0
              return 0;
            }
             
         }
      }
      return 0;
    
  }
  return -1; // Return -1 if no valid solution is found
}

// This function checking whether arm is movin or not
bool armMoving(const sensor_msgs::JointState& joint_states) {
    double velocity_threshold = 0.01; 
    for (const auto& velocity : joint_states.velocity) {
        if (std::abs(velocity) > velocity_threshold) {
            return true; 
        }
    }
    return false; 
}

/** L6- moveToSpecıfıc Points Function
 *  Moves the UR10 robot arm to a series of predefined poses using inverse kinematics (IK) solutions.
 *
 * This function generates a trajectory to move the robot arm through a sequence of specific positions 
 * and orientations defined in Cartesian space. The function calculates joint configurations for each pose 
 * using an inverse kinematics service, then publishes the resulting joint trajectories for execution.
 *
 * Key functionality:
 * - Defines a series of Cartesian positions and orientations for the robot's end-effector.
 * - Uses the `ik_service::PoseIK` service to calculate joint angle solutions for each pose.
 * - Selects a specific IK solution (based on `solutionNum`) to generate a joint trajectory.
 * - Publishes the trajectory to the robot for execution.
 * - Transforms and logs the executed pose in the "world" frame for verification.
 *
 */
void moveToSpecificPoints(ik_service::PoseIK& ik_pose, ros::ServiceClient& pose_ik_client, int solutionNum ) {
  // Clear the vector
  specific_poses.clear();

  std::vector<std::vector<double>> joint_angle_trajectory;

  // Define the positions and orientations for each pose
  std::vector<std::vector<double>> positions = {
      {0.75, 0.0, 0.95},
      {0.0, 0.75, 0.95},
      {0.25, 0.75, 0.95},
      {0.75, 0.25, 0.95},
      {0.75, 0.45, 0.75}
  };

  std::vector<std::vector<double>> orientations = {
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0}
  };

  // Iterate over each set of positions and orientations to create poses
  for (size_t i = 0; i < positions.size(); ++i) {
    geometry_msgs::Pose pose;

    // Set position
    pose.position.x = positions[i][0];
    pose.position.y = positions[i][1];
    pose.position.z = positions[i][2];

    // Set orientation
    pose.orientation.w = orientations[i][0];
    pose.orientation.x = orientations[i][1];
    pose.orientation.y = orientations[i][2];
    pose.orientation.z = orientations[i][3];

    // Set the request field of the ik_service::PoseIK variable equal to the goal pose
    ik_pose.request.part_pose = pose;
    // Update the client.call() to use the ik_service::PoseIK variable.
    bool success = pose_ik_client.call(ik_pose);
    if (success) {
      ROS_INFO("%smtsp: Call to ik_service returned [%i] solutions",GREEN_TEXT, ik_pose.response.num_sols);
      if (ik_pose.response.num_sols > 0) {
        // Iterate over the solution and print the angles
        ROS_INFO("%smtsp:Chosen solution: %d",GREEN_TEXT, solutionNum);
        if (solutionNum >= 0 && solutionNum < ik_pose.response.num_sols) {
          for (int j = 0; j < ik_pose.response.joint_solutions[solutionNum].joint_angles.size(); j++) {
            ROS_INFO("%smtsp:Joint angle [%d]: %.4f",GREEN_TEXT, j + 1, ik_pose.response.joint_solutions[solutionNum].joint_angles[j]);
          }
          // create joint_angles variable from the response of the ik_pose response
          std::vector<double> joint_angles(ik_pose.response.joint_solutions[solutionNum].joint_angles.begin(),
                                           ik_pose.response.joint_solutions[solutionNum].joint_angles.end());
          // give the names of the joint trajectory (according to the order mentioned in the lab)
          trajectory_msgs::JointTrajectory joint_trajectory;
          joint_trajectory.joint_names = {
          "linear_arm_actuator_joint",
          "shoulder_pan_joint",
          "shoulder_lift_joint",
          "elbow_joint",
          "wrist_1_joint",
          "wrist_2_joint",
          "wrist_3_joint"
        };
        // create joint trajectory for the start points
        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.positions.resize(joint_trajectory.joint_names.size());
        for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
          for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
            if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
              start_point.positions[t_joint] = joint_states.position[s_joint];
              break;
            }
          }
        }

        // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
        start_point.positions[0] = joint_states.position[1];
        start_point.time_from_start = ros::Duration(0.0);

        // create joint trajectory for the goal points
        trajectory_msgs::JointTrajectoryPoint goal_point;
        goal_point.positions.resize(joint_trajectory.joint_names.size());
        start_point.positions[0] = joint_states.position[1];
        for(int j = 0; j < joint_angles.size(); j++){
          goal_point.positions[j+1] = joint_angles[j];
        }
        goal_point.time_from_start = ros::Duration(3.0); 

        joint_trajectory.points.push_back(start_point);
        joint_trajectory.points.push_back(goal_point);

        // Publish the desired trajectory
        joint_trajectory_publisher.publish(joint_trajectory);
         
        }
      }
    } else {
      ROS_WARN("mtsp: Failed to call ik_service!!");
    }
    // log the published trajectory pose
    ROS_INFO("%sPublished trajectory to pose (%.2f, %.2f, %.2f)",GREEN_TEXT, pose.position.x, pose.position.y, pose.position.z);

    ros::Rate rate(1);
    while (ros::ok()) {
      // if the arm is not moving anymore, that mean the robot reached the desired position
      if (!armMoving(joint_states)) {
        ROS_INFO("%sUR10 has reached pose (%.2f, %.2f, %.2f)", GREEN_TEXT,
                         pose.position.x, pose.position.y, pose.position.z);
        // Since it is not explicitly mentioned in the lab description, I assumed that the given poses are
        // according to the arm1_base_link frame, not according to the world frame. THerefore,
        // to check whether the end-effector is in the desired position ı transformed the wanted position
        // to the world frame.
        // print the transformed version of this to the workcell so you can check
        geometry_msgs::TransformStamped tfStamped_specific;
        tfStamped_specific = tfBuffer.lookupTransform("world", "arm1_base_link", ros::Time(0.0), ros::Duration(1.0));
        // Create variables
        geometry_msgs::PoseStamped robot_frame_pose, workcell_frame_pose;
        robot_frame_pose.pose = pose;
        tf2::doTransform(robot_frame_pose, workcell_frame_pose, tfStamped_specific);
        // Log transformed pose in robot frame
        ROS_INFO("%sRobot Frame Pose:\n"
                 "  Position: (%.2f, %.2f, %.2f)\n"
                 "  Orientation: (x: %.2f, y: %.2f, z: %.2f, w: %.2f)", GREEN_TEXT,
                 robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y, robot_frame_pose.pose.position.z,
                 robot_frame_pose.pose.orientation.x, robot_frame_pose.pose.orientation.y, 
                 robot_frame_pose.pose.orientation.z, robot_frame_pose.pose.orientation.w);
        tf2::Quaternion quat_workcell(workcell_frame_pose.pose.orientation.x,
                                      workcell_frame_pose.pose.orientation.y,
                                      workcell_frame_pose.pose.orientation.z,
                                      workcell_frame_pose.pose.orientation.w);
        double roll_workcell, pitch_workcell, yaw_workcell;
        tf2::Matrix3x3(quat_workcell).getRPY(roll_workcell, pitch_workcell, yaw_workcell);
        // Print position and orientation in world frame
        ROS_INFO("%sWorld Frame Pose:\n"
                 "  Position: (%.2f, %.2f, %.2f)\n"
                 "  Orientation (RPY): Roll: %.2f, Pitch: %.2f, Yaw: %.2f", GREEN_TEXT,
                 workcell_frame_pose.pose.position.x, workcell_frame_pose.pose.position.y, workcell_frame_pose.pose.position.z,
                 roll_workcell, pitch_workcell, yaw_workcell);
        // ask the user to hit enter to continue the execution
         // Ask the user to press Enter to continue
        ROS_INFO("Press Enter to move robot to next specified point...");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        break;
      }
      rate.sleep();
    }
  }
}

/** L6- setAndPublishJointTrajectory function
 *  Sets and publishes a single waypoint for a joint trajectory.
 *
 * This function prepares and publishes a trajectory message for a robotic arm. 
 * It defines a single waypoint in joint space based on the current joint states, 
 * with a slight adjustment to the elbow joint (index 3) for demonstration purposes. 
 * The waypoint is published as a `trajectory_msgs::JointTrajectory` message.
 *
 */
void setAndPublishJointTrajectory() {
  // Set a waypoint.
  // Declare a joint trajectory message variable
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = {
    "linear_arm_actuator_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };
  joint_trajectory.points.clear();
  // Fill out the joint trajectory header.
  // Each joint trajectory should have a non-monotonically increasing sequence number.
  joint_trajectory.header.seq = joint_trajectory_header_count++; // Keep a variable to increment the count.
  joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
  joint_trajectory.header.frame_id = "arm1_base_link"; // Frame in which this is specified.


  // Create a variable to hold a single point.
  trajectory_msgs::JointTrajectoryPoint point;

  // Set the start point to the current position of the joints from joint_states.
  point.positions.resize(joint_trajectory.joint_names.size());
  for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
    for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
      if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
        point.positions[t_joint] = joint_states.position[s_joint];
        break;
      }
    }
  }

  // The index for the elbow joint is 3 (assuming standard UR10 naming conventions).
  //point.positions[3] += 0.1;

  // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
  point.positions[0] = joint_states.position[1];

  // The duration of the movement.
  point.time_from_start = ros::Duration(0);

  // When to start (shortly after receipt).
  point.time_from_start = ros::Duration(1);

  // Add the point to the joint trajectory
  joint_trajectory.points.push_back(point);
  ROS_INFO("Press Enter to move the elbow joint only...");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // Publish the desired single waypoint.
  joint_trajectory_publisher.publish(joint_trajectory);
  ROS_INFO("Press Enter to move robot to next specified point...");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

//L7- Linear Arm Movement in front of the bin 
void moveInFrontOfBin(double linear_arm_point) {
  // Set a waypoint.
  // Declare a joint trajectory message variable
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = {
    "linear_arm_actuator_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };

  /// create joint trajectory for the start points
  trajectory_msgs::JointTrajectoryPoint start_point;
  start_point.positions.resize(joint_trajectory.joint_names.size());
  for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
    for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
      if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
          start_point.positions[t_joint] = joint_states.position[s_joint];
          break;
        }
      }
    }
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    start_point.time_from_start = ros::Duration(0.0);
    start_point.positions[0] = joint_states.position[0];  // Match the current position


  /// create joint trajectory for the start points
  trajectory_msgs::JointTrajectoryPoint end_point;
  end_point.positions.resize(joint_trajectory.joint_names.size());
  for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
    for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
      if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
          end_point.positions[t_joint] = joint_states.position[s_joint];
          break;
        }
      }
    }
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    end_point.positions[0] = linear_arm_point;
    end_point.time_from_start = ros::Duration(6.0);
    
    joint_trajectory.points.push_back(start_point);
    joint_trajectory.points.push_back(end_point);

    // Publish the desired trajectory
    joint_trajectory_publisher.publish(joint_trajectory);
    joint_trajectory_header_count++;
    ROS_INFO("Sleeping for 7 seconds...");
    ros::Duration(7.0).sleep();
    ROS_INFO("Awake now!");
}


// L7- Callbacks
// This callback is called when a goal becomes active.
void resultCallback(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO("%sAction completed with state: %s", BLUE_TEXT,state.toString().c_str());
}

void goalActiveCallback() {
    ROS_INFO("%sGoal is now active.",BLUE_TEXT);
}

void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback) {
    ROS_DEBUG("%sReceived feedback.",BLUE_TEXT);
}


trajectory_msgs::JointTrajectory returnJointTrajectory(ik_service::PoseIK& ik_pose, ros::ServiceClient& pose_ik_client, int solutionNum){
   // Clear the vector
  specific_poses.clear();
  // give the names of the joint trajectory (according to the order mentioned in the lab)
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = {
          "linear_arm_actuator_joint",
          "shoulder_pan_joint",
          "shoulder_lift_joint",
          "elbow_joint",
          "wrist_1_joint",
          "wrist_2_joint",
          "wrist_3_joint"
        };
  // Define the positions and orientations for each pose
  std::vector<std::vector<double>> positions = {
      {0.75, 0.0, 0.95},
      {0.0, 0.75, 0.95},
      {0.25, 0.75, 0.95},
      {0.75, 0.25, 0.95},
      {0.75, 0.45, 0.75}
  };

  std::vector<std::vector<double>> orientations = {
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0}
  };
  double prev_goal_time = 0;

  // Iterate over each set of positions and orientations to create poses
  for (size_t i = 0; i < positions.size(); ++i) {
    geometry_msgs::Pose pose;

    // Set position
    pose.position.x = positions[i][0];
    pose.position.y = positions[i][1];
    pose.position.z = positions[i][2];

    // Set orientation
    pose.orientation.w = orientations[i][0];
    pose.orientation.x = orientations[i][1];
    pose.orientation.y = orientations[i][2];
    pose.orientation.z = orientations[i][3];

    // Set the request field of the ik_service::PoseIK variable equal to the goal pose
    ik_pose.request.part_pose = pose;
    // Update the client.call() to use the ik_service::PoseIK variable.
    bool success = pose_ik_client.call(ik_pose);
    if (success) {
      ROS_INFO("%smtsp: Call to ik_service returned [%i] solutions",GREEN_TEXT, ik_pose.response.num_sols);
      if (ik_pose.response.num_sols > 0) {
        // Iterate over the solution and print the angles
        ROS_INFO("%smtsp:Chosen solution: %d",GREEN_TEXT, solutionNum);
        if (solutionNum >= 0 && solutionNum < ik_pose.response.num_sols) {
          for (int j = 0; j < ik_pose.response.joint_solutions[solutionNum].joint_angles.size(); j++) {
            ROS_INFO("%smtsp:Joint angle [%d]: %.4f",GREEN_TEXT, j + 1, ik_pose.response.joint_solutions[solutionNum].joint_angles[j]);
          }
          // create joint_angles variable from the response of the ik_pose response
          std::vector<double> joint_angles(ik_pose.response.joint_solutions[solutionNum].joint_angles.begin(),
                                           ik_pose.response.joint_solutions[solutionNum].joint_angles.end());
        // create joint trajectory for the start points
        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.positions.resize(joint_trajectory.joint_names.size());
        for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
          for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
            if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
              start_point.positions[t_joint] = joint_states.position[s_joint];
              break;
            }
          }
        }

        // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
        start_point.positions[0] = joint_states.position[1];
        start_point.time_from_start = ros::Duration(prev_goal_time + 3.0);

        // create joint trajectory for the goal points
        trajectory_msgs::JointTrajectoryPoint goal_point;
        goal_point.positions.resize(joint_trajectory.joint_names.size());
        goal_point.positions[0] = joint_states.position[1];
        for(int j = 0; j < joint_angles.size(); j++){
          goal_point.positions[j+1] = joint_angles[j];
        }
        goal_point.time_from_start = ros::Duration(prev_goal_time + 7.0); 
        prev_goal_time = prev_goal_time + 7.0;
        joint_trajectory.points.push_back(start_point);
        joint_trajectory.points.push_back(goal_point);

         
        }
      }
    } 
    else {
      ROS_WARN("mtsp: Failed to call ik_service!!");
    }  

  }
  for (size_t i = 0; i < joint_trajectory.points.size(); ++i) {
    ROS_INFO("Waypoint %zu: time_from_start = %.2f", i, joint_trajectory.points[i].time_from_start.toSec());
  }

  return joint_trajectory;
}


// L7- Action Server Client - Use FollowJointTrajectoryAction which defines the full action interface, goal, feedback, result structures
// THis defines the server-client comminication and mid-level interaction
void actionServerImplementation(trajectory_msgs::JointTrajectory& joint_trajectory){
  // Instantiate the Action Server client
  // The "true" at the end of the instantiation causes a separate thread to be
  // spun for the client. There is some analogy to the AsyncSpinner here.
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac("ariac/arm1/arm/follow_joint_trajectory", true);

  ROS_INFO("%sWaiting Action Server...",BLUE_TEXT);
  trajectory_ac.waitForServer();
  ROS_INFO("%sAction server started, sending goal.",BLUE_TEXT);

  // Create the structure for the trajectory to populate for running the Action Server.
  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

  // It is possible to reuse the JointTrajectory from above as an element in the goal
  // Create the goal structure for the action server
  control_msgs::FollowJointTrajectoryGoal goal;

  // Populate the goal with the given trajectory
  goal.trajectory = joint_trajectory;

  // Send the goal to the action server
  trajectory_ac.sendGoal(goal,
                         &resultCallback,  // Callback when the goal is finished
                         &goalActiveCallback,  // Callback when the goal becomes active
                         &feedbackCallback);  // Callback for feedback

  // Check the state of the action server
  actionlib::SimpleClientGoalState goal_state = trajectory_ac.getState();

  // Log the state
  ROS_INFO("%sGoal sent, waiting for result...", BLUE_TEXT);

    // Wait for the result (block until the action is done)
    bool finished_before_timeout = trajectory_ac.waitForResult(ros::Duration(120.0)); // Wait for 60 seconds

    if (finished_before_timeout) {
        // Check the state of the action server
        actionlib::SimpleClientGoalState state = trajectory_ac.getState();
        ROS_INFO("%sAction completed with state: [%s]", BLUE_TEXT, state.toString().c_str());

        // If the action was successful
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("%sTrajectory execution succeeded.", BLUE_TEXT);
             joint_trajectory.points.clear();
        }
        else if(state == actionlib::SimpleClientGoalState::ABORTED){
           found_product_pose.erase(found_product_pose.begin());
           joint_trajectory.points.clear();
            ROS_INFO("%sTrajectory execution aborted probably for the collision with the camera.", BLUE_TEXT);
            trajectory_ac.waitForServer();
            ROS_INFO("Action server is ready for new goals.");
        } 
        else {
            ROS_WARN("%sTrajectory execution failed with state: [%s]", BLUE_TEXT, state.toString().c_str());
        }
    } else {
        ROS_WARN("%sAction did not finish before the timeout.", BLUE_TEXT);
        trajectory_ac.cancelGoal(); // Optionally cancel the goal
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_competition_node");
  
  std_srvs::SetBool my_bool_var;
  my_bool_var.request.data = true;
  
  // "n" is the previously declared node handle.
  ros::NodeHandle n;

  // Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // clear the order vector
  order_vector.clear();

  //clear the camera vectors
  logical_cameras_bin.clear();
  logical_cameras_agv.clear();
  logical_cameras_quality.clear();

  //L6- Get the names of the joints being used.
  joint_names.clear();
  

  // clear the found product pose 
  found_product_pose.clear();

  // resize the camera vectors
  logical_cameras_bin.resize(6);
  logical_cameras_agv.resize(2);
  logical_cameras_quality.resize(2);
  // subscribe to ariac/orders topic
  ros::Subscriber orders_subscriber = n.subscribe("/ariac/orders", 1000, orderCallback);
  
  //subscribe to all logical_camera topics. 
  ros::Subscriber logical_camera_subscriber_agv1 = n.subscribe("/ariac/logical_camera_agv1", 10, logicagv1CameraCallback);
  ros::Subscriber logical_camera_subscriber_agv2 = n.subscribe("/ariac/logical_camera_agv2", 10, logicagv2CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin1 = n.subscribe("/ariac/logical_camera_bin1", 10, logicbin1CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin2 = n.subscribe("/ariac/logical_camera_bin2", 10, logicbin2CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin3 = n.subscribe("/ariac/logical_camera_bin3", 10, logicbin3CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin4 = n.subscribe("/ariac/logical_camera_bin4", 10, logicbin4CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin5 = n.subscribe("/ariac/logical_camera_bin5", 10, logicbin5CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin6 = n.subscribe("/ariac/logical_camera_bin6", 10, logicbin6CameraCallback);
  ros::Subscriber logical_camera_subscriber_quality_control_sensor1 = n.subscribe("/ariac/quality_control_sensor_1", 10, logicQuality1CameraCallback);
  ros::Subscriber logical_camera_subscriber_quality_control_sensor2 = n.subscribe("/ariac/quality_control_sensor_2", 10, logicQuality2CameraCallback);


  // L6- Wait to make sure the service is available.
  while(!ros::service::waitForService("/ariac/start_competition", 3000)){
    ROS_WARN("/ariac/start_competition service is not available");
  }
  ROS_WARN("/ariac/start_competitionservice is available now.");

  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");


  // L6-Wait to make sure the service is available.
  if(!ros::service::waitForService("pose_ik", 3000)){
    ROS_WARN("pose_ik service is not available");
  }
  ROS_WARN("pose_ik service is available now.");
  // Create client for pose_ik service
  ros::ServiceClient pose_ik_client = n.serviceClient<ik_service::PoseIK>("pose_ik");
  
  // Create service client in order to request the bin location of a specific material type 
  ros::ServiceClient request_bin_location = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  // L6- Get the arm joint names
  ros::param::get("/ariac/arm1/arm/joints", joint_names);

  //L6- Create a subscriber to receive the state of the joints of the robot.
  ros::Subscriber joint_states_h = n.subscribe("ariac/arm1/joint_states", 50, jointStatesCallback);

  //L6- Create a publisher for the joint trajectory command topic
  joint_trajectory_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

  // Variable to capture service call success.
  int service_call_succeeded;

  // Call the Service
  service_call_succeeded = begin_client.call(begin_comp);

  
    if (service_call_succeeded == 0){
	    ROS_ERROR("Competition service call failed! Goodnes Gracious!!");
    }
    else{
	  if(begin_comp.response.success)
    {
      ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
    }
    else{
	      ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
      }
    }

  // Let ROS handle incoming messages 
  ros::Duration(1.0).sleep(); 

  // Lab 6- Asynchronous spinner is added 
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start(); // A spinner makes calling ros::spin() unnecessary

  //store original joint traj
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = {
    "linear_arm_actuator_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };
  

  // In order to iterate over the orders 
  while (ros::ok()) {
    // If the order_vector has a sıze greater than zero
    if (order_vector.size() > 0) {
      // get the current order from the vector
      osrf_gear::Order &curr_order = order_vector.back();
            
      // print order ID info to check in the output
      ROS_INFO("Order ID: %s", curr_order.order_id.c_str());

      original_joint_trajectory_point.positions.resize(joint_trajectory.joint_names.size());
  for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
    for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
      if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
          original_joint_trajectory_point.positions[t_joint] = joint_states.position[s_joint];
          break;
          }
      }
  }


    // Iterate over each shipment in the order
    while (curr_order.shipments.size() > 0) {
      // get the current shipment
      osrf_gear::Shipment &curr_shipment = curr_order.shipments.back();
      // print the shipment type and agv_id info
      ROS_INFO("Shipment Type: %s, agv_id: %s", curr_shipment.shipment_type.c_str(), curr_shipment.agv_id.c_str());
                
      // Iterate over each product in the shipment
      while(curr_shipment.products.size() >0 ){
        // get the current product
       osrf_gear::Product &product = curr_shipment.products.back();
        ROS_INFO("-----------------PRODUCT----------------------");
        // print the type of the product and its pose
			  ROS_INFO("Product Type: %s", product.type.c_str());
			
				 
        // Now, according to the product type call the material_location service to find where that product type may found
        // Use GetMaterialLocations for that
        // It will return the storage_unit vector where the material may be found
        // Declare local get material location obj
        osrf_gear::GetMaterialLocations get_mat_location;
			
        // requested material type should be equal to the product type
        get_mat_location.request.material_type = product.type;
        
        // call the bin location service
        int bin_location_request_success = request_bin_location.call(get_mat_location);
        
        if(bin_location_request_success == 0){
          ROS_ERROR("Bin location service call  for this material failed! Goodnes Gracious!!");
        }
			  else{
				  // get the all storage units that may contain the specificed material type
				  for (const auto& storage_unit : get_mat_location.response.storage_units) {
					  // Print product type and its corresponding storage unit
					  ROS_INFO(" MATERIAL_LOCATION RESULTS: Product type: %s is in storage unit: %s", product.type.c_str(), storage_unit.unit_id.c_str());
					  // now, search through the logical_camera data for the bin specified by the material_location service for a product of the correct type 
					  if( storage_unit.unit_id != "belt"){
					    // get camera data 
						  const osrf_gear::LogicalCameraImage* camera_data = getCameraData(storage_unit.unit_id, logical_cameras_bin, logical_cameras_agv, logical_cameras_quality);
					 	  bool found_product = false;
						  for (const auto& model : camera_data->models) {
							  // Check if the model type matprint the pose of this tooches the product type we're looking for
							  ROS_INFO("Search through the logical_camera data for the bin specified by the material_location service");
							  if (model.type == product.type) {
							    // Log the product type, bin, and pose information
							    ROS_INFO_STREAM("Found product: " << product.type.c_str() 
									    << " in bin: " << storage_unit.unit_id.c_str() 
									    << " at position (x: " << model.pose.position.x 
									    << ", y: " << model.pose.position.y 
									    << ", z: " << model.pose.position.z 
									    << ") with orientation (x: " << model.pose.orientation.x 
									    << ", y: " << model.pose.orientation.y 
									    << ", z: " << model.pose.orientation.z 
									    << ", w: " << model.pose.orientation.w << ")");
							    
							    found_product = true;
                  // then let's store the pose of the found product so we can apply transformations on it later
                  found_product_pose.push_back(model.pose);
							    //break; // Stop searching models if we found the product
							  }  
					    }
					    if (!found_product) {
						    ROS_WARN_STREAM("Product type " << product.type.c_str() << " not found in bin: " << storage_unit.unit_id.c_str());
					    }
            
                 double prev_goal_time = 0;
                // if there is any found product, apply transformations
                while(found_product_pose.size() > 0){

                   // get camera frame name in order to transform
                  std::string camera_frame_name = "logical_camera_"+storage_unit.unit_id+"_frame";
                  // get bin frame name in order to transform
                  std::string bin_frame_name = storage_unit.unit_id+ "_frame";

                  ROS_INFO("Camera frame name is %s", camera_frame_name.c_str());
                
                  geometry_msgs::PoseStamped part_pose, goal_pose;
                  //part_pose.pose = found_product_pose.back();
                  part_pose.pose = found_product_pose[found_product_index];

                  ROS_INFO("Location of the part in refernce frame of the logical camera: Position -> x: %.2f, y: %.2f, z: %.2f, Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                          part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z,
                          part_pose.pose.orientation.x, part_pose.pose.orientation.y,
                          part_pose.pose.orientation.z, part_pose.pose.orientation.w);

                  // Retrieve the transformation
                  geometry_msgs::TransformStamped tfStamped;
                  try {
                      tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_name.c_str(),
                      ros::Time(0.0), ros::Duration(10.0));
                      ROS_INFO("Transform from [%s] to [%s]: Translation before robot moves in front of the bin -> x=%.2f, y=%.2f, z=%.2f, Rotation -> x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                      camera_frame_name.c_str(), "arm1_base_link",
                      tfStamped.transform.translation.x, tfStamped.transform.translation.y, tfStamped.transform.translation.z,
                      tfStamped.transform.rotation.x, tfStamped.transform.rotation.y, tfStamped.transform.rotation.z, tfStamped.transform.rotation.w);



                      // do the transfromation so the pose of the part with respect to the robot is known
                      tf2::doTransform(part_pose, goal_pose, tfStamped);

                      // Print the transformed goal pose
                      ROS_INFO("Location of the part in the reference frame of the robot arm1_base_link before robot moves in front of the bin: Position -> x: %.2f, y: %.2f, z: %.2f, Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                              goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
                              goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                              goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);


                        //============================================================
                        // This part transforms the camera position of the parts into the world frame. So, we can track which part it is reaching for.
                        geometry_msgs::TransformStamped tfStamped_specific;
                        tfStamped_specific = tfBuffer.lookupTransform("world", camera_frame_name.c_str(), ros::Time(0.0), ros::Duration(1.0));
                        // Create variables
                        geometry_msgs::PoseStamped robot_frame_pose, workcell_frame_pose;
                        robot_frame_pose.pose = part_pose.pose;
                        tf2::doTransform(robot_frame_pose, workcell_frame_pose, tfStamped_specific);
                        // Log transformed pose in robot frame
                        ROS_INFO("%sLogic Frame Pose:\n"
                                "  Position: (%.2f, %.2f, %.2f)\n"
                                "  Orientation: (x: %.2f, y: %.2f, z: %.2f, w: %.2f)", GREEN_TEXT,
                                robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y, robot_frame_pose.pose.position.z,
                                robot_frame_pose.pose.orientation.x, robot_frame_pose.pose.orientation.y, 
                                robot_frame_pose.pose.orientation.z, robot_frame_pose.pose.orientation.w);
                        tf2::Quaternion quat_workcell(workcell_frame_pose.pose.orientation.x,
                                                      workcell_frame_pose.pose.orientation.y,
                                                      workcell_frame_pose.pose.orientation.z,
                                                      workcell_frame_pose.pose.orientation.w);
                        double roll_workcell, pitch_workcell, yaw_workcell;
                        tf2::Matrix3x3(quat_workcell).getRPY(roll_workcell, pitch_workcell, yaw_workcell);
                        // Print position and orientation in world frame
                        ROS_INFO("%sWorld Frame Pose:\n"
                                "  Position: (%.2f, %.2f, %.2f)\n"
                                "  Orientation (RPY): Roll: %.2f, Pitch: %.2f, Yaw: %.2f", GREEN_TEXT,
                                workcell_frame_pose.pose.position.x, workcell_frame_pose.pose.position.y, workcell_frame_pose.pose.position.z,
                                roll_workcell, pitch_workcell, yaw_workcell);

                        //============================================================

                      // before finding the inverse kinematics of the part, need to move the linear_arm actuator in front of the bin
                      // therefore create joint_trajectory for it first
                      // find the place of the bin by using the pose of the product
                      geometry_msgs::PoseStamped in_front_of_bin_pose = goal_pose;
                      //in_front_of_bin_pose.pose.position.y += 0.25;
                      moveInFrontOfBin(in_front_of_bin_pose.pose.position.y);

              
                      // perform the transformation after the robot moved in front of the bin
                      tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_name.c_str(),
                      ros::Time(0.0), ros::Duration(10.0));
                      ROS_INFO("Transform from [%s] to [%s]: Translation after the robot moves in front of the bin -> x=%.2f, y=%.2f, z=%.2f, Rotation -> x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                      camera_frame_name.c_str(), "arm1_base_link",
                      tfStamped.transform.translation.x, tfStamped.transform.translation.y, tfStamped.transform.translation.z,
                      tfStamped.transform.rotation.x, tfStamped.transform.rotation.y, tfStamped.transform.rotation.z, tfStamped.transform.rotation.w);

                      // do the transfromation so the pose of the part with respect to the robot is known
                      tf2::doTransform(part_pose, goal_pose, tfStamped);

                      // Add some waypoints
                      geometry_msgs::PoseStamped approach_pose_5, approach_pose_10, approach_pose_5_goal, approach_pose_10_goal;
                      // create two waypoints that are gradually far away from the goal point
                      approach_pose_10_goal = goal_pose;
                      approach_pose_5_goal = goal_pose;

                      // gradual increment
                      approach_pose_10_goal.pose.position.z += 0.3; // 30 cm above the part
                      if(part_pose.pose.position.y <  0){
                        approach_pose_10_goal.pose.position.y -= 0.3; // take the trajectory wider so do not collide with the camera
                      }
                      else{
                        approach_pose_10_goal.pose.position.y += 0.5; // take the trajectory wider so do not collide with the camera
                      }
                      approach_pose_5_goal.pose.position.z += 0.10; // 10 cm above the part
                      goal_pose.pose.position.z += 0.05; // 5 cm above the part
                      

                      // Print the transformed goal pose
                      ROS_INFO("Location of the part in the reference frame of the robot arm1_base_link after the robot moves in front of the bin: Position -> x: %.2f, y: %.2f, z: %.2f, Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                              goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
                              goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                              goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
                      //Set the request field of the ik_service::PoseIK variable equal to the goal pose
                      ik_pose.request.part_pose = goal_pose.pose;

                      // Update the client.call() to use the ik_service::PoseIK variable.
                      bool success = pose_ik_client.call(ik_pose);
                      if (success)
                      {
                      // Finally, update the ROS_INFO() messages to indicate that the client.call() returned request.num_sols solutions
                        //ROS_INFO("Call to ik_service returned [%i] solutions", ik_pose.response.num_sols);
                        int solutionNum = chooseSolution(ik_pose.response,part_pose);
                      if(part_pose.pose.position.y >  0){
                        solutionNum = 2;
                      }
                        // Set and publish the joint trajectory
                        ROS_INFO("1-Chosen Solution Num : %d ", solutionNum);
                        // print the chosen solution
                        for(int j = 0 ; j < 6 ; j++){
                          ROS_INFO("Joint angle [%d]: %.4f ", j+1, ik_pose.response.joint_solutions[solutionNum].joint_angles[j]);
                        }
                        // Variable for the trajectory
                        
                        // create joint_angles variable from the response of the ik_pose response
                        std::vector<double> joint_angles(ik_pose.response.joint_solutions[solutionNum].joint_angles.begin(),
                                           ik_pose.response.joint_solutions[solutionNum].joint_angles.end());

                        // create joint trajectory for the goal points
                        trajectory_msgs::JointTrajectoryPoint goal_point;
                        goal_point.positions.resize(joint_trajectory.joint_names.size());
                        goal_point.positions[0] = in_front_of_bin_pose.pose.position.y; //0.00001+0.10;
                        for(int j = 0; j < joint_angles.size(); j++){
                          goal_point.positions[j+1] = joint_angles[j];
                        }
                      

                        //Set the request field of the ik_service::PoseIK variable equal to the goal pose
                         ik_pose.request.part_pose = approach_pose_5_goal.pose;

                        // Update the client.call() to use the ik_service::PoseIK variable.
                        bool success_5 = pose_ik_client.call(ik_pose);
                        solutionNum = chooseSolution(ik_pose.response,part_pose);
                      if(part_pose.pose.position.y >  0){
                        solutionNum = 2;
                      }
                        // Set and publish the joint trajectory
                        ROS_INFO("2-Chosen Solution Num : %d ", solutionNum);
                        // print the chosen solution
                        for(int j = 0 ; j < 6 ; j++){
                          ROS_INFO("2-Joint angle [%d]: %.4f ", j+1, ik_pose.response.joint_solutions[solutionNum].joint_angles[j]);
                        }
                         // create joint_angles variable from the response of the ik_pose response
                        std::vector<double> joint_angles_5(ik_pose.response.joint_solutions[solutionNum].joint_angles.begin(),
                                           ik_pose.response.joint_solutions[solutionNum].joint_angles.end());

                        // create joint trajectory for the goal points
                        trajectory_msgs::JointTrajectoryPoint approach_point_5;
                        approach_point_5.positions.resize(joint_trajectory.joint_names.size());
                        approach_point_5.positions[0] = in_front_of_bin_pose.pose.position.y; //0.00001+0.10;
                        for(int j = 0; j < joint_angles_5.size(); j++){
                          approach_point_5.positions[j+1] = joint_angles_5[j];
                        }
                        

                        //Set the request field of the ik_service::PoseIK variable equal to the goal pose
                        ik_pose.request.part_pose = approach_pose_10_goal.pose;

                        // Update the client.call() to use the ik_service::PoseIK variable.
                        bool success_10 = pose_ik_client.call(ik_pose);
                        solutionNum = chooseSolution(ik_pose.response,part_pose);
                      if(part_pose.pose.position.y >  0){
                        solutionNum = 2;
                      }
                        // Set and publish the joint trajectory
                        ROS_INFO("3-Chosen Solution Num : %d ", solutionNum);
                        // print the chosen solution
                        for(int j = 0 ; j < 6 ; j++){
                          ROS_INFO("3-Joint angle [%d]: %.4f ", j+1, ik_pose.response.joint_solutions[solutionNum].joint_angles[j]);
                        }
                         // create joint_angles variable from the response of the ik_pose response
                        std::vector<double> joint_angles_10(ik_pose.response.joint_solutions[solutionNum].joint_angles.begin(),
                                           ik_pose.response.joint_solutions[solutionNum].joint_angles.end());

                        // create joint trajectory for the goal points
                        trajectory_msgs::JointTrajectoryPoint approach_point_10;
                        approach_point_10.positions.resize(joint_trajectory.joint_names.size());
                        approach_point_10.positions[0] = in_front_of_bin_pose.pose.position.y; //0.00001+0.10;
                        for(int j = 0; j < joint_angles_10.size(); j++){
                          approach_point_10.positions[j+1] = joint_angles_10[j];
                        }
                       

                        // create joint trajectory for the start points
                        trajectory_msgs::JointTrajectoryPoint start_point;
                        start_point.positions.resize(joint_trajectory.joint_names.size());
                        for (int t_joint = 0; t_joint < joint_trajectory.joint_names.size(); t_joint++) {
                          for (int s_joint = 0; s_joint < joint_states.name.size(); s_joint++) {
                            if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
                              start_point.positions[t_joint] = joint_states.position[s_joint];
                              break;
                            }
                          }
                        }

                        // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
                        //prev_goal_time = prev_goal_time + 20.0;

                        start_point.time_from_start = ros::Duration(prev_goal_time + 3.0);
                        joint_trajectory.points.push_back(start_point);


                        approach_point_10.time_from_start = ros::Duration(prev_goal_time + 7.0);
                        joint_trajectory.points.push_back(approach_point_10);
                        

                        approach_point_5.time_from_start = ros::Duration(prev_goal_time + 8);
                        joint_trajectory.points.push_back(approach_point_5); 


                        goal_point.time_from_start = ros::Duration(prev_goal_time + 9);
                        joint_trajectory.points.push_back(goal_point);

                        approach_point_5.time_from_start = ros::Duration(prev_goal_time + 10);
                        joint_trajectory.points.push_back(approach_point_5); 

                        approach_point_10.time_from_start = ros::Duration(prev_goal_time + 11.0);
                        joint_trajectory.points.push_back(approach_point_10);

                        original_joint_trajectory_point.time_from_start = ros::Duration(prev_goal_time + 17);
                        joint_trajectory.points.push_back(original_joint_trajectory_point);

                        actionServerImplementation(joint_trajectory);
                         ROS_INFO("Sleeping for 20 seconds...");
                          ros::Duration(7.0).sleep();
                          ROS_INFO("Awake now!");

                      //Remove the processed product from the queue when its is completd
                      found_product_pose.erase(found_product_pose.begin());

                      
                      }
                      else
                      {
                        if(!success){
                          //Remove the processed product from the queue when its is completd
                          //found_product_pose.pop_back();
                          found_product_pose.erase(found_product_pose.begin());
                          // Finally, update the ROS_INFO() messages to indicate that the client.call() failed
                            ROS_INFO("ik_service could not find a solution for this transformation: Transformed goal pose: Position -> x: %.2f, y: %.2f, z: %.2f, Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                              goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
                              goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                              goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
                        }
                        else{
                          //Remove the processed product from the queue when its is completd
                          found_product_pose.erase(found_product_pose.begin());
                          //found_product_pose.pop_back();
                          // Finally, update the ROS_INFO() messages to indicate that the client.call() failed
                            ROS_ERROR("Failed to call service ik_service");

                        }
                        
                      }

                    } catch (tf2::TransformException &ex) {
                      ROS_ERROR("%s", ex.what());
                    }
                }
					  }
					
				  }	
				
			    }
			    //Remove the processed product from the queue when its is completd
			    curr_shipment.products.pop_back();
			    ROS_INFO("-------------------------------------------");
        }

        // Remove the processed shipment from the queue when it is completd
        curr_order.shipments.pop_back();
      }

      // Remove the processed order from the queue when it is completed
      order_vector.pop_back(); 
    } 
  else 
  {
    // Check for new orders every second
    ros::Duration(1.0).sleep(); 
  } 
  // LAB 6 PRINTABLE OUTCOMES
  //  ROS_INFO("%s===========================LAB6=============================================",GREEN_TEXT);
  //  setAndPublishJointTrajectory();
  //  moveToSpecificPoints(ik_pose,pose_ik_client,2);
  //  ROS_INFO("%s============================================================================",GREEN_TEXT);

  // LAB 7 PRINTABLE OUTCOMES
  //  ROS_INFO("%s===========================LAB7=============================================",BLUE_TEXT);
  //   trajectory_msgs::JointTrajectory joint_trajectory = returnJointTrajectory(ik_pose, pose_ik_client, 2);
  //   actionServerImplementation(joint_trajectory);
  //  ROS_INFO("%s============================================================================",BLUE_TEXT);
  }
  // ROS spin is removed in Lab 6
  //ros::spin(); 
  return 0;
}
