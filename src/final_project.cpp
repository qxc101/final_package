//Include statements
#include <algorithm>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

//Action server
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
// The Action Server “message type”
#include "control_msgs/FollowJointTrajectoryAction.h"

// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h" //header for retrieving current state of robot joints
#include "trajectory_msgs/JointTrajectory.h" //header for generating a joint trajectory


class Final{
    public:
        ros::Publisher joint_trajectory_publisher_;
        ros::ServiceClient enable_gripper;
        trajectory_msgs::JointTrajectory joint_trajectory_msg;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_as = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("ariac/arm/follow_joint_trajectory", true);
        
        geometry_msgs::Pose agv1_position; //So that we can access the position of the tray agv1
        
        // Create the structure to populate for running the Action Server.
        control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
        
        explicit Final(ros::NodeHandle & n, tf2_ros::Buffer & tfBuffer) : has_been_zeroed_(false) {
            joint_trajectory_publisher_ = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
            order_received.resize(0);
            
            // Fill the names of the joints to be controlled.
            // Note that the vacuum_gripper_joint is not controllable.
            joint_trajectory_msg.joint_names.clear();
            joint_trajectory_msg.header.frame_id = "/world";
            joint_trajectory_msg.joint_names.push_back("elbow_joint");
            joint_trajectory_msg.joint_names.push_back("linear_arm_actuator_joint");
            joint_trajectory_msg.joint_names.push_back("shoulder_lift_joint");
            joint_trajectory_msg.joint_names.push_back("shoulder_pan_joint");
            joint_trajectory_msg.joint_names.push_back("wrist_1_joint");
            joint_trajectory_msg.joint_names.push_back("wrist_2_joint");
            joint_trajectory_msg.joint_names.push_back("wrist_3_joint");
            
            //get the gripper service handle
            enable_gripper = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
            if (!enable_gripper.exists()) {
                enable_gripper.waitForExistence();
            }
            
            this->tfBuffer = &tfBuffer;
        }
        
        /// Called when a new message is received.
        void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
            
            competition_state_ = msg->data;
          }
          
        /// Called when a new JointState message is received.
        void joint_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
          {
            joint_states = *joint_state_msg;
            exist_joint_states = true;
        }
        
        bool has_joint_states() {
            return exist_joint_states;
        }
        
        
        
        void move_to_position(int num_positions, double x[], double y[], double z[], ros::Duration durs[]) {
            //move to position in the base_link frame.
            double send_array[num_positions][8][6];
            // ROS_INFO_STREAM("Size of send_array" << sizeof(send_array));
            
            for (int i = 0; i < num_positions; i++) {
                double T_des[4][4];
                double q_des[8][6];
                
                // Desired pose of the end effector wrt the base_link.
                T_des[0][3] = x[i];
                T_des[1][3] = y[i];
                T_des[2][3] = z[i];
                T_des[3][3] = 1.0;

                // The orientation of the end effector so that the end effector is down.
                T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
                T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
                T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
                T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

                //set the final position for the trajectory to be the desired position
                //find the destination joint angles
                int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
                if(num_sols == 0){
                    ROS_WARN("No solutions found!!");
                    return;
                }else{
                    // ROS_INFO("Solutions calculated!");
                }

                memcpy(send_array+i, q_des, sizeof(q_des));
            }
            set_trajectory(num_positions, send_array, durs);
            
            // Publish the specified trajectory.
            this->send_trajectory_msg_();
        }

          void order_action(){
         osrf_gear::VacuumGripperControl gripper;
            gripper.request.enable = false;
            enable_gripper.call(gripper);
            
            
            for (int i = 0; i < order_received.size(); i++) {
                osrf_gear::OrderPtr current_Order;
                current_Order = order_received[i]; // order_received has type osrf_gear::OrderPtr
                //ROS_INFO_STREAM("Current order is:" << current_Order);
                for (int j = 0; j < current_Order->kits.size(); j++) {
                    osrf_gear::Kit current_Kit;
                    current_Kit = current_Order->kits[j];
                    
                    for (int l = 0; l < current_Kit.objects.size(); l++) {
                        osrf_gear::KitObject current_Object;
                        current_Object = current_Kit.objects[l];
                        
                        for (int m = 0; m < image_from_cam->models.size(); m++) {
                            if (current_Object.type == image_from_cam->models[m].type) {
                                //grab part
                                int current_model_ = (current_model_+1)%image_from_cam->models.size();
                                    //Get part position in base coordinates
                                osrf_gear::Model current_model = image_from_cam->models[current_model_];
                                geometry_msgs::PoseStamped part_pose;
                                geometry_msgs::PoseStamped goal_pose;
                                part_pose.pose = current_model.pose;
                                get_transform(&part_pose, &goal_pose, "base_link");
                                double x = goal_pose.pose.position.x;
                                double y = goal_pose.pose.position.y;
                                double z = goal_pose.pose.position.z;
                                attach(x, y, z);
                                
                                // move to bin
                                    //get bin position in baselink frame
                                geometry_msgs::PoseStamped agv1_pose;
                                geometry_msgs::PoseStamped agv1_goal_pose;
                                get_transform(&agv1_pose, &agv1_goal_pose, "base_link");
                                double bin_x[] = {agv1_goal_pose.pose.position.x};
                                double bin_y[] = {agv1_goal_pose.pose.position.y};
                                double bin_z[] = {agv1_goal_pose.pose.position.z};
                                ros::Duration time[] = {ros::Duration(3.0)};
                                move_to_position(1, bin_x, bin_y, bin_z, time);
                                //It's trying to REACH for the agv1 bin and then giving up without moving the linear arm actuator
                                    //why is move to position function not working?
                                //Building a trajectory message:
                                
                                
                                // drop part
                                detach(x, y, z);
                                // move to home
                                
                                // set flag to true for this object in kit
                            }
                        }
                    }
                }
            }
}
        
        void set_trajectory(int num_positions, double q_des[][8][6], ros::Duration durs[]) {
            int q_des_indx = 0;
            
            //setup start and end points
            joint_trajectory_msg.points.resize(num_positions);
            
            //resize all position vectors to the length of our number of joints DOF
            for (int i = 0; i < joint_trajectory_msg.points.size(); i++) {
                joint_trajectory_msg.points[i].positions.resize(joint_trajectory_msg.joint_names.size());
            }
        
            // The actuators are commanded in an odd order, enter the joint positions in the correct positions
            int shoulder_pan_index, shoulder_lift_index, elbow_index, wrist_1_index, wrist_2_index, wrist_3_index, linear_actuator_index, vacuum_gripper_index;
            for (int i = 0; i < joint_trajectory_msg.joint_names.size(); i++) {
                if ("shoulder_pan_joint" == joint_trajectory_msg.joint_names[i])
                    shoulder_pan_index = i;
                else if ("shoulder_lift_joint" == joint_trajectory_msg.joint_names[i])
                    shoulder_lift_index = i;
                else if ("elbow_joint" == joint_trajectory_msg.joint_names[i])
                    elbow_index = i;
                else if ("wrist_1_joint" == joint_trajectory_msg.joint_names[i])
                    wrist_1_index = i;
                else if ("wrist_2_joint" == joint_trajectory_msg.joint_names[i])
                    wrist_2_index = i;
                else if ("wrist_3_joint" == joint_trajectory_msg.joint_names[i])
                    wrist_3_index = i;
                else if ("linear_arm_actuator_joint" == joint_trajectory_msg.joint_names[i])
                    linear_actuator_index = i;
                else if ("vacuum_gripper_joint" == joint_trajectory_msg.joint_names[i])
                    vacuum_gripper_index = i;
                else
                    break;
            }
        
            // ROS_INFO("Building joint messages");
            for (int i = 0; i < num_positions; i++) {                
                //ROS_INFO_STREAM(shoulder_pan_index << shoulder_lift_index << elbow_index << wrist_1_index << wrist_2_index << wrist_3_index << linear_actuator_index << vacuum_gripper_index);
                
                joint_trajectory_msg.points[i].positions[linear_actuator_index] = 0.0;
                //joint_trajectory_msg.points[0].positions[vacuum_gripper_index] = 0.0;
                joint_trajectory_msg.points[i].positions[shoulder_pan_index] = q_des[i][q_des_indx][0];
                joint_trajectory_msg.points[i].positions[shoulder_lift_index] = q_des[i][q_des_indx][1];
                joint_trajectory_msg.points[i].positions[elbow_index] = q_des[i][q_des_indx][2];
                joint_trajectory_msg.points[i].positions[wrist_1_index] = q_des[i][q_des_indx][3];
                joint_trajectory_msg.points[i].positions[wrist_2_index] = q_des[i][q_des_indx][4];
                joint_trajectory_msg.points[i].positions[wrist_3_index] = q_des[i][q_des_indx][5];
                
                // How long to take for the movement.
                joint_trajectory_msg.points[i].time_from_start = durs[i];
            }
        }
        
        void get_transform(geometry_msgs::PoseStamped *part_pose, geometry_msgs::PoseStamped *goal_pose, std::string target_frame) {
            //calculates the part_pose with respect to the target_frame and stores the transformed coordinates in goal_pose
            geometry_msgs::TransformStamped tfStamped;
            try {
                tfStamped = tfBuffer->lookupTransform(target_frame, "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
                
            } 
            catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            
            tf2::doTransform(*part_pose, *goal_pose, tfStamped);
        }
        

        
        
        void attach(double x, double y, double z) {
            //picks up part from coordinates relative to base_link
            ros::Duration sleep(3.0);
                //MAkING IT WAIT A LITTLE BIT LONGER FIXES IT IDK
            //Build an array of ever closer (vertical) points to the part that each take 
            //one second
            double x1[] = {x, x, x};
            double y1[] = {y, y, y};
            double z1[] = {z+0.1, z+0.075, z+0.01};
            ros::Duration time1[] = {ros::Duration(1.0), ros::Duration(2.0), ros::Duration(3.0)};
            move_to_position(3, x1, y1, z1, time1);
            sleep.sleep();
            //grab the object
             osrf_gear::VacuumGripperControl gripper;
            gripper.request.enable = true;
            enable_gripper.call(gripper);
            sleep.sleep();
            
            //move above the object and wait
            double x2[] = {x, x, x};
            double y2[] = {y, y, y};
            double z2[] = {z+0.01, z+0.075, z+0.1};
            ros::Duration time2[] = {ros::Duration(1.0), ros::Duration(2.0), ros::Duration(3.0)};
            move_to_position(3, x2, y2, z2, time2);
            sleep.sleep();
        }
        
        void detach(double x, double y, double z) {
            //Drops the part at the specified XYZ position in base link frame
            ros::Duration sleep(3.0);
            //Build an array of ever closer (vertical) points to the part that each take 
            //one second
            double x1[] = {x, x, x};
            double y1[] = {y, y, y};
            double z1[] = {z+0.1, z+0.075, z+0.025};
        
            //come back to position
            
            ros::Duration time3[] = {ros::Duration(1.0), ros::Duration(2.0), ros::Duration(3.0)};
            move_to_position(3, x1, y1, z1, time3);
            sleep.sleep();
            
            //drop object
            
            osrf_gear::VacuumGripperControl gripper;
            gripper.request.enable = false;
            enable_gripper.call(gripper);       
            sleep.sleep();
            
            //move above the object and wait
            double x2[] = {x, x, x};
            double y2[] = {y, y, y};
            double z2[] = {z+0.025, z+0.075, z+0.1};
            ros::Duration time4[] = {ros::Duration(1.0), ros::Duration(2.0), ros::Duration(3.0)};
            move_to_position(3, x2, y2, z2, time4);
            sleep.sleep();
        }
        
      
        

        void cam_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
            // ROS_INFO_STREAM_THROTTLE(10, "Logical camera: '" << image_msg->models.size() << "' objects.");
            image_from_cam = image_msg;
        }
        
        void agv1_cam_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
            this->agv1_position = image_msg->pose;
        }
        
        void order_callback(const osrf_gear::OrderPtr & order) {
            order_received.push_back(order);
        }
        
    private:
      std::string competition_state_;
      std::vector<osrf_gear::OrderPtr> order_received;
      sensor_msgs::JointState joint_states;
      bool has_been_zeroed_;
      bool exist_joint_states = false;
      osrf_gear::LogicalCameraImage::ConstPtr image_from_cam;
      int current_model_ = 0;
      tf2_ros::Buffer* tfBuffer;
      
      void send_trajectory_msg_() {
        joint_trajectory_msg.header.seq = joint_trajectory_msg.header.seq+1;
        joint_trajectory_msg.header.stamp = ros::Time::now();
        joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory_msg;        
        actionlib::SimpleClientGoalState state = trajectory_as->sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
      }
};            


//Main function
int main(int argc, char ** argv) {
    ros::init(argc, argv, "ariac_example_node");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle n;
    Final final(n, tfBuffer);
    ros::Subscriber comp_state_subscriber = n.subscribe("/ariac/competition_state", 10, &Final::competition_state_callback, &final);
    ros::Subscriber joint_states_subscriber = n.subscribe("/ariac/joint_states", 10, &Final::joint_callback, &final);
    ros::Subscriber cam_subscriber = n.subscribe("/ariac/logical_camera", 10, &Final::cam_callback, &final);
    ros::Subscriber agv1_cam_subscriber = n.subscribe("/ariac/logical_camera_over_agv1", 10, &Final::agv1_cam_callback, &final);
    ros::Subscriber orders_subscriber = n.subscribe("/ariac/orders", 10, &Final::order_callback, &final);
    ros::Rate loop_rate(0.2);
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient start_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists()) {
	start_client.waitForExistence();
    }
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv);  // Call the start Service.
    do {
        ros::spinOnce();
        loop_rate.sleep();
    } while (!final .has_joint_states());
  
    geometry_msgs::PoseStamped part_pose, goal_pose;
    part_pose.pose.position.x = -0.2203;
    part_pose.pose.position.y = -0.1709;
    part_pose.pose.position.z = 1.2335;
    tf2::Quaternion orient;
    orient.setRPY(0.0, 0.0, 1.57);
    orient.normalize();
    geometry_msgs::Quaternion orient_msg;
    tf2::convert(orient_msg, orient);
    part_pose.pose.orientation = orient_msg;
    final.get_transform(&part_pose, &goal_pose, "base_link");

    double x = goal_pose.pose.position.x;
    double y = goal_pose.pose.position.y;
    double z = goal_pose.pose.position.z;
                
    double home_x[] = {x};
    double home_y[] = {y};
    double home_z[] = {z};
    ros::Duration time[] = {ros::Duration(5.0)};
    final.move_to_position(1, home_x, home_y, home_z, time);
    ros::Duration(1.0).sleep();
  
    while (ros::ok()) {      
        ros::spinOnce();
        final.order_action();
            
            
        loop_rate.sleep();
    }

  return 0;
}
