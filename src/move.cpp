#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <random>

double Uniform( void ){
   return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

int main(int argc, char **argv){
        double xlimit[2] = {120, 240};
        double ylimit[2] = {-120, 120};
        double thlimit[2] = {-M_PI+1.0e-10, M_PI-1.0e-10};

        double x = Uniform() * (xlimit[1] - xlimit[0]) + xlimit[0];
        double y = Uniform() * (ylimit[1] - ylimit[0]) + ylimit[0];
        double th = Uniform() * (thlimit[1] - thlimit[0]) + thlimit[0];
        x /= 1000.0;
        y /= 1000.0;

        double z = 40 / 1000.0;
        double z_min = 10 / 1000.0;

        double s2 = sin(th);
        double c2 = cos(th);

        ros::init(argc, argv, "pickandplacer");
        ros::NodeHandle nh;

        ros::AsyncSpinner spinner(2);
        spinner.start();

        // moveit::planning_interface::MoveGroupInterface arm("arm");
        moveit::planning_interface::MoveGroupInterface arm("arm");
        arm.setPoseReferenceFrame("base_link");

          actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
              "/crane_plus_gripper/gripper_command",
              "true");
          gripper.waitForServer();
          
        // arm.setNamedTarget("rest_pose"); //or zero_pose
        // arm.move();

        // ROS_INFO("Moving to prepare pose");
        // arm.setNamedTarget("rest_pose"); // zero_pose, rest_pose
        // if (!arm.move()) {
        //         ROS_WARN("Could not move to prepare pose");
        //         return 1;
        // }

          // Open gripper
          ROS_INFO("Opening gripper");
          control_msgs::GripperCommandGoal goal;
          goal.command.position = 0.1;
          gripper.sendGoal(goal);
          bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
          if (!finishedBeforeTimeout) {
            ROS_WARN("Gripper open action did not complete");
            return 1;
          }

        ROS_INFO("Moving to next pose");
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = 0.2;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.1;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.707106;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.707106;

        arm.setPoseTarget(pose);
        if (!arm.move()) {
                ROS_WARN("Could not move to prepare pose");
                return 1;
        }


        // std::cout << "Reach (" << x << ", " << y << ", " << z << ")" << std::endl;
        // geometry_msgs::PoseStamped pose;
        // pose.header.frame_id = "base_link";
        // //pose.header.frame_id = "panda_link0";
        // pose.pose.position.x = x;
        // pose.pose.position.y = y;
        // pose.pose.position.z = z;
        // // pose.pose.orientation.x = 0.0;
        // // pose.pose.orientation.y = 0.707106;
        // // pose.pose.orientation.z = 0.0;
        // // pose.pose.orientation.w = 0.707106;
        // arm.setPoseTarget(pose);
        // if (!arm.move()) {
        //         ROS_WARN("Could not move to prepare pose");
        //         return 1;
        // }
        //
        // std::cout << "[ArmImageGenerator] Down" << std::endl;
        // pose.pose.position.z = z_min;
        // arm.setPoseTarget(pose);
        // if (!arm.move()) {
        //         ROS_WARN("Could not move to prepare pose");
        //         return 1;
        // }
        //
        // std::cout << "[ArmImageGenerator] Release" << std::endl;
        //
        //
        // std::cout << "[ArmImageGenerator] Escape" << std::endl;
        // arm.setNamedTarget("rest_pose"); // zero_pose, rest_pose
        // if (!arm.move()) {
        //         ROS_WARN("Could not move to prepare pose");
        //         return 1;
        // }
        //
        // std::cout << "[ArmImageGenerator] Waiting for CameraImage...." << std::ends;


        ros::shutdown();
        return 0;
}
