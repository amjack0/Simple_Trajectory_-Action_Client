#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <jointspace/OptStates.h>
#include <array>

#define N_JOINT 6


typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> TrajClient;


class Ur3Arm
{
private:

  // Action client for the follow joint trajectory action used to trigger the arm movement action
  TrajClient* traj_client_;
public:

  int ind = 0; //! always zero
  float angle[N_JOINT]; float velocity[N_JOINT]; float acc[N_JOINT];
  ros::NodeHandle n;
  ros::Subscriber my_states_sub ; bool completed_2;
  // Initialize the action client and wait for action server to come up.
  Ur3Arm()
  {
    //! Tell the action client that we want to spin a thread by default:
    traj_client_ = new TrajClient("/arm_controller/follow_joint_trajectory", true);
    my_states_sub = n.subscribe("/opt_states", 1000, &Ur3Arm::state_callback, this);
    // Wait for the action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("[ST] Wait for the follow_joint_trajectory_action server");
    }
  }

  // Clean up the action client
  // -> Destructor of class Ur3Arm

  ~Ur3Arm()
  {
    delete traj_client_;
  }

  // Send the command to start a given trajectory

  bool startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0)
    traj_client_->sendGoalAndWait(goal, ros::Duration(0,0), ros::Duration(0,0)); //!  wait forever untill goal is finished
    return true;
  }

  void state_callback(const jointspace::OptStates::ConstPtr& msg){

    for (short int i=0; i< N_JOINT; i++){
      angle[i] = msg->q.data[i];
      velocity[i] = msg->qdot.data[i];
      acc[i] = msg->qddot.data[i];

      /*std::cout << "velocity" << "[" << i <<"]: " << velocity[i] << std::endl;
      std::cout << "Vel" << "[" << i <<"]: " << velocity[i] << std::endl;
      std::cout << "Acc" << "[" << i <<"]: " << acc[i] << std::endl;*/
    }
  }

  // Generates a simple trajectory with two waypoints, used as an example
  /* Note that this trajectory contains two waypoints, joined together as a single trajectory.
           Alternatively, each of these waypoints could be in its own trajectory - a trajectory can have one
           or more waypoints depending on the desired application.
        */

  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    // The goal variable:
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    //goal.trajectory.joint_names.resize(1);
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");

    //int ind = 0; // Represents the number of the actual waypoint

    // In this simple case there asre only two waypoints in this goal trajectory
    goal.trajectory.points.resize(1); // important

    // First trajectory waypoint

    goal.trajectory.points[ind].positions.resize(N_JOINT);
    goal.trajectory.points[ind].velocities.resize(N_JOINT);
    goal.trajectory.points[ind].accelerations.resize(N_JOINT);

    for(size_t i = 0; i < N_JOINT; ++i)
    {
      goal.trajectory.points[ind].positions[i] = angle[i];
      goal.trajectory.points[ind].velocities[i] = velocity[i];
      goal.trajectory.points[ind].accelerations[i] = acc[i];
    }

    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0); //! To be reached 1 second after starting along the trajectory
    return goal;      // Function done, return the goal
  }



  actionlib::SimpleClientGoalState getState() //! Returns the current state of the action
  {
    return traj_client_->getState();
  }

};


int main (int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory"); //ros::NodeHandle n;

  Ur3Arm arm;
  ros::Rate loop_rate(4);

  // Wait for trajectory completion
  while( ros::ok() ) // !arm.getState().isDone() &&
  {
    //ros::spinOnce();
    // Start the trajectory

    if(arm.getState().isDone()){
      //arm.startTrajectory(arm.armExtensionTrajectory());

      if(arm.startTrajectory(arm.armExtensionTrajectory())==true){
        ros::spinOnce();
      }
      else{
        std::cout << "[ST] Unable to reach trajectory goal !" <<std::endl;}
    }

    //usleep(50000); // Argument of usleep is given in miliseconds -> Wait fo 50s
    loop_rate.sleep();
  }

  return 0;
}
