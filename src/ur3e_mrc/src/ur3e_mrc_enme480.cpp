
#include <ur3e_mrc/ur3e_mrc_enme480.h>

#define N_JOINTS 6
#define CTRL_TO_RUN "scaled_pos_joint_traj_controller"

UR3eArm::UR3eArm()
{
  trajectory_client_ = new TrajectoryClient("scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
  ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
  init_status_ = trajectory_client_->waitForServer(ros::Duration(20.0));
  // while (!trajectory_client_->waitForServer(ros::Duration(60)))
  //   ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
  if (init_status_)
    ROS_INFO_STREAM("Connected to the action server");
  else
    ROS_ERROR("Failed to connect to the action server");
  // init_status_ = true;

  pos_pub = nh_.advertise<ur3e_mrc::position>("ur3/position", 10);
  ROS_INFO("Advertised ur3e position topic");

  sub_js_ = nh_.subscribe("joint_states", 10, &UR3eArm::jsCallback, this);
  ROS_INFO("Subscribed to joint states");

  sub_comm_ = nh_.subscribe("ur3/command", 10, &UR3eArm::commCallback, this);
  ROS_INFO("Subscribed to ur3 command");

  // Controller manager service to switch controllers
  ctrl_manager_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  // Controller manager service to list controllers
  ctrl_list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

  ctrl_manager_srv_.waitForExistence();
  ctrl_list_srv_.waitForExistence();

  ROS_INFO("Running scaled_pos_joint_traj_controller check");
  ctrlRunCheck();
}

UR3eArm::~UR3eArm()
{
  delete trajectory_client_;
  nh_.shutdown();
}

void UR3eArm::ctrlRunCheck()
{
  controller_manager_msgs::ListControllers list_srv;
  ctrl_list_srv_.call(list_srv);
  // stopped_controllers_.clear();
  for (auto &controller : list_srv.response.controller)
  {
    // Check if in consistent_controllers
    // Else:
    //   Add to stopped_controllers
    // if (controller.state == "running")
    if (controller.name == CTRL_TO_RUN)
    {
      // auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
      if (controller.state != "running")
      {
        ROS_INFO("Attempting to activate scaled_pos_joint_traj_controller");
        std::vector<std::string> ctrl_to_run; // = {CTRL_TO_RUN};
        ctrl_to_run.push_back(CTRL_TO_RUN);

        controller_manager_msgs::SwitchController srv;
        srv.request.strictness = srv.request.STRICT;
        srv.request.start_controllers = ctrl_to_run;
        if (!ctrl_manager_srv_.call(srv))
        {
          ROS_ERROR_STREAM("Could not activate scaled_pos_joint_traj_controller");
        }
      }
    }
  }
}

void UR3eArm::sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  trajectory_client_->sendGoal(goal);
  ROS_INFO_STREAM("Send joint trajectory goal to server successfully!");
}

void UR3eArm::initGoal(control_msgs::FollowJointTrajectoryGoal &goal)
{
  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("elbow_joint");
  goal.trajectory.joint_names.push_back("wrist_1_joint");
  goal.trajectory.joint_names.push_back("wrist_2_joint");
  goal.trajectory.joint_names.push_back("wrist_3_joint");
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].velocities.resize(N_JOINTS);
  goal.trajectory.points[0].velocities = {}; // Initialize with all zeros
  goal.trajectory.points[0].accelerations.resize(N_JOINTS);
  goal.trajectory.points[0].accelerations = {}; // Initialize with all zeros
}

bool UR3eArm::initStatus()
{
  return init_status_;
}

//   goal.trajectory.points[index].positions = {0, -1.57, -1.57, 0, 0, 0};
//   goal.trajectory.points[index].positions = {0.2, -1.57, -1.57, 0, 0, 0};
//   goal.trajectory.points[index].positions = {-0.5, -1.57, -1.2, 0, 0, 0};
//   goal.trajectory.points[index].positions = {0.0, 0.0, 0.0, 0, 0, 0};

actionlib::SimpleClientGoalState UR3eArm::getState()
{
  return trajectory_client_->getState();
}

void UR3eArm::jsCallback(const sensor_msgs::JointState &msg)
{
  pos_msg.position = {msg.position[2] + boost::math::constants::pi<double>(), msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]};

  pos_msg.isReady = true;
  pos_pub.publish(pos_msg);
}

void UR3eArm::commCallback(const ur3e_mrc::command &msg)
{
  // ROS_INFO("Got ur3e command");
  if (msg.destination.size() != N_JOINTS)
  {
    ROS_INFO("WARNNING: In commCallback- received command size is not 6");
    return;
  }

  // if(msg->io_0 == true ) {
  // 	sprintf(buf,"set_digital_out(0,True)\n");
  // 	sendCMD = buf;
  // } else {
  // 	sprintf(buf,"set_digital_out(0,False)\n");
  // 	sendCMD = buf;
  // }

  // initiate the goal variable
  control_msgs::FollowJointTrajectoryGoal goal;
  initGoal(goal);

  // velocity = msg->v;
  // acceleration = msg->a;
  // if (acceleration < 0.1) {
  // 	ROS_INFO("Acceleration too low setting to 0.1 rad/s^2");
  // 	acceleration = 0.1;
  // }
  // if (acceleration > 4.0) {
  // 	ROS_INFO("Acceleration too high setting to 4.0 rad/s^2");
  // 	acceleration = 4.0;
  // }

  // if (velocity < 0.1) {
  // 	ROS_INFO("Velocity too low setting to 0.1 rad/s");
  // 	velocity = 0.1;
  // }
  // if (velocity > 4.0) {
  // 	ROS_INFO("Velocity too high setting to 4.0 rad/s");
  // 	velocity = 4.0;
  // }

  // fill the positions of the goal
  goal.trajectory.points[0].positions.resize(N_JOINTS);
  goal.trajectory.points[0].positions = msg.destination;
  goal.trajectory.points[0].positions[0] -= boost::math::constants::pi<double>();
  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
  // pt.time_from_start = get_duration(data.destination, data.v)

  sendTrajectory(goal);
  while (!getState().isDone() && ros::ok())
  {
    usleep(50000);
  }

  // global gripper_is_on

  // // Vacuum Gripper

  // if data.io_0 and (not gripper_is_on):
  //     gripper_on_srv = rospy.ServiceProxy('/gripper/on', Empty)
  //     gripper_on_srv()
  //     gripper_is_on = True
  // elif (not data.io_0) and gripper_is_on:
  //     gripper_off_srv = rospy.ServiceProxy('/gripper/off', Empty)
  //     gripper_off_srv()
  //     gripper_is_on = False

  // cmd_pub.publish(jt)
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ur3e_mrc_enme480");

  UR3eArm ur3e_mrc_var;

  if (ur3e_mrc_var.initStatus())
    ros::spin();
  else
    ros::shutdown();

  return 0;
}