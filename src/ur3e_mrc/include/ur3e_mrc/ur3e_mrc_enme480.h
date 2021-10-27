#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <ur3e_mrc/command.h>
#include <ur3e_mrc/position.h>
#include <ur3e_mrc/gripper_input.h>

#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// #include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
// #include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

ros::Publisher chatter_pub;
ros::Publisher pos_pub;
ur3e_mrc::position pos_msg;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class UR3eArm
{
private:
    ros::NodeHandle nh_;
    TrajectoryClient *trajectory_client_;
    ros::Subscriber sub_js_;
    ros::Subscriber sub_comm_;

    bool init_status_ = false;

public:
    UR3eArm();
    ~UR3eArm();
    void jsCallback(const sensor_msgs::JointState &msg);
    void commCallback(const ur3e_mrc::command &msg);

    void initGoal(control_msgs::FollowJointTrajectoryGoal &goal);
    void sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
    // control_msgs::FollowJointTrajectoryGoal ur3eTrajectory(); //const ur3e_mrc::position &msg
    bool initStatus();
    actionlib::SimpleClientGoalState getState();

protected:
};