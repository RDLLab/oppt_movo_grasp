#ifndef _POPCORN_ROS_OPPT_INTERFACE_HPP_
#define _POPCORN_ROS_OPPT_INTERFACE_HPP_
#include <iostream>
#include <popcorn_oppt_ros_interface_msgs/OPPTPlanAction.h>
#include <popcorn_oppt_ros_interface_msgs/OPPTJointPositionAction.h>
#include <popcorn_oppt_ros_interface_msgs/OPPTObservation.h>
#include <popcorn_oppt_ros_interface_msgs/OPPTInitBeliefAction.h>
#include <popcorn_oppt_ros_interface_msgs/OPPTTestAction.h>
#include <actionlib/server/simple_action_server.h>
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "ActionReceiver.hpp"
#include "BeliefUpdater.hpp"

using std::cout;
using std::endl;

namespace oppt
{

typedef actionlib::SimpleActionServer<popcorn_oppt_ros_interface_msgs::OPPTPlanAction> PlanServer;
typedef actionlib::SimpleActionServer<popcorn_oppt_ros_interface_msgs::OPPTInitBeliefAction> InitialBeliefActionServer;

class PopcornOpptROSInterface
{
public:
    PopcornOpptROSInterface();

    ~PopcornOpptROSInterface();

    /**
     * @brief Initializes the OPPT ProblemEnvironment such that is uses the multithreaded version of
     * ABT on the Movo problem
     */
    bool initProblemEnvironment(int argc, char const* argv[]);

private:
    /**
     * @brief Callback function that is called when the action server receives a new request
     */
    void planCallback_(const popcorn_oppt_ros_interface_msgs::OPPTPlanGoalConstPtr& goal);

    void initBeliefCallback_(const popcorn_oppt_ros_interface_msgs::OPPTInitBeliefGoalConstPtr& goal);

    ReceivedActionUniquePtr getNextAction_(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation);

private:
    /** @brief The action server for plan action */
    std::unique_ptr<PlanServer> planServer_ = nullptr;

    std::unique_ptr<InitialBeliefActionServer> initialBeliefActionserver_ = nullptr;

    std::unique_ptr<InitialBeliefActionServer> initialBeliefActionserver2_ = nullptr;    

    //std::unique_ptr<TestServer> testServer_ = nullptr;

    /** @brief A ROS node handle */
    ros::NodeHandle nh_;

    /** @brief The OPPT problem environment */
    std::unique_ptr<ProblemEnvironment> problemEnvironment_ = nullptr;

    std::unique_ptr<ActionReceiver> actionReceiver_ = nullptr;    

    std::unique_ptr<BeliefUpdater> beliefUpdater_ = nullptr;

    /** @brief A shared pointer to the last action that was executed. Initially this pointer is NULL*/
    ReceivedActionUniquePtr lastAction_ = nullptr;

    popcorn_oppt_ros_interface_msgs::OPPTPlanResult result_;

    mutable bool ignoreObjectPoseFromRos_ = false;

    FloatType stepExecutionDuration_ = 0.0;    

    bool firstStep_ = true;

private:
    /**
     * @brief Converts a popcorn_oppt_ros_interface_msgs::OPPTObservation to a shared pointer to an oppt::Observation
     */
    ObservationSharedPtr getObservationFromRosMsg_(const popcorn_oppt_ros_interface_msgs::OPPTObservation& msg);

    /**
     * @brief Converts a oppt::ActionSharedPtr to a popcorn_oppt_ros_interface_msgs::OPPTJointPositionAction
     */
    popcorn_oppt_ros_interface_msgs::OPPTJointPositionAction getRosMsgFromAction_(ReceivedAction *action) const;

    /**
     * @brief Tells the solver that the run is finished
     */
    void abort_();

};
}

#endif