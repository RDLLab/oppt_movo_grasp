#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "popcorn_oppt_ros_interface.hpp"
#include "MultithreadedABT/solverABTMultithreaded.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "MovoGraspingObservation.hpp"
#include "MovoAction.hpp"
#include "MovoGraspingTransitionOptions.hpp"
#include "MovoUserData.hpp"
namespace oppt
{



PopcornOpptROSInterface::PopcornOpptROSInterface():
    nh_("~")
{
    nh_.setParam("/oppt_plan_server_initialized", false);
    nh_.setParam("/oppt_init_belief_server_initialized", false);

    // Initialize the initial belief action server
    initialBeliefActionserver_ =
        std::make_unique<InitialBeliefActionServer>(nh_,
                "init_belief",
                boost::bind(&PopcornOpptROSInterface::initBeliefCallback_, this, _1),
                false);

    // Initialize the action server
    planServer_ = std::make_unique<PlanServer>(nh_, "plan", boost::bind(&PopcornOpptROSInterface::planCallback_, this, _1), false);

    
    // Construct a oppt::ProblemEnvironment
    problemEnvironment_ = std::make_unique<ProblemEnvironment>();

    // Add me back in again!!!!
    result_.belief_updated = false;
}

PopcornOpptROSInterface::~PopcornOpptROSInterface() {
    nh_.setParam("/oppt_plan_server_initialized", false);
    nh_.setParam("/oppt_init_belief_server_initialized", false);
}


bool PopcornOpptROSInterface::initProblemEnvironment(int argc, char const* argv[])
{

    char const* arguments[argc];
    for (int i=0;i<argc;i++)
    {
        arguments[i]=argv[i];
    }
    
    std::string configFile;
    int ret = 2;
    if (nh_.getParam("problemConfigFile", configFile)) {
        // This will be executed when the popcorn_oppt_ros_interface is launched from a launch file
        argc = 3;
        char const* argv3[3] = {"popcorn_oppt_ros_interface_node", "--cfg", configFile.c_str()};
        ret = problemEnvironment_->setup<solvers::ABTMultithreaded, solvers::MultithreadedABTOptions>(argc, argv3);
    } else {
        ret = problemEnvironment_->setup<solvers::ABTMultithreaded, solvers::MultithreadedABTOptions>(argc, arguments);
    }

    if (ret != 0) {
        // If we couldn't setup the ProblemEnvironment, we can't go on
        ROS_ERROR("Couldn't setup problem environment");
        return false;
    }

    auto transitionPluginOptions =
        problemEnvironment_->getRobotExecutionEnvironment()->getRobot()->getTransitionPlugin()->getOptions();
    stepExecutionDuration_ =
        static_cast<MovoGraspingTransitionPluginOptions *const>(transitionPluginOptions)->stepDuration;


    initialBeliefActionserver_->start();
    LOGGING("BELIEF SERVER STARTED");
    nh_.setParam("/oppt_init_belief_server_initialized", true);    
    actionReceiver_ = std::make_unique<ActionReceiver>(problemEnvironment_.get());
    beliefUpdater_ = std::make_unique<BeliefUpdater>(problemEnvironment_.get());


    ros::Duration(2.0).sleep();

    // Start the ActionServer
    return true;
}


void PopcornOpptROSInterface::planCallback_(const popcorn_oppt_ros_interface_msgs::OPPTPlanGoalConstPtr& goal)
{

    FloatType timeStart = oppt::clock_ms();

    result_ = popcorn_oppt_ros_interface_msgs::OPPTPlanResult();
    auto solver = static_cast<solvers::ABTMultithreaded *>(problemEnvironment_->getSolver());

    // Convert the PopcornTaskObservation contained in the goal to a oppt::ObservationSharedPtr
    ObservationSharedPtr observation = getObservationFromRosMsg_(goal->observation);
    cout << "Time to get observation from ROs msg: " << (oppt::clock_ms() - timeStart) / 1000.0 << endl;;
    if (!observation)
        ERROR("No observation?!");
    LOGGING("-- GOT OBSERVATION FROM ROBOT:");
    cout << "Observation: " << *(observation.get()) << endl;

    // Update the belief
    timeStart = oppt::clock_ms();
    bool beliefUpdated = beliefUpdater_->updateBelief(lastAction_.get(), observation, goal->observation);
    cout << "time to update belief: " << (oppt::clock_ms() - timeStart) / 1000.0 << endl;
    if (!beliefUpdated)
        ERROR("Belief could not be updated");
    result_.belief_updated = beliefUpdated;




    // Get the next action to execute
    timeStart = oppt::clock_ms();
    lastAction_ = getNextAction_(goal->observation);
    cout << "Time to get next action: " << (oppt::clock_ms() - timeStart) / 1000.0 << endl;
    if (lastAction_) {
        timeStart = oppt::clock_ms();
        popcorn_oppt_ros_interface_msgs::OPPTJointPositionAction cmd = getRosMsgFromAction_(lastAction_.get());
        cout << "Time to get ROS msg from action: " << (oppt::clock_ms() - timeStart) / 1000.0 << endl;
        result_.action = cmd;

        // Tell our clients that their request was successful
        planServer_->setSucceeded(result_);

        if (beliefUpdated && !firstStep_) {
            std::ofstream os;
            LOGGING("-- Serialize step");
            problemEnvironment_->getSolver()->serializeStep(os);
        }

        firstStep_ = false;
        return;
    }

    LOGGING("Aborting result");
    planServer_->setAborted(result_);
    abort_();
}

ReceivedActionUniquePtr PopcornOpptROSInterface::getNextAction_(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) {
    // Use the ActionReceiver to get the next action
    FloatType timeStart = oppt::clock_ms();
    ReceivedActionUniquePtr receivedAction = std::move(actionReceiver_->getNextAction(observation));
    cout << "time to receive action: " << (oppt::clock_ms() - timeStart) / 1000.0 << endl;
    if (receivedAction->movoSubAction) {
        // We got a subaction. Add it to the MovoAction
        receivedAction->primitiveAction->as<MovoAction>()->movoSubActions.push_back(receivedAction->movoSubAction);
    }
    cout << "Time for getNextAction_ to finish: " << (oppt::clock_ms() - timeStart) / 1000.0 << endl;



    return std::move(receivedAction);
}



void PopcornOpptROSInterface::initBeliefCallback_(const popcorn_oppt_ros_interface_msgs::OPPTInitBeliefGoalConstPtr& goal) {
    auto userData = std::make_shared<MovoInitialBeliefPluginUserData>();
    auto ud = static_cast<MovoInitialBeliefPluginUserData *>(userData.get());

    ud->jointNames = goal->initial_state.joint_names;
    ud->jointAngles = goal->initial_state.joint_position.positions;

    for (auto &object : goal->initial_state.objects) {
        VectorFloat position({object.primitive_poses[0].position.x,
                              object.primitive_poses[0].position.y,
                              object.primitive_poses[0].position.z
                             });
        Quaternionf orientation(object.primitive_poses[0].orientation.w,
                                object.primitive_poses[0].orientation.x,
                                object.primitive_poses[0].orientation.y,
                                object.primitive_poses[0].orientation.z);
        LOGGING("-- Got object with id '" + object.id + "'");
        if (object.id.find("merlo_cup") != std::string::npos || object.id.find("pringles") != std::string::npos || object.id.find("tube") != std::string::npos) {
            ud->objectPosition = position;
            ud->objectOrientation = orientation;
        } else if (object.id.find("table") != std::string::npos) {
            ud->tablePosition = position;
            ud->tableOrientation = Quaternionf(1.0, 0.0, 0.0, 0.0);

            // Use very long model of table to be conservative 

            ud->tableDimensions = VectorFloat({object.primitives[0].dimensions[0],
                                               100,
                                               object.primitives[0].dimensions[2]
                                              });
            printVector(ud->tableDimensions, "TABLE DIMENSIONS");
        } else if (object.id.find("candy") != std::string::npos) {
            ud->candyBoxPosition = position;
            ud->candyBoxOrientation = orientation;
            ud->candyHeight = object.primitive_poses[1].position.z;
        } else {
            WARNING("Detected an obstacle, but obstacles are currently not handled by the solver");

        }

    }


    ud->gripperClosed = goal->initial_state.gripper_closed;
    ud->graspEstablished = goal->initial_state.object_grasped;

    if (!(problemEnvironment_->getRobotPlanningEnvironment()->getInitialBeliefPlugin()))
        ERROR("Initial belief plugin is null");
    problemEnvironment_->getRobotPlanningEnvironment()->getInitialBeliefPlugin()->setUserData(userData);
    problemEnvironment_->getRobotExecutionEnvironment()->getInitialBeliefPlugin()->setUserData(userData);
    problemEnvironment_->getRobotExecutionEnvironment()->applyChanges();

    auto result = popcorn_oppt_ros_interface_msgs::OPPTInitBeliefResult();
    result.belief_updated = true;
    initialBeliefActionserver_->setSucceeded(result);
    if (!(problemEnvironment_->getSolver()->reset())) {
        // If we couldn't reset the solver (which also starts the planning loop)
        // we can't go on
        ERROR("Can't reset solver");
    }



    // Sleep for a second to allow for some time to plan
    boost::this_thread::sleep_for(boost::chrono::milliseconds((unsigned int)1000));
    auto initState = problemEnvironment_->getRobotExecutionEnvironment()->sampleInitialState();    

    planServer_->start();
    nh_.setParam("/oppt_plan_server_initialized", true);
    ROS_INFO("Planning action server started");

    problemEnvironment_->updateViewer(initState);

}

void PopcornOpptROSInterface::abort_()
{
    std::ofstream os;
    problemEnvironment_->getSolver()->runFinished(os, 0);
    nh_.setParam("/oppt_plan_server_initialized", false);
    static_cast<solvers::ABTMultithreaded *>(problemEnvironment_->getSolver())->switchToGraspingSolver();
    nh_.setParam("/oppt_init_belief_server_initialized", false);
    actionReceiver_->clear();
    firstStep_ = false;
    throw 2;
}



ObservationSharedPtr PopcornOpptROSInterface::getObservationFromRosMsg_(const popcorn_oppt_ros_interface_msgs::OPPTObservation& msg)
{
   VectorFloat observationVector(10, 0);
    observationVector[0] = msg.object_pose.position.x;
    observationVector[1] = msg.object_pose.position.y;
    for (size_t i = 0; i != msg.joint_names.size(); ++i) {
        if (msg.joint_names[i].find("right_shoulder_pan_joint") != std::string::npos) {
            observationVector[2] = msg.joint_position.positions[i];
        } else if (msg.joint_names[i].find("right_shoulder_lift_joint") != std::string::npos) {
            observationVector[3] = msg.joint_position.positions[i];
        } else if (msg.joint_names[i].find("right_elbow_joint") != std::string::npos) {
            observationVector[4] = msg.joint_position.positions[i];
        } else if (msg.joint_names[i].find("right_wrist_spherical_1_joint") != std::string::npos) {
            observationVector[5] = msg.joint_position.positions[i];
        } else if (msg.joint_names[i].find("right_wrist_spherical_2_joint") != std::string::npos) {
            observationVector[6] = msg.joint_position.positions[i];
        } else if (msg.joint_names[i].find("right_wrist_3_joint") != std::string::npos) {
            observationVector[7] = msg.joint_position.positions[i];
        }

    }

    if (int(msg.gripper_closed) == 1) {
        observationVector[8] = 1.0;
    }

    if (int(msg.object_grasped) == 1) {
        observationVector[9] = 1.0;
    }

    /////////////////////////////////////////////////////////
    if (observationVector[9] > 0.5) {
        ignoreObjectPoseFromRos_ = true;
    }

    if (ignoreObjectPoseFromRos_) {
        
    }
    ////////////////////////////////////////////////////////
    printVector(observationVector, "observationVector");

    ObservationSharedPtr observation(new MovoGraspingObservation(observationVector));
    if (!ignoreObjectPoseFromRos_) {
        
    }

    return observation;
}

popcorn_oppt_ros_interface_msgs::OPPTJointPositionAction PopcornOpptROSInterface::getRosMsgFromAction_(ReceivedAction *action) const
{
    // Prepare the ros message

    popcorn_oppt_ros_interface_msgs::OPPTJointPositionAction msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names = VectorString({"right_shoulder_pan_joint",
                                    "right_shoulder_lift_joint",
                                    "right_arm_half_joint",
                                    "right_elbow_joint",
                                    "right_wrist_spherical_1_joint",
                                    "right_wrist_spherical_2_joint",
                                    "right_wrist_3_joint"
                                   });

    if (!action)
        ERROR("Action is null?!");
    if (action->movoSubAction) {
        if (action->movoSubAction->getType() == MovoSubActionType::APPROACH) {
            auto actionSequence = action->movoSubAction->as<MovoApproachAction>()->getActionSequence();
            for (size_t i = 0; i != actionSequence.size(); ++i) {
                msg.point_increments.push_back(trajectory_msgs::JointTrajectoryPoint());
                VectorFloat actionVec = actionSequence[i]->as<VectorAction>()->asVector();
                msg.point_increments[i].positions = VectorFloat({actionVec[0],
                                                    actionVec[1],
                                                    actionVec[2],
                                                    actionVec[3],
                                                    actionVec[4],
                                                    actionVec[5]
                                                                });
                if (actionSequence.size() > 1) {
                    msg.point_increments[i].time_from_start = ros::Duration(10.0);
                } else {
                    msg.point_increments[i].time_from_start = ros::Duration(4.0);
                }
            }

            msg.gripper_command = 0;
            msg.update_observation = false;
        } else if (action->movoSubAction->getType() == MovoSubActionType::SCAN) {
            msg.gripper_command = 0;
            msg.update_observation = true;
        } else if (action->movoSubAction->getType() == MovoSubActionType::GRASP) {
            msg.gripper_command = 1;
            msg.update_observation = false;
        } else if (action->movoSubAction->getType() == MovoSubActionType::RETRACT) {
            auto actionSequence = action->movoSubAction->as<MovoRetractAction>()->getActionSequence();
            if (actionSequence.empty())
                ERROR("EMPTY RETRACT SEQUENCE");
            for (size_t i = 0; i != actionSequence.size(); ++i) {
                msg.point_increments.push_back(trajectory_msgs::JointTrajectoryPoint());
                VectorFloat actionVec = actionSequence[i]->as<VectorAction>()->asVector();
                msg.point_increments[i].positions = VectorFloat({actionVec[0],
                                                    actionVec[1],
                                                    actionVec[2],
                                                    actionVec[3],
                                                    actionVec[4],
                                                    actionVec[5]
                                                                });
                if (actionSequence.size() > 1) {
                    msg.point_increments[i].time_from_start = ros::Duration(10.0);
                } else {
                    msg.point_increments[i].time_from_start = ros::Duration(4.0);
                }

            }
            msg.gripper_command = 0;
            msg.update_observation = false;
        } else if (action->movoSubAction->getType() == MovoSubActionType::NONE) {
            ERROR("None action");
        }
        else {
            ERROR("UNKNOWN SUB ACTION?!");
        }
    } else {
        // Primitive action only
        if (!(action->primitiveAction))
            ERROR("Primitive action is null");
        VectorFloat actionVec = action->primitiveAction->as<VectorAction>()->asVector();        
        msg.point_increments.push_back(trajectory_msgs::JointTrajectoryPoint());
        msg.point_increments[0].positions = VectorFloat({actionVec[0],
                                            actionVec[1]});
        msg.point_increments[0].positions.push_back(0.0);
        for (size_t i = 2; i != 6; ++i) {
            msg.point_increments[0].positions.push_back(actionVec[i]);
        }
        


        msg.point_increments[0].time_from_start = ros::Duration(stepExecutionDuration_ / 1000.0);
        if (actionVec[actionVec.size() - 1] > 0.5)
            msg.gripper_command = 1;
        else if (actionVec[actionVec.size() - 1] < -0.5)
            msg.gripper_command = -1;
        else
            msg.gripper_command = 0;
        //msg.gripper_command = actionVec[actionVec.size() - 1] > 0.5 ? 1 : 0;

        if(msg.gripper_command == 1)
            LOGGING("SENDING CLOSE GRIPPER")
        //msg.gripper_command = 0;
        msg.update_observation = false;
    }

    

    return msg;
}

}

int main(int argc, char const* argv[])
{
    int argc2 = 0;
    char** argv2;
    ros::init(argc2, argv2, "oppt_ros_interface");
    ros::NodeHandle node;


    oppt::PopcornOpptROSInterface rosInterface;
    if (!(rosInterface.initProblemEnvironment(argc, argv))) {
        ROS_ERROR("Initialization of ProblemEnvironment failed.");
        return 2;
    }
    ros::spin();
        
    return 0;
}