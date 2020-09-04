#ifndef _ACTION_RECEIVER_HPP_
#define _ACTION_RECEIVER_HPP_
#include "MovoAction.hpp"
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "StateMachine.hpp"
#include "MovoGraspingTransitionOptions.hpp"
#ifdef ENABLE_MACRO_ACTIONS
#include "plugins/transitionPlugins/MovoTransitionPlugins/PRM/PRM.hpp"
#include "plugins/transitionPlugins/MovoTransitionPlugins/IKSolutions/IKSolutionsParser.hpp"
#include "plugins/transitionPlugins/MovoTransitionPlugins/IKSolutions/PoseNearestNeighbour.hpp"
#endif

namespace oppt {

struct ReceivedAction {
	std::shared_ptr<MovoSubAction> movoSubAction = nullptr;

	ActionSharedPtr primitiveAction = nullptr;

};

typedef std::unique_ptr<ReceivedAction> ReceivedActionUniquePtr;

class ActionReceiver {
public:
	ActionReceiver(ProblemEnvironment *problemEnvironment):
		problemEnvironment_(problemEnvironment) {
		auto transitionPluginOptions =
		    problemEnvironment_->getRobotExecutionEnvironment()->getRobot()->getTransitionPlugin()->getOptions();
#ifdef ENABLE_MACRO_ACTIONS
		IKSolutionsParser ikParser;
		IKSolutionsVector ikSolutionsObject = ikParser.loadIKSamples(static_cast<MovoTransitionPluginOptions*>(transitionPluginOptions)->ikOutputFileObject);
		IKSolutionsVector ikSolutionsRetract = ikParser.loadIKSamples(static_cast<MovoTransitionPluginOptions*>(transitionPluginOptions)->ikOutputFileRetract);
		LOGGING("Setting up pose nearest-neighbour datastructure...");
		objectPoseNN_ = std::make_unique<PoseNearestNeighbour>(ikSolutionsObject);
		LOGGING("Setting up retract nearest-neighbour datastructure");
		retractPoseNN_ = std::make_unique<PoseNearestNeighbour>(ikSolutionsRetract);
#endif

		makeMovoJointMap();

		std::vector<gazebo::physics::Joint*> jts({movoJointMap_.at("right_shoulder_pan_joint"),
		                           movoJointMap_.at("right_shoulder_lift_joint"),
		                           movoJointMap_.at("right_elbow_joint"),
		                           movoJointMap_.at("right_wrist_1_joint"),
		                           movoJointMap_.at("right_wrist_2_joint"),
		                           movoJointMap_.at("right_wrist_3_joint")});

#ifdef ENABLE_MACRO_ACTIONS
		prm_ = std::make_unique<PRM>(problemEnvironment_->getRobotExecutionEnvironment());
		prm_->setup();
		prm_->setRobotJoints(jts);
		singleScanAction_ = static_cast<MovoTransitionPluginOptions*>(transitionPluginOptions)->singleScanAction;
#endif

	}

	void clear() {
		currentAction_ = nullptr;
		stateMachine_ = nullptr;
	}

	ReceivedActionUniquePtr getNextAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) {
		ReceivedActionUniquePtr nextAction(new ReceivedAction);
#ifndef ENABLE_MACRO_ACTIONS
		// If we don't use macro actions, we simply return a
		// new action obtained by the solver
		
		nextAction->primitiveAction = problemEnvironment_->getSolver()->getNextAction();
		nextAction->primitiveAction->as<MovoAction>()->finishedExecuting = true;
		return std::move(nextAction);
#else
		if (!currentAction_) {
			// No action => Get one from the solver
			currentAction_ = problemEnvironment_->getSolver()->getNextAction();
		}

		nextAction->primitiveAction = currentAction_;

		// Check if we need to intialize a state machine
		if (!stateMachine_) {
			VectorFloat actionVec = currentAction_->as<MovoAction>()->asVector();
			if (actionVec[actionVec.size() - 1] > 0.1) {
				// Scan and approach
				if (singleScanAction_) {
					LOGGING("-- GENERATE A SINGLE-SCAN STATE MACHINE");
					stateMachine_ = std::unique_ptr<StateMachine>(new SingleScanStateMachine());
				} else {
					LOGGING("-- GENERATE A SCAN-AND-APPROACH STATE MACHINE");
					stateMachine_ = std::unique_ptr<StateMachine>(new ScanAndApproachStateMachine());
				}
			} else if (actionVec[actionVec.size() - 2] > 0.5) {
				// Grasp and rescann
				if (singleScanAction_) {
					LOGGING("-- GENERATE A SINGLE-GRASP STATE MACHINE");
					stateMachine_ = std::unique_ptr<StateMachine>(new SingleGraspStateMachine());
				} else {
					LOGGING("-- GENERATE A GRASP-AND-SCAN STATE MACHINE");
					stateMachine_ = std::unique_ptr<StateMachine>(new GraspAndScanStateMachine());
				}
			}

			if (stateMachine_) {
				// If we just initialized a state machine, get the first subaction

				stateMachine_->setPRM(prm_.get());
				stateMachine_->setObjectPoseNN(objectPoseNN_.get());
				stateMachine_->setRetractPoseNN(retractPoseNN_.get());
				nextAction->movoSubAction = stateMachine_->getCurrentState()->generateAction(observation);
				nextAction->primitiveAction->as<MovoAction>()->finishedExecuting = stateMachine_->getCurrentState()->startFiltering();
			} else {
				// We perform a primitive action, so in the next step we get a new action
				nextAction->primitiveAction->as<MovoAction>()->finishedExecuting = true;
				currentAction_ = nullptr;
			}
		} else {
			// Update the state machine
			LOGGING("-- UPDATING THE STATE MACHINE");
			stateMachine_->transitionState(observation);
			MachineState *const machineState = stateMachine_->getCurrentState();
			if (machineState) {
				// Get the action from the next state in the state machine
				LOGGING("-- WE GOT THE NEXT ACTION FROM THE STATE MACHINE. THE STATE WAS UPDATED TO " + machineState->description());
				nextAction->movoSubAction = machineState->generateAction(observation);
				nextAction->primitiveAction->as<MovoAction>()->finishedExecuting = stateMachine_->getCurrentState()->startFiltering();
			} else {
				// We reached the end of the current state machine, so we get a fresh action from the solver
				LOGGING("-- REACHED THE END OF THE STATE MACHINE. GETTING A NEW ACTION FROM THE SOLVER");
				stateMachine_ = nullptr;
				currentAction_ = nullptr;
				nextAction->primitiveAction = problemEnvironment_->getSolver()->getNextAction();
				nextAction->primitiveAction->as<MovoAction>()->finishedExecuting = true;
			}
		}

		return std::move(nextAction);
#endif
	}

private:
	ProblemEnvironment *problemEnvironment_ = nullptr;

	ActionSharedPtr currentAction_ = nullptr;

	std::unique_ptr<StateMachine> stateMachine_ = nullptr;

#ifdef ENABLE_MACRO_ACTIONS
	std::unique_ptr<PRM> prm_ = nullptr;

	std::unique_ptr<PoseNearestNeighbour> objectPoseNN_ = nullptr;

	std::unique_ptr<PoseNearestNeighbour> retractPoseNN_ = nullptr;
#endif

	std::unordered_map<std::string, gazebo::physics::Joint*> movoJointMap_;

	bool singleScanAction_ = false;

	bool useMacroActions_ = false;

private:
	void makeMovoJointMap() {
		movoJointMap_["linear_joint"] = nullptr;
		movoJointMap_["right_shoulder_pan_joint"] = nullptr;
		movoJointMap_["right_shoulder_lift_joint"] = nullptr;
		movoJointMap_["right_elbow_joint"] = nullptr;
		movoJointMap_["right_wrist_1_joint"] = nullptr;
		movoJointMap_["right_wrist_2_joint"] = nullptr;
		movoJointMap_["right_wrist_3_joint"] = nullptr;
		movoJointMap_["right_gripper_finger1_joint"] = nullptr;
		movoJointMap_["right_gripper_finger2_joint"] = nullptr;
		movoJointMap_["right_gripper_finger3_joint"] = nullptr;
		auto joints = problemEnvironment_->getRobotExecutionEnvironment()->getGazeboInterface()->getJoints();
		for (auto & joint : joints) {
			std::string name = joint->GetName();
			std::string unscopedName = name;
			if (name.find("::") != std::string::npos) {
				VectorString nameElems;
				split(name, "::", nameElems);
				unscopedName = nameElems[nameElems.size() - 1];
			}

			if (movoJointMap_.find(unscopedName) != movoJointMap_.end())
				movoJointMap_.at(unscopedName) = joint;
		}
	}

};
}

#endif