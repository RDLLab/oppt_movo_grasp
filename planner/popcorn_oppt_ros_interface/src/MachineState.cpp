#include "StateMachine.hpp"

namespace oppt {

///////////////////////////////////////////////////////////////////////
//// Scan state before approach
///////////////////////////////////////////////////////////////////////
ScanStateBeforeApproach::ScanStateBeforeApproach():
	MachineState() {

}

std::unique_ptr<MachineState> ScanStateBeforeApproach::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	std::unique_ptr<MachineState> nextState = nullptr;

	// If the scan was unsuccessful, the next state action should be retract
	if (std::isnan(observation.object_pose.position.x)) {
		LOGGING("-- UPDATE STATE MACHINE AFTER SCAN BEFORE APPROACH STATE. UPDATE TO RETRACT ACTION");
		nextState = std::unique_ptr<MachineState>(new RetractStateBeforeApproach);
	} else {
		// Otherwise we approach the object
		// But only if the object is in reach of the hand. If not, we retract first
		//if (observation.object_pose.position.y < observation.gripper_pose.position.y + 0.02) {
		//	LOGGING("-- UPDATE STATE MACHINE AFTER SCAN BEFORE APPROACH STATE. UPDATE TO RETRACT ACTION (BECAUSE OBJECT IS OUT OF REACH)")
		//	nextState = std::unique_ptr<MachineState>(new RetractStateBeforeApproach);
		//} else {
		LOGGING("-- UPDATE STATE MACHINE AFTER SCAN BEFORE APPROACH STATE. UPDATE TO APPROACH ACTION");
		nextState = std::unique_ptr<ApproachState>(new ApproachState);
		//}
	}

	return std::move(nextState);
}

std::shared_ptr<MovoSubAction> ScanStateBeforeApproach::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	LOGGING("-- GENERATE A SCAN ACTION");
	return std::make_shared<MovoScanAction>();
}

bool ScanStateBeforeApproach::startFiltering() const {
	return false;
}

///////////////////////////////////////////////////////////////////////
//// Approach state
///////////////////////////////////////////////////////////////////////
ApproachState::ApproachState():
	MachineState() {

}

std::unique_ptr<MachineState> ApproachState::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	// After performing the approach action, we reset the state machine
	LOGGING("-- UPDATE STATE MACHINE AFTER APPROACH ACTION");
	return nullptr;
}

std::shared_ptr<MovoSubAction> ApproachState::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	// Get the current joint angles
#ifdef ENABLE_MACRO_ACTIONS
	LOGGING("-- GENERATE A APPROACH ACTION");
	VectorFloat jointAngles(7, 0.0);
	for (size_t i = 0; i != observation.joint_names.size(); ++i) {
		if (observation.joint_names[i].find("linear_joint") != std::string::npos) {
			jointAngles[0] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_shoulder_pan_joint") != std::string::npos) {
			jointAngles[1] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_shoulder_lift_joint") != std::string::npos) {
			jointAngles[2] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_elbow_joint") != std::string::npos) {
			jointAngles[3] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_1_joint") != std::string::npos) {
			jointAngles[4] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_2_joint") != std::string::npos) {
			jointAngles[5] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_3_joint") != std::string::npos) {
			jointAngles[6] = observation.joint_position.positions[i];
		}
	}

	// Calculate the target joint angles
	VectorFloat objPose({observation.object_pose.position.x,
	                     observation.object_pose.position.y,
	                     observation.object_pose.position.z
	                    });
	ClosestPose closestPose = stateMachine_->getObjectPoseNN()->getClosestJointAngles(objPose);

	// Solve the PRM
	stateMachine_->getPRM()->ignoreFingerCollisions(true);
	stateMachine_->getPRM()->setNumSolutionAttempts(3);
	size_t trajectorySize = 100000;
	size_t attempt = 0;
	TrajectorySharedPtr trajectory = nullptr;
	TrajectorySharedPtr bestTrajectory = nullptr;
	while (trajectorySize > 1) {
		LOGGING("-- ATTEMPT " + std::to_string(attempt))
		attempt += 1;
		if (attempt > 5) {
			break;
		}

		trajectory = stateMachine_->getPRM()->solve(VectorFloat(), jointAngles, closestPose.jointAngles, 5.0);
		if (!trajectory) {
			WARNING("No retract trajectory");
			return std::make_shared<MovoNoneAction>();
		}

		if (trajectory) {
			if (!bestTrajectory)
				bestTrajectory = trajectory;
			trajectorySize = trajectory->actionTrajectory.size();
			if (trajectorySize < bestTrajectory->actionTrajectory.size())
				bestTrajectory = trajectory;

			LOGGING("-- TRAJECTORY SIZE: " + std::to_string(trajectorySize));
		}
	}

	if (!bestTrajectory)
		ERROR("Couldn't find a trajectory");

	std::vector<ActionSharedPtr> actionSequence = bestTrajectory->actionTrajectory;;
	return std::make_shared<MovoApproachAction>(actionSequence);
#else
	ERROR("You're trying to generate a macro action, but your system doesn't support macro actions");
#endif
}

bool ApproachState::startFiltering() const {
	return true;
}

///////////////////////////////////////////////////////////////////////
//// Retract state before approach
///////////////////////////////////////////////////////////////////////
RetractStateBeforeApproach::RetractStateBeforeApproach():
	MachineState() {

}

std::unique_ptr<MachineState> RetractStateBeforeApproach::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	// After retracting, we always perform a scan action
	LOGGING("-- UPDATE STATE MACHINE AFTER RETRACT STATE BEFORE APPROACH. UPDATE TO SCAN STATE BEFORE APPROACH");
	std::unique_ptr<MachineState> nextState(new ScanStateBeforeApproach());
	return std::move(nextState);
}

std::shared_ptr<MovoSubAction> RetractStateBeforeApproach::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	// Get the current joint angles
#ifdef ENABLE_MACRO_ACTIONS
	LOGGING("-- GENERATE RETRACT ACTION BEFORE APPROACH");
	VectorFloat jointAngles(7, 0.0);
	for (size_t i = 0; i != observation.joint_names.size(); ++i) {
		if (observation.joint_names[i].find("linear_joint") != std::string::npos) {
			jointAngles[0] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_shoulder_pan_joint") != std::string::npos) {
			jointAngles[1] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_shoulder_lift_joint") != std::string::npos) {
			jointAngles[2] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_elbow_joint") != std::string::npos) {
			jointAngles[3] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_1_joint") != std::string::npos) {
			jointAngles[4] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_2_joint") != std::string::npos) {
			jointAngles[5] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_3_joint") != std::string::npos) {
			jointAngles[6] = observation.joint_position.positions[i];
		}
	}

	// Calculate the target joint angles
	VectorFloat handPosition({observation.gripper_pose.position.x,
	                          observation.gripper_pose.position.y,
	                          observation.gripper_pose.position.z
	                         });
	ClosestPose closestPose = stateMachine_->getRetractPoseNN()->getClosestJointAngles(handPosition);
	if (closestPose.closestPose.size() < 2)
		ERROR("Closest pose too small");

	// Solve the PRM
	stateMachine_->getPRM()->ignoreFingerCollisions(true);
	stateMachine_->getPRM()->setMaximumyPositionHand(observation.gripper_pose.position.y);
	stateMachine_->getPRM()->setMinimumYPositionHand(closestPose.closestPose[1] - 0.2);
	stateMachine_->getPRM()->setNumSolutionAttempts(3);
	stateMachine_->getPRM()->setAggressiveMode(true);
	size_t trajectorySize = 100000;
	size_t attempt = 0;
	TrajectorySharedPtr trajectory = nullptr;
	TrajectorySharedPtr bestTrajectory = nullptr;
	while (trajectorySize > 1) {
		LOGGING("-- ATTEMPT " + std::to_string(attempt))
		attempt += 1;
		if (attempt > 5) {
			break;
		}

		trajectory = stateMachine_->getPRM()->solve(VectorFloat(), jointAngles, closestPose.jointAngles, 5.0);
		if (!trajectory) {
			WARNING("No retract trajectory");
			continue;
		}

		if (trajectory) {
			if (!bestTrajectory)
				bestTrajectory = trajectory;
			trajectorySize = trajectory->actionTrajectory.size();
			if (trajectorySize < bestTrajectory->actionTrajectory.size())
				bestTrajectory = trajectory;

			LOGGING("-- TRAJECTORY SIZE: " + std::to_string(trajectorySize));
		}
	}

	if (!bestTrajectory)
		ERROR("Couldn't find a trajectory");

	stateMachine_->getPRM()->unsetMaxYPositionHand();
	stateMachine_->getPRM()->unsetMinYPositionHand();
	stateMachine_->getPRM()->setAggressiveMode(false);

	std::vector<ActionSharedPtr> actionSequence = bestTrajectory->actionTrajectory;
	return std::make_shared<MovoRetractAction>(actionSequence);
#else
	ERROR("You're trying to generate a macro action, but your system doesn't support macro actions");
#endif
}

bool RetractStateBeforeApproach::startFiltering() const {
	return false;
}

///////////////////////////////////////////////////////////////////////
//// Retract state after grasp
///////////////////////////////////////////////////////////////////////
RetractStateAfterGrasp::RetractStateAfterGrasp():
	MachineState() {

};

std::unique_ptr<MachineState> RetractStateAfterGrasp::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	LOGGING("-- UPDATE STATE MACHINE AFTER RETRACT AFTER GRASP. UPDATE TO SCAN STATE AFTER GRASP");
	std::unique_ptr<MachineState> nextState(new ScanStateAfterGrasp);
	return std::move(nextState);
}

std::shared_ptr<MovoSubAction> RetractStateAfterGrasp::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
#ifdef ENABLE_MACRO_ACTIONS
	LOGGING("-- GENERATE RETRACT ACTION AFTER UNSUCCESSFUL GRASP");
	// Get the current joint angles
	VectorFloat jointAngles(7, 0.0);
	for (size_t i = 0; i != observation.joint_names.size(); ++i) {
		if (observation.joint_names[i].find("linear_joint") != std::string::npos) {
			jointAngles[0] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_shoulder_pan_joint") != std::string::npos) {
			jointAngles[1] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_shoulder_lift_joint") != std::string::npos) {
			jointAngles[2] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_elbow_joint") != std::string::npos) {
			jointAngles[3] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_1_joint") != std::string::npos) {
			jointAngles[4] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_2_joint") != std::string::npos) {
			jointAngles[5] = observation.joint_position.positions[i];
		} else if (observation.joint_names[i].find("right_wrist_3_joint") != std::string::npos) {
			jointAngles[6] = observation.joint_position.positions[i];
		}
	}

	// Calculate the target joint angles
	VectorFloat handPosition({observation.gripper_pose.position.x,
	                          observation.gripper_pose.position.y,
	                          observation.gripper_pose.position.z
	                         });
	ClosestPose closestPose = stateMachine_->getRetractPoseNN()->getClosestJointAngles(handPosition);
	if (closestPose.closestPose.size() < 2)
		ERROR("Closest pose too small");

	// Solve the PRM
	stateMachine_->getPRM()->ignoreFingerCollisions(true);
	stateMachine_->getPRM()->setMaximumyPositionHand(observation.gripper_pose.position.y);
	stateMachine_->getPRM()->setMinimumYPositionHand(closestPose.closestPose[1] - 0.2);
	stateMachine_->getPRM()->setNumSolutionAttempts(3);
	stateMachine_->getPRM()->setAggressiveMode(true);
	size_t trajectorySize = 100000;
	size_t attempt = 0;
	TrajectorySharedPtr trajectory = nullptr;
	TrajectorySharedPtr bestTrajectory = nullptr;
	while (trajectorySize > 1) {
		LOGGING("-- ATTEMPT " + std::to_string(attempt))
		attempt += 1;
		if (attempt > 5) {
			break;
		}

		trajectory = stateMachine_->getPRM()->solve(VectorFloat(), jointAngles, closestPose.jointAngles, 5.0);
		if (!trajectory) {
			WARNING("No retract trajectory");
			continue;
		}

		if (trajectory) {
			if (!bestTrajectory)
				bestTrajectory = trajectory;
			trajectorySize = trajectory->actionTrajectory.size();
			if (trajectorySize < bestTrajectory->actionTrajectory.size())
				bestTrajectory = trajectory;

			LOGGING("-- TRAJECTORY SIZE: " + std::to_string(trajectorySize));
		}
	}

	if (!bestTrajectory)
		ERROR("Couldn't find a trajectory");

	stateMachine_->getPRM()->unsetMaxYPositionHand();
	stateMachine_->getPRM()->unsetMinYPositionHand();
	stateMachine_->getPRM()->setAggressiveMode(false);

	std::vector<ActionSharedPtr> actionSequence = bestTrajectory->actionTrajectory;
	return std::make_shared<MovoRetractAction>(actionSequence);
#else
	ERROR("You're trying to generate a macro action, but your system doesn't support macro actions");
#endif
	
}

bool RetractStateAfterGrasp::startFiltering() const {
	return true;
}

///////////////////////////////////////////////////////////////////////
//// Scan state after grasp
///////////////////////////////////////////////////////////////////////
ScanStateAfterGrasp::ScanStateAfterGrasp():
	MachineState() {

}

std::unique_ptr<MachineState> ScanStateAfterGrasp::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	if (std::isnan(observation.object_pose.position.x)) {
		LOGGING("-- UPDATE STATE MACHINE AFTER SCAN STATE AFTER GRASP. UPDATE TO RETRACT ACTION");
		auto nextState = std::unique_ptr<MachineState>(new RetractStateAfterGrasp);
		return std::move(nextState);
	} else {
		/**if (observation.object_pose.position.y < observation.gripper_pose.position.y + 0.02) {
			LOGGING("-- UPDATE STATE MACHINE AFTER SCAN STATE AFTER GRASP. UPDATE TO RETRACT ACTION (BECAUSE OBJECT IS OUT OF REACH)");
			auto nextState = std::unique_ptr<MachineState>(new RetractStateAfterGrasp);
			return std::move(nextState);
		}*/
		LOGGING("-- UPDATE STATE MACHINE AFTER SCAN STATE AFTER GRASP. UPDATE TO NULLPTR");
		return nullptr;
	}
}

std::shared_ptr<MovoSubAction> ScanStateAfterGrasp::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	return std::make_shared<MovoScanAction>();
}

bool ScanStateAfterGrasp::startFiltering() const {
	return false;
}

///////////////////////////////////////////////////////////////////////
//// Grasp state
///////////////////////////////////////////////////////////////////////
GraspState::GraspState():
	MachineState() {

}

std::unique_ptr<MachineState> GraspState::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	if (observation.object_grasped) {
		// Successful grasp
		LOGGING("-- UPDATE STATE MACHINE AFTER GRASP STATE. UPDATE TO NULLPTR");
		return nullptr;
	} else {
		// Unsuccessful grasp. We need a scan action
		LOGGING("-- UPDATE STATE MACHINE AFTER GRASP STATE. UPDATE TO SCAN STATE AFTER GRASP");
		std::unique_ptr<MachineState> nextState(new ScanStateAfterGrasp);
		return std::move(nextState);
	}
}

std::shared_ptr<MovoSubAction> GraspState::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	return std::make_shared<MovoGraspAction>();
}

bool GraspState::startFiltering() const {
	return true;
}

///////////////////////////////////////////////////////////////////////
//// Single scan state
///////////////////////////////////////////////////////////////////////
SingleScanState::SingleScanState():
	MachineState() {
}

std::unique_ptr<MachineState> SingleScanState::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	return nullptr;
}

std::shared_ptr<MovoSubAction> SingleScanState::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	return std::make_shared<MovoScanAction>();
}

bool SingleScanState::startFiltering() const {
	return true;
}

///////////////////////////////////////////////////////////////////////
//// Single grasp state
///////////////////////////////////////////////////////////////////////
SingleGraspState::SingleGraspState():
	MachineState() {
}

std::unique_ptr<MachineState> SingleGraspState::getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	return nullptr;
}

std::shared_ptr<MovoSubAction> SingleGraspState::generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const {
	return std::make_shared<MovoGraspAction>();
}

bool SingleGraspState::startFiltering() const {
	return true;
}

}