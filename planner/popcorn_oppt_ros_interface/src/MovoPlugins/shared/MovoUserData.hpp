#ifndef _MOVO_USER_DATA_HPP_
#define _MOVO_USER_DATA_HPP_
#include "oppt/opptCore/RobotStateUserData.hpp"

namespace oppt
{

class MovoTransitionPluginUserData: public RobotStateUserData {
public:
	MovoTransitionPluginUserData():
		RobotStateUserData() {

	}

	virtual ~MovoTransitionPluginUserData() = default;

	VectorFloat jointAngles;

	GZPose relativeObjectPose;

	CollisionReportSharedPtr collisionReport = nullptr;

	GZPose rightEEPose;

	GZPose knuckleLinkPose;

	VectorFloat knucklePoseVector;
	VectorFloat rightEEPoseVector;

	// Bool flag parameters used in setUserData
	bool scoopingPhase = false;
	
	bool handClosed = false;

	bool rightArmCollision = false;
	
	bool leftArmCollision = false;

	bool knuckleObjectCollision = false;

	bool graspJustEstablished = false;

	bool illegalHandOpening = false;

	bool graspAttempted = false;

	bool inGraspingCondition = false;

	bool leftArmObjectCollided = false;

	bool bothArmsCollided = false;

	bool objectOutOfBounds = false;

	bool illegalFingerCollision = false;
};

class MovoGraspingTransitionPluginUserData: public MovoTransitionPluginUserData
{
public:
	MovoGraspingTransitionPluginUserData():
		MovoTransitionPluginUserData() {

	}

	virtual ~MovoGraspingTransitionPluginUserData() = default;	

	FloatType distanceToObject = 0;

	FloatType distanceXAngle = 0.0;

	FloatType distanceYAngle = 0.0;

	FloatType distX = 0.0;
	FloatType distY = 0.0;
	FloatType distZ = 0.0;

};

class MovoScoopingTransitionPluginUserData: public MovoTransitionPluginUserData {
public:
	MovoScoopingTransitionPluginUserData():
		MovoTransitionPluginUserData() {

	}

	VectorRobotStatePtr subStates;

	bool validIKSolutions = true;

	bool resampleWorldState = true;
};

class MovoObservationPluginUserData: public OpptUserData {
public:
	MovoObservationPluginUserData():
		OpptUserData() {

	}

	virtual ~MovoObservationPluginUserData() = default;

	bool objectPoseRandomized = false;

};

class MovoInitialBeliefPluginUserData: public oppt::OpptUserData {
public:
	MovoInitialBeliefPluginUserData(): 
	OpptUserData() {

	}

	VectorString jointNames;

	VectorFloat jointAngles;

	VectorFloat objectPosition;

	Quaternionf objectOrientation;

	VectorFloat tablePosition;

	Quaternionf tableOrientation;

	VectorFloat tableDimensions;

	VectorFloat candyBoxPosition;

	Quaternionf candyBoxOrientation;

	FloatType candyHeight = 0.0;

	bool gripperClosed = false;

	bool graspEstablished = false;

};

class MovoInitialBeliefPluginScoopingUserData: public oppt::OpptUserData {
public:
	MovoInitialBeliefPluginScoopingUserData():
	OpptUserData() {

	}

	VectorRobotStatePtr beliefParticles;

	bool randomizeObjectPose = false;

};

class MovoRetreatUserData: public oppt::OpptUserData {
public:
	MovoRetreatUserData():
	OpptUserData() {

	}

	ObservationSharedPtr observation = nullptr;

};

}

#endif
