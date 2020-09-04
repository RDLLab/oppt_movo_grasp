#ifndef _MOVO_UTILS_HPP_
#define _MOVO_UTILS_HPP_
#include "oppt/opptCore/core.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

namespace oppt {

/**
 * @brief Calculate the angle between the object orientation and a target vector
 */
inline FloatType distOrientation(const GZPose &targetLinkPose,
                                 const oppt::Vector3f &targetVec,
                                 const unsigned int &axis = 2) {

	// Object orientation in quaternion form
#ifdef GZ_GT_7
	oppt::Quaternionf objectQ(targetLinkPose.Rot().W(),
	                          targetLinkPose.Rot().X(),
	                          targetLinkPose.Rot().Y(),
	                          targetLinkPose.Rot().Z());
#else
	oppt::Quaternionf objectQ(targetLinkPose.rot.w,
	                          targetLinkPose.rot.x,
	                          targetLinkPose.rot.y,
	                          targetLinkPose.rot.z);
#endif


	// The axis we're rotating into the object frame
	oppt::Vector3f axisVec;
	if (axis == 0) {
		axisVec = oppt::Vector3f(1.0, 0.0, 0.0);
	} else if (axis == 1) {
		axisVec = oppt::Vector3f(0.0, 1.0, 0.0);
	} else if (axis == 2) {
		axisVec = oppt::Vector3f(0.0, 0.0, 1.0);

	} else {
		ERROR("Axis not specified");
	}

	// Rotate the axis into the object orientation
	oppt::Vector3f rotatedVec = objectQ._transformVector(axisVec);

	// Calculate the angle between the rotated z-axis and the target axis;
	FloatType angle = acos(rotatedVec.dot(targetVec) / (rotatedVec.norm() * targetVec.norm()));

	return angle;
}

inline bool satisfiesSphereGoalGrasping(const RobotEnvironment *robotEnvironment,
                                        const GZPose &targetLinkPose,
                                        const VectorFloat &goalPosition,
                                        const FloatType &goalRadius) {
#ifdef GZ_GT_7
	VectorFloat targetLinkPosition({targetLinkPose.Pos().X(), targetLinkPose.Pos().Y(), targetLinkPose.Pos().Z()});
#else
	VectorFloat targetLinkPosition({targetLinkPose.pos.x, targetLinkPose.pos.y, targetLinkPose.pos.z});
#endif
	FloatType dist = math::euclideanDistance(targetLinkPosition.data(), goalPosition.data(), 3);
	bool satisfiesDistance = dist <= goalRadius ? true : false;
	if (!satisfiesDistance)
		return false;
	oppt::Vector3f targetVec(1.0, 0.0, -1.0);
	targetVec.normalize();
	FloatType distOr = distOrientation(targetLinkPose, targetVec);
	if (distOr < 0.5)
		return true;
	return false;
}

inline bool linkInsideBox(const GZPose &targetLinkPose,
                          const VectorFloat &box,
                          const FloatType maxZDistance = 0.0) {
#ifdef GZ_GT_7
	VectorFloat targetLinkPosition({targetLinkPose.Pos().X(), targetLinkPose.Pos().Y(), targetLinkPose.Pos().Z()});
#else
	VectorFloat targetLinkPosition({targetLinkPose.pos.x, targetLinkPose.pos.y, targetLinkPose.pos.z});
#endif
	if (targetLinkPosition[0] > box[0] + box[3] / 2.0)
		return false;
	if (targetLinkPosition[0] < box[0] - box[3] / 2.0)
		return false;
	if (targetLinkPosition[1] > box[1] + box[4] / 2.0)
		return false;
	if (targetLinkPosition[1] < box[1] - box[4] / 2.0)
		return false;
	if (maxZDistance == 0.0) {
		if (targetLinkPosition[2] < box[2] - box[5] / 2.0)
			return false;
		if (targetLinkPosition[2] > box[2] + box[5] / 2.0)
			return false;
	} else {
		if (targetLinkPosition[2] < box[2] - box[5] / 2.0)
			return false;
		if (targetLinkPosition[2] > box[2] + maxZDistance)
			return false;
	}

	return true;

}

inline bool stateInsideGoalArea(const GZPose& targetLinkPose, const VectorFloat &goalArea) {
#ifdef GZ_GT_7
	VectorFloat targetLinkPosition({targetLinkPose.Pos().X(), targetLinkPose.Pos().Y(), targetLinkPose.Pos().Z()});
#else
	VectorFloat targetLinkPosition({targetLinkPose.pos.x, targetLinkPose.pos.y, targetLinkPose.pos.z});
#endif
	if (math::euclideanDistance(targetLinkPosition.data(), goalArea.data(), 3) < goalArea[3])
		return true;
	return false;
}

inline bool satisfiesCandyBoxGoalGrasping(const RobotEnvironment *robotEnvironment,
        const GZPose& targetLinkPose,
        const VectorFloat &enteringPose,
        const VectorFloat &box,
        const VectorFloat &goalArea) {
	if (!stateInsideGoalArea(targetLinkPose, goalArea)) {
		return false;
	}

	//if (!stateInsideBox(state, box))
	//	return false;

	oppt::Vector3f targetVec(enteringPose[0], enteringPose[1], enteringPose[2]);
	FloatType distOr1 = distOrientation(targetLinkPose, targetVec, 2);
	targetVec[0] = -1.0;
	FloatType distOr2 = distOrientation(targetLinkPose, targetVec, 2);
	if (std::fabs(distOr1) < 0.15 || std::fabs(distOr2) < 0.15) {
		return true;
	}
	return false;
}

/**
     * @brief Helper function that returns the gazebo::physics::LinkPtr for the given link name
     */
inline gazebo::physics::Link* getLinkPtr(const std::string &linkName, const RobotEnvironment *robotEnvironment) {
	auto links = robotEnvironment->getGazeboInterface()->getLinks();

	for (auto & link : links) {
		if (link->GetName().find(linkName) != std::string::npos) {
			return link;
		}
	}

	return nullptr;
}

inline VectorFloat getSphereGoal(const std::string &name, const RobotEnvironment *robotEnvironment) {
	auto goalAreaLink = getLinkPtr(name, robotEnvironment);
#ifdef GZ_GT_7
	auto pose = goalAreaLink->WorldPose();
#else
	auto pose = goalAreaLink->GetWorldPose();
#endif
	auto sdf = goalAreaLink->GetSDF();
	auto visualElement = sdf->GetElement("visual");
	if (!visualElement)
		ERROR("Visual element doesn't exists");
	auto geometryElement = visualElement->GetElement("geometry");
	if (!geometryElement)
		ERROR("GeometryElement is null");
	auto sphereElement = geometryElement->GetElement("sphere");
	if (!sphereElement)
		ERROR("Sphere element doesn't exist");
	auto radiusElement = sphereElement->GetElement("radius");
	FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
#ifdef GZ_GT_7
	return VectorFloat({pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), radius});
#else
	return VectorFloat({pose.pos.x, pose.pos.y, pose.pos.z, radius});
#endif
}

inline VectorFloat getBoxGoal(const std::string &name, const RobotEnvironment *robotEnvironment) {
	auto boxLinkPtr = getLinkPtr(name, robotEnvironment);
#ifdef GZ_GT_7
	auto pose = boxLinkPtr->WorldPose();
#else
	auto pose = boxLinkPtr->GetWorldPose();
#endif
	auto sdf = boxLinkPtr->GetSDF();
	auto visualElement = sdf->GetElement("visual");
	if (!visualElement)
		ERROR("Visual element doesn't exists");
	auto geometryElement = visualElement->GetElement("geometry");
	if (!geometryElement)
		ERROR("GeometryElement is null");
	auto boxElement = geometryElement->GetElement("box");
	sdf::ElementPtr sizeElement = boxElement->GetElement("size");
	std::string sizeStr = sizeElement->GetValue()->GetAsString();
	VectorString sizeElems;
	split(sizeStr, ' ', sizeElems);

#ifdef GZ_GT_7
	return VectorFloat({pose.Pos().X(),
	                    pose.Pos().Y(),
	                    pose.Pos().Z(),
	                    atof(sizeElems[0].c_str()),
	                    atof(sizeElems[1].c_str()),
	                    atof(sizeElems[2].c_str())
	                   });
#else
	return VectorFloat({pose.pos.x,
	                    pose.pos.y,
	                    pose.pos.z,
	                    atof(sizeElems[0].c_str()),
	                    atof(sizeElems[1].c_str()),
	                    atof(sizeElems[2].c_str())
	                   });
#endif
}

}
#endif
