#ifndef _MOVO_OBSERVATION_HPP_
#define _MOVO_OBSERVATION_HPP_
#include <oppt/robotHeaders/Observation.hpp>

namespace oppt {
class MovoGraspingObservation: public VectorObservation {
public:
	MovoGraspingObservation(VectorFloat& observationVec):
		VectorObservation(observationVec) {

	}

	MovoGraspingObservation(const VectorFloat& observationVec):
		VectorObservation(observationVec) {

	}

	virtual ~MovoGraspingObservation() {}

	virtual FloatType distanceTo(const Observation& otherObservation) const override {

		FloatType dist = 0.0;


		VectorFloat observationVec = static_cast<const MovoGraspingObservation &>(otherObservation).asVector();
		

		// If one of the observations perceived an object pose and the other
		// didn't, return max distance
		if ((std::isnan(observationVec_[0]) && !(std::isnan(observationVec[0]))) ||
		        (!(std::isnan(observationVec_[0])) && std::isnan(observationVec[0]))) {
			return std::numeric_limits<FloatType>::max();
		}

		// If one observation says hand is closed and the other doesn't, return max distance
		if ((observationVec_[observationVec_.size() - 2] != observationVec[observationVec.size() - 2])){
			return std::numeric_limits<FloatType>::max();
		}

		// If one observation says grasp established and the other doesn't, return max distance
		if ((observationVec_[observationVec_.size() - 1] !=  observationVec[observationVec.size() - 1] )){
			//std::cout << "INCONSISTENT GRASPING OBSERVATION" << std::endl;
			return std::numeric_limits<FloatType>::max();
		}

		
		// If both observations perceived an object pose, calc the distance based on the
		// perceived object pose. However, if both observations didn't perceive an object pose,
		// the distance will be zero, i.e. they are treated as equal
		if (!(std::isnan(observationVec_[0]))) {
			for (size_t i = 0; i != 2; ++i) {
				dist += std::pow(observationVec_[i] - observationVec[i], 2);
			}
		}

		for (size_t i = 2; i != 8; ++i) {
			FloatType diffA = diffAngles(observationVec_[i], observationVec[i]);			
			dist += diffA * diffA;
		}


		return sqrt(dist);	

	}

protected:
	FloatType diffAngles(const FloatType& a1, const FloatType& a2) const {
		return atan2(sin(a1 - a2), cos(a1 - a2));
	}

};
}

#endif