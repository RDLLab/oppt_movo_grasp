#ifndef _BELIEF_UPDATER_HPP_
#define _BELIEF_UPDATER_HPP_
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "ActionReceiver.hpp"
#include "MovoGraspingObservation.hpp"
#include "MovoGraspingTransitionOptions.hpp"

namespace oppt {
class BeliefUpdater {
public:
	BeliefUpdater(ProblemEnvironment *const problemEnvironment):
		problemEnvironment_(problemEnvironment) {
		auto transitionPluginOptions =
		    problemEnvironment_->getRobotExecutionEnvironment()->getRobot()->getTransitionPlugin()->getOptions();
#ifdef ENABLE_MACRO_ACTIONS
		singleScanAction_ = static_cast<MovoTransitionPluginOptions*>(transitionPluginOptions)->singleScanAction;
#endif

	}

	bool updateBelief(ReceivedAction *action, ObservationSharedPtr &observation, const popcorn_oppt_ros_interface_msgs::OPPTObservation& observationMsg) {
		if (!action) {
			// Action is null, so we don't need to update the belief
			return true;
		}

		if (!observation)
			return false;

		auto solver = problemEnvironment_->getSolver();
		if (action->movoSubAction) {
			// Handle grasp subaction
			if (action->movoSubAction->getType() == MovoSubActionType::GRASP) {
				VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
				if (observationVec[observationVec.size() - 1] > 0.5 || singleScanAction_) {
					// If we performed a grasp action and we perceived a grasp, update the belief as usual.
					// Also update the belief if we performed a single grasp action
					LOGGING("-- UPDATE BELIEF AFTER GRASP");
					return solver->updateBelief(action->primitiveAction, observation);
				} else {
					// If we performed a grasp and we perceived no grasp, don't update the belief yet.
					// We will perform a scan action first
					LOGGING("-- GRASP NOT SUCCESSFUL. DON'T UPDATE BELIEF")
					return true;
				}
			} else if (action->movoSubAction->getType() == MovoSubActionType::SCAN) {
				// Handle scan action
				VectorFloat observationVec = observation->as<MovoGraspingObservation>()->asVector();
				VectorFloat actionVec = action->primitiveAction->as<VectorAction>()->asVector();

				if (!(std::isnan(observationVec[0]))) {
					// We performed a scan action and observed the object. In a scan-and-approach action
					// we won't update the belief (unless we performed a single scan action),
					// since we will perform an approach action first

					// However, in a grasp action, we will update the belief, since the scan action is
					// the last action in a grasp-and-scan macro action
					if (actionVec[actionVec.size() - 1] > 0.5 && !singleScanAction_) {
						// Scan-and-approach action. Store the observation
						LOGGING("-- PERFORMED SCAN ACTION OF SCAN-AND-APPROACH. STORING OBSERVATION");
						cachedObservation_ = observation;
						return true;
					} else {
						//LOGGING("-- PERFORMED SCAN ACTION AFTER UNSUCCESSFUL GRASP. CHECKING IF WE CAN UPDATE THE BELIEF");
						/**if (observationMsg.object_pose.position.y < observationMsg.gripper_pose.position.y + 0.02) {
						    LOGGING("-- BLOCK BELIEF UPDATE FOR UPCOMING RETRACT ACTION");
						    return true;
						}*/
						LOGGING("-- UPDATE BELIEF WITH OBSERVATION:");
						cout << *(observation.get()) << endl;
						if (!(action->primitiveAction))
							ERROR("Trying to update belief with a null action");
						return solver->updateBelief(action->primitiveAction, observation);
					}
				} else {
					// We performed a scan action but got no observation. This means that we will now
					// perform a retract action. Don't update the belief
					if (singleScanAction_) {
						// However, if we performed a single scan action, we update the belief regardless of the
						// outcome
						return solver->updateBelief(action->primitiveAction, observation);
					} else {
						LOGGING("-- PERFORMED SCAN ACTION BUT GOT NO OBSERVATION. DON'T UPDATE BELIEF");
						return true;
					}
				}
			} else if (action->movoSubAction->getType() == MovoSubActionType::APPROACH) {
				// Handle approach action
				// We performed an approach action. This means that there was a previous scan action.
				// So we combine the cached observation from the scan action and the observation
				// we received after the approach action and perform the belief update
				if (!cachedObservation_)
					ERROR("No cached observation?!");
				cout << "-- CACHED OBSERVATION: " << *(cachedObservation_.get()) << endl;
				VectorFloat cachedObservationVec = cachedObservation_->as<MovoGraspingObservation>()->asVector();
				VectorFloat currentObservationVec = observation->as<MovoGraspingObservation>()->asVector();
				VectorFloat newObservationVec(cachedObservationVec.begin(), cachedObservationVec.begin() + 2);
				newObservationVec.insert(newObservationVec.end(), currentObservationVec.begin() + 2, currentObservationVec.end());
				ObservationSharedPtr mergedObservation(new MovoGraspingObservation(newObservationVec));
				cout << "-- MERGED OBSERVATION: " << *(mergedObservation.get()) << endl;
				cachedObservation_ = nullptr;
				return solver->updateBelief(action->primitiveAction, mergedObservation);
			} else if (action->movoSubAction->getType() == MovoSubActionType::RETRACT) {
				// Don't update the belief after a retract action. We will perform another scan action first
				LOGGING("-- PERFORMED RETRACT ACTION. DON'T UPDATE BELIEF");
				return true;
			} else if (action->movoSubAction->getType() == MovoSubActionType::NONE) {
				LOGGING("-- NONE ACTION. DON'T UPDATE BELIEF");
				return true;
			}

		} else {
			// If we have a primitive action only, update the belief as usual
			LOGGING("-- UPDATE BELIEF USING PRIMITIVE ACTION ONLY");
			return solver->updateBelief(action->primitiveAction, observation);
		}

		return false;
	}

private:
	ProblemEnvironment *problemEnvironment_ = nullptr;

	ObservationSharedPtr cachedObservation_ = nullptr;

	bool singleScanAction_ = false;
};
}

#endif