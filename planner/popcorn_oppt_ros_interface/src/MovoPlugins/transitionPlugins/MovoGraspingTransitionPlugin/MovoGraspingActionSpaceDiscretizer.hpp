#ifndef _MOVO_ACTION_SPACE_DISCRETIZER_
#define _MOVO_ACTION_SPACE_DISCRETIZER_
#include "oppt/robotHeaders/ActionSpaceDiscretizer.hpp"
#include "MovoAction.hpp"

namespace oppt {

class MovoGraspingActionSpaceDiscretizer: public CustomActionSpaceDiscretizer {
public:
	MovoGraspingActionSpaceDiscretizer(ActionSpaceSharedPtr &actionSpace,
	                                   const std::vector<unsigned int>& discretization):
		CustomActionSpaceDiscretizer(actionSpace, discretization) {
	}

	virtual ~MovoGraspingActionSpaceDiscretizer() {}

	virtual std::vector<ActionSharedPtr> getAllActionsInOrder(const unsigned int& numStepsPerDimension) const {
		// Container for the results
		std::vector<ActionSharedPtr> allActionsOrdered_;

		ActionLimitsSharedPtr actionLimits = actionSpace_->getActionLimits();
		VectorFloat lowerLimits;
		VectorFloat upperLimits;
		actionLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerLimits, upperLimits);
		unsigned int numActions = discretization_[0];
		//Compute size of action space
		for (size_t i = 1; i != discretization_.size(); ++i) {
			numActions *= discretization_[i];
		}		

		// Get lower limits
		VectorFloat actionVals(lowerLimits);

		// Loop variables
		long code = 0;
		int currIndex = 0;
		unsigned int processesActions = 0;



		// Loop to populate action space with defined discretization in cfg file
		while (processesActions != (numActions) - 1) {
			actionVals[currIndex] +=
			    ((upperLimits[currIndex] - lowerLimits[currIndex]) / ((FloatType)(discretization_[currIndex]) - 1));
			if (actionVals[currIndex] > upperLimits[currIndex]) {
				while (true) {
					currIndex++;
					actionVals[currIndex] +=
					    ((upperLimits[currIndex] - lowerLimits[currIndex]) / ((FloatType)(discretization_[currIndex]) - 1));
					if (actionVals[currIndex] <= upperLimits[currIndex])
						break;
				}
				for (size_t i = 0; i != currIndex; ++i) {
					actionVals[i] = lowerLimits[i];
				}
				currIndex = 0;
			}

			
			ActionSharedPtr action(new MovoAction(actionVals));
			action->as<DiscreteVectorAction>()->setBinNumber(code);
			allActionsOrdered_.push_back(action);
			code++;
			
			processesActions++;
		}

		// Generate custom action definitions
		VectorFloat closingActionVals({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
		VectorFloat openingActionVals({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0});	

		// Closing action as movo action
		ActionSharedPtr closingAction(new MovoAction(closingActionVals));
		closingAction->as<DiscreteVectorAction>()->setBinNumber(code);
		code++;
		// Opening action as movoAction
		ActionSharedPtr openingAction(new MovoAction(openingActionVals));
		openingAction->as<DiscreteVectorAction>()->setBinNumber(code);
		code++;


		// Push newly created actions to the list of actions
		allActionsOrdered_.push_back(closingAction);
		allActionsOrdered_.push_back(openingAction);
	

		
		return allActionsOrdered_;
	}

};

}

#endif