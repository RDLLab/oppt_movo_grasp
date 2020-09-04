#ifndef _MOVO_ACTION_HPP_
#define _MOVO_ACTION_HPP_
#include "oppt/robotHeaders/Action.hpp"

namespace oppt {

enum MovoSubActionType {
	NONE,
	SCAN,
	APPROACH,
	GRASP,
	RETRACT
};

class MovoSubAction {
public:
	MovoSubAction() = default;

	virtual ~MovoSubAction() = default;

	/**
	 * @brief Static cast to type T
	 * @tparam T The action type to cast to
	 */
	template<class T>
	T* as() {
		return static_cast<T*>(this);
	}

	virtual std::unique_ptr<MovoSubAction> copy() const = 0;

	virtual MovoSubActionType getType() const = 0;

	virtual std::string description() const = 0;

};

typedef std::shared_ptr<MovoSubAction> MovoSubActionSharedPtr;
typedef std::vector<MovoSubActionSharedPtr> MovoSubActions;

class MovoApproachAction: public MovoSubAction {
public:
	MovoApproachAction(std::vector<ActionSharedPtr> &actionSequence):
		MovoSubAction(),
		actionSequence_(actionSequence) {

	}

	virtual ~MovoApproachAction() = default;

	virtual MovoSubActionType getType() const override {
		return MovoSubActionType::APPROACH;
	}

	virtual std::unique_ptr<MovoSubAction> copy() const override {
		std::vector<ActionSharedPtr> copiedActionSequence(actionSequence_.size(), nullptr);
		for (size_t i = 0; i != copiedActionSequence.size(); ++i) {
			copiedActionSequence[i] = ActionSharedPtr(std::move(actionSequence_[i]->copy()));
		}

		std::unique_ptr<MovoSubAction> copiedSubAction(new MovoApproachAction(copiedActionSequence));
		return std::move(copiedSubAction);
	}

	virtual std::string description() const override {
		return "APPROACH ACTION";
	}

	std::vector<ActionSharedPtr> getActionSequence() const {
		return actionSequence_;
	}

private:
	std::vector<ActionSharedPtr> actionSequence_;

};

class MovoScanAction: public MovoSubAction {
public:
	MovoScanAction():
		MovoSubAction() {
	}

	virtual ~MovoScanAction() = default;

	virtual MovoSubActionType getType() const override {
		return MovoSubActionType::SCAN;
	}

	virtual std::unique_ptr<MovoSubAction> copy() const override {
		std::unique_ptr<MovoSubAction> copiedAction(new MovoScanAction());
		return std::move(copiedAction);
	}

	virtual std::string description() const override {
		return "SCAN ACTION";
	}

};

class MovoGraspAction: public MovoSubAction {
public:
	MovoGraspAction():
		MovoSubAction() {
	}

	virtual ~MovoGraspAction() = default;

	virtual MovoSubActionType getType() const override {
		return MovoSubActionType::GRASP;
	}

	virtual std::unique_ptr<MovoSubAction> copy() const override {
		std::unique_ptr<MovoSubAction> copiedAction(new MovoGraspAction());
		return std::move(copiedAction);
	}

	virtual std::string description() const override {
		return "GRASP ACTION";
	}

};

class MovoRetractAction: public MovoSubAction {
public:
	MovoRetractAction(std::vector<ActionSharedPtr> &actionSequence):
		MovoSubAction(),
		actionSequence_(actionSequence) {

	}

	virtual ~MovoRetractAction() = default;

	virtual MovoSubActionType getType() const override {
		return MovoSubActionType::RETRACT;
	}

	virtual std::unique_ptr<MovoSubAction> copy() const override {
		std::vector<ActionSharedPtr> copiedActionSequence(actionSequence_.size(), nullptr);
		for (size_t i = 0; i != copiedActionSequence.size(); ++i) {
			copiedActionSequence[i] = ActionSharedPtr(std::move(actionSequence_[i]->copy()));
		}

		std::unique_ptr<MovoSubAction> copiedAction(new MovoRetractAction(copiedActionSequence));
		return std::move(copiedAction);
	}

	virtual std::string description() const override {
		return "RETRACT ACTION";
	}

	std::vector<ActionSharedPtr> getActionSequence() const {
		return actionSequence_;
	}

private:
	std::vector<ActionSharedPtr> actionSequence_;
};

class MovoNoneAction: public MovoSubAction {
public:
	MovoNoneAction():
		MovoSubAction() {

	}

	virtual ~MovoNoneAction() = default;

	virtual MovoSubActionType getType() const override {
		return MovoSubActionType::NONE;
	}

	virtual std::unique_ptr<MovoSubAction> copy() const override {
		std::unique_ptr<MovoSubAction> copiedAction(new MovoNoneAction());
		return std::move(copiedAction);
	}

	virtual std::string description() const override {
		return "NONE ACTION";
	}

};

class MovoAction: public DiscreteVectorAction {
public:
	/**
	 * @brief Construct from a VectorFloat
	 */
	MovoAction(VectorFloat& actionValues):
		DiscreteVectorAction(actionValues) {

	}

	/**
	 * @brief Construct from a const VectorFloat
	 */
	MovoAction(const VectorFloat& actionValues):
		DiscreteVectorAction(actionValues) {

	}

	virtual ~MovoAction() = default;

	virtual ActionUniquePtr copy() const override {
		ActionUniquePtr copiedAction(new MovoAction(actionVec_));
		copiedAction->as<MovoAction>()->movoSubActions = MovoSubActions(movoSubActions.size(), nullptr);
		auto copiedSubAction = &(copiedAction->as<MovoAction>()->movoSubActions);
		for (size_t i = 0; i != movoSubActions.size(); ++i) {
			(*copiedSubAction)[i] = MovoSubActionSharedPtr(std::move(movoSubActions[i]->copy()));
		}

		copiedAction->as<MovoAction>()->setBinNumber(binNumber_);
		return std::move(copiedAction);
	}

	virtual ActionUniquePtr shallowCopy() const {
		ActionUniquePtr copiedAction(new MovoAction(actionVec_));

		copiedAction->as<DiscreteVectorAction>()->setBinNumber(binNumber_);
		return std::move(copiedAction);
	}

	mutable bool finishedExecuting = false;

	mutable MovoSubActions movoSubActions;
};
}

#endif