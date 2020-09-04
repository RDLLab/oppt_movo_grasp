#ifndef _OPPT_STATE_MACHINE_HPP_
#define _OPPT_STATE_MACHINE_HPP_
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "MovoAction.hpp"
#include <popcorn_oppt_ros_interface_msgs/OPPTObservation.h>
#ifdef ENABLE_MACRO_ACTIONS
#include "plugins/transitionPlugins/MovoTransitionPlugins/PRM/PRM.hpp"
#include "plugins/transitionPlugins/MovoTransitionPlugins/IKSolutions/IKSolutionsParser.hpp"
#include "plugins/transitionPlugins/MovoTransitionPlugins/IKSolutions/PoseNearestNeighbour.hpp"
#endif

namespace oppt {
class StateMachine;

/** Abstract machine state */
class MachineState {
public:
	friend class StateMachine;
	friend class ScanAndApproachStateMachine;
	friend class GraspAndScanStateMachine;
	friend class SingleScanStateMachine;
	friend class SingleGraspStateMachine;
	MachineState() {

	}

	virtual ~MachineState() = default;

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const = 0;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const = 0;

	virtual std::string description() const = 0;

	virtual bool startFiltering() const = 0;

protected:
	StateMachine *stateMachine_ = nullptr;
};

class ApproachState: public MachineState {
public:
	friend class StateMachine;
	ApproachState();

	virtual ~ApproachState() = default;

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "ApproachState";
	}

	virtual bool startFiltering() const override;

};

class ScanStateBeforeApproach: public MachineState {
public:
	friend class StateMachine;
	ScanStateBeforeApproach();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "ScanStateBeforeApproach";
	}

	virtual bool startFiltering() const override;

};

class RetractStateBeforeApproach: public MachineState {
public:
	friend class StateMachine;
	RetractStateBeforeApproach();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "RetractStateBeforeApproach";
	}

	virtual bool startFiltering() const override;
};

class GraspState: public MachineState {
public:
	friend class StateMachine;
	GraspState();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "GraspState";
	}

	virtual bool startFiltering() const override;
};

class SingleScanState: public MachineState {
public:
	friend class StateMachine;
	SingleScanState();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "SingleScanState";
	}

	virtual bool startFiltering() const override;
};

class SingleGraspState: public MachineState {
public:
	friend class StateMachine;
	SingleGraspState();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "SingleGraspState";
	}

	virtual bool startFiltering() const override;
};

class ScanStateAfterGrasp: public MachineState {
public:
	friend class StateMachine;
	ScanStateAfterGrasp();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "ScanStateAfterGrasp";
	}

	virtual bool startFiltering() const override;
};

class RetractStateAfterGrasp: public MachineState {
public:
	friend class StateMachine;
	RetractStateAfterGrasp();

	virtual std::unique_ptr<MachineState> getNextState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::shared_ptr<MovoSubAction> generateAction(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) const override;

	virtual std::string description() const override {
		return "RetractStateAfterGrasp";
	}

	virtual bool startFiltering() const override;
};

class StateMachine {
public:
	StateMachine() {
	}

	virtual ~StateMachine() = default;

	virtual void reset() = 0;

	virtual void transitionState(const popcorn_oppt_ros_interface_msgs::OPPTObservation& observation) {
		currentState_ = currentState_->getNextState(observation);
		if (currentState_)
			currentState_->stateMachine_ = this;
	}

	virtual MachineState *const getCurrentState() const {
		if (currentState_)
			return currentState_.get();
		return nullptr;
	}

#ifdef ENABLE_MACRO_ACTIONS
	virtual void setPRM(PRM *prm) {
		prm_ = prm;
	}

	PRM *getPRM() const {
		return prm_;
	}

	void setObjectPoseNN(PoseNearestNeighbour *objectPoseNN) {
		objectPoseNN_ = objectPoseNN;
	}

	void setRetractPoseNN(PoseNearestNeighbour *retractPoseNN) {
		retractPoseNN_ = retractPoseNN;
	}

	PoseNearestNeighbour *getObjectPoseNN() const {
		return objectPoseNN_;
	}

	PoseNearestNeighbour *getRetractPoseNN() const {
		return retractPoseNN_;
	}
#endif

protected:
	std::unique_ptr<MachineState> currentState_ = nullptr;

#ifdef ENABLE_MACRO_ACTIONS
	PRM *prm_ = nullptr;

	PoseNearestNeighbour *objectPoseNN_ = nullptr;

	PoseNearestNeighbour *retractPoseNN_ = nullptr;
#endif

};

class ScanAndApproachStateMachine: public StateMachine {
public:
	ScanAndApproachStateMachine():
		StateMachine() {
		reset();

	}

	virtual void reset() override {
		currentState_ = std::unique_ptr<MachineState>(new ScanStateBeforeApproach());
		currentState_->stateMachine_ = this;
	}

};

class GraspAndScanStateMachine: public StateMachine {
public:
	GraspAndScanStateMachine():
		StateMachine() {
		reset();
	}

	virtual void reset() override {
		currentState_ = std::unique_ptr<MachineState>(new GraspState());
		currentState_->stateMachine_ = this;
	}

};

class SingleScanStateMachine: public StateMachine {
public:
	SingleScanStateMachine():
		StateMachine() {
		reset();
	}

	virtual void reset() override {
		currentState_ = std::unique_ptr<MachineState>(new SingleScanState());
		currentState_->stateMachine_ = this;
	}
};

class SingleGraspStateMachine: public StateMachine {
public:
	SingleGraspStateMachine():
		StateMachine() {
		reset();
	}

	virtual void reset() override {
		currentState_ = std::unique_ptr<MachineState>(new SingleGraspState());
		currentState_->stateMachine_ = this;
	}
};

}

#endif