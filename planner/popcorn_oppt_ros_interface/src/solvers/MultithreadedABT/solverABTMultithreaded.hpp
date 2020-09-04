#ifndef __SOLVER_GEN_HPP_
#define __SOLVER_GEN_HPP_
#include <oppt/solver/solver.hpp>
#include "MultithreadedABTOptions.hpp"
#include "ABT/solverABT.hpp"
#include "ABT/solver/BeliefNode.hpp"
#include "PreFilterWorker.hpp"
#include "MovoUserData.hpp"

using namespace oppt;
namespace solvers
{

enum SOLVER_TYPE {
    graspSolver,
    approachSolver,
    scoopingSolver,
    placeSolver
};

class ABTMultithreaded: public Solver
{
public:
    ABTMultithreaded():
        Solver() {
        this->solverName_ = "ABTMultithreaded";
    }

    virtual ~ABTMultithreaded() = default;

    virtual void setup() override {
        // Setup ABT
        abt_ = std::make_unique<ABT>();
        abt_->robotPlanningEnvironment_ = robotPlanningEnvironment_;
        abt_->problemEnvironmentOptions_ = problemEnvironmentOptions_;
        abt_->registerForEnvironmentChangesFn_ = registerForEnvironmentChangesFn_;
        abt_->loadHeuristicPlugin("libmovoGraspingHeuristicPlugin.so");
        abt_->randGen_ = randGen_;
        abt_->setup();

        //unregisterForEnvironmentChanges(robotPlanningEnvironment_);

        preFilterWorker_ = std::make_unique<PreFilterWorker>(robotPlanningEnvironment_);
        auto clonedRobotEnvironment = preFilterWorker_->getClonedRobotEnvironment();
        registerForEnvironmentChanges(clonedRobotEnvironment, robotPlanningEnvironment_);
        registerForEnvironmentChanges(robotPlanningEnvironment_);

        // In case we made any changes, apply them now so they are applied
        // to the cloned environment
        LOGGING("CALL APPLY CHANGES PLANNING");

        robotPlanningEnvironment_->applyChanges();
        maxNumHistories_ = static_cast<MultithreadedABTOptions*>(problemEnvironmentOptions_)->maxNumHistories;
        currentAction_ = nullptr;
        solverType_ = SOLVER_TYPE::graspSolver;
        LOGGING("CHANGES APPLIED");
    }

    virtual bool reset() override {
        currentAction_ = nullptr;
        if (abt_->reset()) {
            LOGGING("Resetting solver");
            boost::timer t0;
            abt_->improvePolicy(-1, 1);
            cout << "time to improve policy: " << t0.elapsed() << endl;
            currentBelief_ = abt_->simulator_->getAgent()->getCurrentBelief();
            sampler_ = abt_->solver_->getStateSampler(currentBelief_);
            numHistories_ = 0;
            planningThread_ = std::make_unique<boost::thread>(&ABTMultithreaded::planningThread, this);
            planningThreadRunning_ = true;
            interruptPlanning(false);
            auto robotModel = abt_->getRobotModel();
            if (!robotModel)
                ERROR("Robot model is null");
            preFilterWorker_->setParticleFilter(abt_->getParticleFilter());
            long minNumParticles = static_cast<MultithreadedABTOptions*>(problemEnvironmentOptions_)->minParticleCount;
            preFilterWorker_->setMinNumParticles(minNumParticles);
            LOGGING("pre filter worker setup");
            LOGGING("Current belief ID: " + std::to_string(currentBelief_->getId()));
            currentState_ = robotPlanningEnvironment_->sampleInitialState();
            return true;
        }
        return false;

    }

    bool resetWithoutStartingPlanningThread() {
        currentAction_ = nullptr;
        if (abt_->reset()) {
            LOGGING("Resetting solver");
            abt_->improvePolicy(-1, 1);
            currentBelief_ = abt_->simulator_->getAgent()->getCurrentBelief();
            sampler_ = abt_->solver_->getStateSampler(currentBelief_);
            numHistories_ = 0;
            auto robotModel = abt_->getRobotModel();
            if (!robotModel)
                ERROR("Robot model is null");
            preFilterWorker_->setParticleFilter(abt_->getParticleFilter());
            long minNumParticles = static_cast<MultithreadedABTOptions*>(problemEnvironmentOptions_)->minParticleCount;
            preFilterWorker_->setMinNumParticles(minNumParticles);
            LOGGING("pre filter worker setup");
            LOGGING("Current belief ID: " + std::to_string(currentBelief_->getId()));
            currentState_ = robotPlanningEnvironment_->sampleInitialState();
            return true;
        }

        return false;
    }

    /**
     * @brief Interrupts or continues the planning thrad
     * @param interrupt If true, interrupt the planning thread. Otherwise continue it
     */
    void interruptPlanning(const bool& interrupt) {
        if (!planningThreadRunning_)
            return;
        if (interrupt) {
            mtx_.lock();
            planningInterrupted_ = true;
            //cout << "planningInterrupted_1: " << planningInterrupted_ << endl;
            mtx_.unlock();
            while (true) {
                mtx_.lock();
                if (isInterrupted_ || !planningThreadRunning_) {
                    mtx_.unlock();
                    break;
                }

                mtx_.unlock();
                usleep(2);
            }
        } else {
            mtx_.lock();
            planningInterrupted_ = false;
            mtx_.unlock();
            while (true) {
                mtx_.lock();
                if (!isInterrupted_ || !planningThreadRunning_) {
                    mtx_.unlock();
                    break;
                }

                mtx_.unlock();
                usleep(2);
            }
        }
    }

    /**
     * @brief The planning thread that improves the policy in a separate thread
     */
    void planningThread() {
        try {
            bool planningInterrupted = false;
            while (true) {
                mtx_.lock();
                planningInterrupted = planningInterrupted_;
                //cout << "planningInterrupted_0: " << planningInterrupted_ << endl;
                mtx_.unlock();
                if (!planningInterrupted) {
                    mtx_.lock();
                    isInterrupted_ = false;
                    mtx_.unlock();
                    if (!sampler_) {
                        WARNING("No valid state sampler. Exiting planning thread");
                        break;
                    }

                    if (numHistories_ < maxNumHistories_) {
                        if (currentAction_) {
                            //cout << "solve with action: " << currentAction_.get() << endl;
                            abt_->solver_->singleSearch(currentBelief_, sampler_(), 100000, currentAction_.get());
                        }
                        else {
                            abt_->solver_->singleSearch(currentBelief_, sampler_(), 100000, nullptr);
                        }
                        numHistories_++;
                        //LOGGING("Num histories: " + std::to_string(numHistories_) + "/" + std::to_string(maxNumHistories_));
                    }

                    //boost::this_thread::interruption_point();
                } else {
                    mtx_.lock();
                    isInterrupted_ = true;
                    mtx_.unlock();
                    boost::this_thread::interruption_point();
                }
                //sleep(1);
            }
        } catch (boost::thread_interrupted&) {}
        mtx_.lock();
        isInterrupted_ = true;
        planningThreadRunning_ = false;
        mtx_.unlock();
    }

    virtual bool improvePolicy(const FloatType& timeout) override {
        if (!planningThreadRunning_)
            return false;
        return true;
    }

    virtual bool updateBelief(const ActionSharedPtr& action,
                              const ObservationSharedPtr& observation,
                              const bool &allowTerminalStates = false) override {
        if (!planningThreadRunning_) {
            WARNING("Planning thread not running. Belief could not be updated");
            return false;
        }

        interruptPlanning(true);
        bool beliefUpdated = updatePreFilteredBelief_(observation, allowTerminalStates);
        currentAction_ = nullptr;
        if (beliefUpdated) {
            currentBelief_ = abt_->simulator_->getAgent()->getCurrentBelief();
            sampler_ = abt_->solver_->getStateSampler(currentBelief_);
            if (!sampler_ && !allowTerminalStates) {
                if (switchSolver_)
                    return true;
                return false;
            }
            cout << "numHistories: " << numHistories_ << endl;
            numHistories_ = 0;
            cout << "start planning from belief " << currentBelief_->getId() << endl;
            interruptPlanning(false);
        }
        return beliefUpdated;
    }

    virtual ActionSharedPtr getNextAction() override {
        interruptPlanning(true);
        abt_->solver_->doBackup();
        auto model = static_cast<robot::RobotModel*>(abt_->getSimulator()->getModel());
        cout << "Getting action from belief #" << abt_->getSimulator()->getAgent()->getCurrentBelief() << endl;

        cout << "Belief: " << abt_->getSimulator()->getAgent()->getCurrentBelief() << endl;
        abt_->getSolver()->printBelief(abt_->getSimulator()->getAgent()->getCurrentBelief(), std::cout);




        currentAction_ = std::move(abt_->getSimulator()->getAgent()->getPreferredAction());
        oppt::ActionSharedPtr opptAction = model->getBaseAction(*(currentAction_.get()));
        preFilterWorker_->preFilter(currentBelief_,
                                    opptAction,
                                    false,
                                    static_cast<MultithreadedABTOptions*>(problemEnvironmentOptions_)->allowZeroWeightParticles);
        /////////////////////////////
        // For Debugging purposes
        /////////////////////////////
        PropagationRequestSharedPtr propagationRequest(new PropagationRequest());
        propagationRequest->currentState = currentState_;
        propagationRequest->action = opptAction.get();

        auto propRes = robotPlanningEnvironment_->getRobot()->propagateState(propagationRequest);
        currentState_ = propRes->nextState;
        ObservationRequestSharedPtr observationRequest = std::make_shared<ObservationRequest>();
        observationRequest->currentState = currentState_;
        observationRequest->action = opptAction.get();
        ObservationResultSharedPtr observationResult =
            robotPlanningEnvironment_->getRobot()->makeObservationReport(observationRequest);
        currentObservation_ = observationResult->observation;
        VectorFloat actionVec = opptAction->as<VectorAction>()->asVector();
        auto ud = static_cast<MovoTransitionPluginUserData *>(currentState_->getUserData().get());
        if (actionVec[actionVec.size() - 1] > 0.5) {
            VectorFloat stateVec = currentState_->as<oppt::VectorState>()->asVector();
            VectorFloat objectPos({stateVec[0], stateVec[1], stateVec[2]});
            VectorFloat handPos({stateVec[6], stateVec[7], stateVec[8]});
        }
        //cout << "num finger object collisions: " << ud->numFingerObjectCollisions << endl;
        //cout << "illegalFingerCollision: " << ud->illegalFingerCollisionOccured << endl;

        //cout << "dist x angle: " << ud->distanceXAngle << endl;
        //cout << "dist y angle: " << ud->distanceYAngle << endl;
        //printVector(ud->jointAngles, "jointAngles");
        //printBelief(getCurrentBelief());
        //getchar();

        auto actionNode = currentBelief_->getMapping()->getActionNode(*(currentAction_.get()));
        if (actionNode) {
            auto obsMapping = actionNode->getMapping();
            if (obsMapping) {
                cout << "CHILD OBSERVATIONS: " << obsMapping->getNChildren() << endl;
                cout << "visit count: " << obsMapping->getTotalVisitCount() << endl;
            }
        }

        /////////////////////////////
        interruptPlanning(false);
        cout << "belief id getNextAction: " << currentBelief_->getId() << endl;
        return opptAction;
    }

    virtual VectorRobotStatePtr getBeliefParticles() override {
        interruptPlanning(true);
        auto beliefParticles =  abt_->getBeliefParticles();
        if (!switchSolver_)
            interruptPlanning(false);
        return beliefParticles;
    }

    virtual void handleEnvironmentChanges(const std::vector<EnvironmentChangeSharedPtr>& environmentChanges) override {
        abt_->handleEnvironmentChanges(environmentChanges);
    }

    virtual void serializeStep(std::ofstream& os) override {
        interruptPlanning(true);
        abt_->serializeStep(os);
        if (!switchSolver_) {
            interruptPlanning(false);
        }
    }

    virtual void stepFinished(const size_t &step) override {
        if (switchSolver_) {
            switchSolver();
            switchSolver_ = false;
            //interruptPlanning(false);
        }
    }

    virtual void runFinished(std::ofstream& os, const unsigned int& run) override {
        // Finish the planning thread
        if (planningThreadRunning_) {
            interruptPlanning(true);
            planningThread_->interrupt();
            planningThread_->join();
        }
    }

    void printBelief(abt::BeliefNode* belief) const {
        std::stringstream newStream;
        abt_->getSolver()->printBelief(belief, newStream);
        cout << "BELIEF: " << endl;
        while (newStream.good()) {
            std::string s1, s2;
            std::getline(newStream, s2);
            cout << s2 << std::setw(40 - s2.size()) << "";
            cout << endl;
        }
    }

    abt::BeliefNode *const getCurrentBelief() const {
        return currentBelief_;
    }

    PreFilterWorker *const getPreFilterWorker() const {
        return preFilterWorker_.get();
    }

    RobotStateSharedPtr getCurrentState() const {
        return currentState_;
    }

    ObservationSharedPtr getCurrentObservation() const {
        return currentObservation_;
    }

    bool switchingSolver() const {
        return switchSolver_;
    }

    bool switchToGraspingSolver() {
        LOGGING("Switch to grasping plugins");
        solverType_ = SOLVER_TYPE::graspSolver;
        problemEnvironmentOptions_->initialBeliefPluginPlanning = "libmovoInitialBeliefPlugin.so";
        problemEnvironmentOptions_->planningTerminalPlugin = "libmovoGraspingTerminalPlugin.so";
        problemEnvironmentOptions_->heuristicPlugin = "libmovoGraspingHeuristicPlugin.so";
        problemEnvironmentOptions_->planningRewardPlugin = "libmovoGraspingRewardPlugin.so";
        loadPlugins_(true, false);
        OpptUserDataSharedPtr userData(new MovoTransitionPluginUserData());
        static_cast<MovoTransitionPluginUserData *>(userData.get())->scoopingPhase = false;
        robotPlanningEnvironment_->getRobot()->getTransitionPlugin()->setUserData(userData);
        preFilterWorker_->getClonedRobotEnvironment()->getRobot()->getTransitionPlugin()->setUserData(userData);
        return true;
    }

private:
    std::unique_ptr<ABT> abt_ = nullptr;

    std::unique_ptr<abt::Action> currentAction_ = nullptr;

    abt::BeliefNode* currentBelief_ = nullptr;

    /**
     * @brief Flag for the planning thread to notify it that it has to interrupt planning
     */
    bool planningInterrupted_ = false;


    /**
     * @brief Determines if the planning thread is interrupted
     */
    bool isInterrupted_ = false;

    /**
     * @brief std::unique_ptr to the planning thread
     */
    std::unique_ptr<boost::thread> planningThread_ = nullptr;

    /**
     * @brief Flag to indicate if the planning thread is still running
     */
    bool planningThreadRunning_ = false;

    unsigned int numHistories_ = 0;

    size_t maxNumHistories_ = 0;

    std::function<abt::StateInfo *()> sampler_ = nullptr;

    boost::mutex mtx_;

    std::unique_ptr<PreFilterWorker> preFilterWorker_;

    RobotStateSharedPtr currentState_ = nullptr;

    ObservationSharedPtr currentObservation_ = nullptr;

    bool switchSolver_ = false;

    SOLVER_TYPE solverType_;

private:
    bool switchSolver() {
        LOGGING("Switching solver");
        boost::timer t0;
        interruptPlanning(true);
        preFilterWorker_->interruptPreFiltering(true);
        auto solverType = solverType_;
        cout << "solverType: " << solverType_ << endl;
        switch (solverType) {
        case SOLVER_TYPE::graspSolver:
            switchToApproachSolver();
            break;
        case SOLVER_TYPE::approachSolver:
            switchToScoopingSolver();
            break;
        case SOLVER_TYPE::scoopingSolver:
            switchToPlaceSolver();
            break;
        default:
            switchToGraspingSolver();
            break;
        }

        if (!resetWithoutStartingPlanningThread())
            return false;

        LOGGING("Switched solver");
        cout << "time to switch: " << t0.elapsed() << endl;
        interruptPlanning(false);
        boost::this_thread::sleep_for(boost::chrono::milliseconds((unsigned int)2000));
        return true;
        //switchToScoopingSolver();
    }

    bool switchToApproachSolver() {
        LOGGING("Switch to approach plugins");
        solverType_ = SOLVER_TYPE::approachSolver;
        problemEnvironmentOptions_->initialBeliefPluginPlanning = "libmovoInitialBeliefPluginScooping.so";
        problemEnvironmentOptions_->planningTerminalPlugin = "libmovoApproachTerminalPlugin.so";
        problemEnvironmentOptions_->heuristicPlugin = "libmovoApproachHeuristicPlugin.so";
        problemEnvironmentOptions_->planningRewardPlugin = "libmovoApproachRewardPlugin.so";
        loadPlugins_(true);
        OpptUserDataSharedPtr userData(new MovoTransitionPluginUserData());
        static_cast<MovoTransitionPluginUserData *>(userData.get())->scoopingPhase = false;
        robotPlanningEnvironment_->getRobot()->getTransitionPlugin()->setUserData(userData);
        preFilterWorker_->getClonedRobotEnvironment()->getRobot()->getTransitionPlugin()->setUserData(userData);
        return true;
    }

    bool switchToScoopingSolver() {
        LOGGING("Switch to scooping plugins");
        solverType_ = SOLVER_TYPE::scoopingSolver;
        problemEnvironmentOptions_->initialBeliefPluginPlanning = "libmovoInitialBeliefPluginScooping.so";
        problemEnvironmentOptions_->planningTerminalPlugin = "libmovoScoopingTerminalPlugin.so";
        problemEnvironmentOptions_->heuristicPlugin = "libmovoScoopingHeuristicPlugin.so";
        problemEnvironmentOptions_->planningRewardPlugin = "libmovoScoopingRewardPlugin.so";
        loadPlugins_(true);
        OpptUserDataSharedPtr userData(new MovoTransitionPluginUserData());
        static_cast<MovoTransitionPluginUserData *>(userData.get())->scoopingPhase = true;
        robotPlanningEnvironment_->getRobot()->getTransitionPlugin()->setUserData(userData);
        preFilterWorker_->getClonedRobotEnvironment()->getRobot()->getTransitionPlugin()->setUserData(userData);
        return true;
    }

    bool switchToPlaceSolver() {
        LOGGING("Switch to place plugins");
        solverType_ = SOLVER_TYPE::placeSolver;
        problemEnvironmentOptions_->initialBeliefPluginPlanning = "libmovoInitialBeliefPluginScooping.so";
        problemEnvironmentOptions_->planningTerminalPlugin = "libmovoPlaceTerminalPlugin.so";
        problemEnvironmentOptions_->heuristicPlugin = "libmovoPlaceHeuristicPlugin.so";
        problemEnvironmentOptions_->planningRewardPlugin = "libmovoPlaceRewardPlugin.so";
        loadPlugins_(true, true);
        OpptUserDataSharedPtr userData(new MovoTransitionPluginUserData());
        static_cast<MovoTransitionPluginUserData *>(userData.get())->scoopingPhase = false;
        robotPlanningEnvironment_->getRobot()->getTransitionPlugin()->setUserData(userData);
        preFilterWorker_->getClonedRobotEnvironment()->getRobot()->getTransitionPlugin()->setUserData(userData);
        return true;
    }

    bool loadPlugins_(const bool &loadInitialBeliefPlugin = true, const bool &randomizeObjectPose = false) {
        robotPlanningEnvironment_->loadTerminalPlugin(problemEnvironmentOptions_->planningTerminalPlugin,
                problemEnvironmentOptions_->configPath);
        if (loadInitialBeliefPlugin) {
            robotPlanningEnvironment_->loadInitialBeliefPlugin(problemEnvironmentOptions_->initialBeliefPluginPlanning,
                    problemEnvironmentOptions_->configPath);
        }
        robotPlanningEnvironment_->loadRewardPlugin(problemEnvironmentOptions_->planningRewardPlugin,
                problemEnvironmentOptions_->configPath);

        abt_->loadHeuristicPlugin("libmovoGraspingHeuristicPlugin.so");

        // Set user data in the initial belief plugin, unless we use the grasping plugin
        if (problemEnvironmentOptions_->initialBeliefPluginPlanning.find("libmovoInitialBeliefPlugin.so") ==
                std::string::npos) {
            oppt::OpptUserDataSharedPtr initialBeliefPluginUserData(new MovoInitialBeliefPluginScoopingUserData());
            auto initialBeliefPluginUD =
                static_cast<MovoInitialBeliefPluginScoopingUserData *>(initialBeliefPluginUserData.get());
            auto currentBelief = abt_->getSimulator()->getAgent()->getCurrentBelief();
            auto beliefStates = currentBelief_->getNonTerminalStates();
            if (beliefStates.size() == 0)
                ERROR("Current belief contains no non-terminal states. Aborting");
            initialBeliefPluginUD->beliefParticles = VectorRobotStatePtr(beliefStates.size(), nullptr);
            initialBeliefPluginUD->randomizeObjectPose = randomizeObjectPose;
            for (size_t i = 0; i != beliefStates.size(); ++i) {
                initialBeliefPluginUD->beliefParticles[i] =
                    static_cast<const shared::RobotState*>(beliefStates[i])->getOpptState();
            }

            robotPlanningEnvironment_->getInitialBeliefPlugin()->setUserData(initialBeliefPluginUserData);

            OpptUserDataSharedPtr observationPluginUserData(new MovoObservationPluginUserData());
            static_cast<MovoObservationPluginUserData *>(observationPluginUserData.get())->objectPoseRandomized =
                randomizeObjectPose;
            robotPlanningEnvironment_->getRobot()->getObservationPlugin()->setUserData(observationPluginUserData);
        }
        return true;
    }

    bool updatePreFilteredBelief_(const ObservationSharedPtr& observation,
                                  const bool &allowTerminalStates = false) {
        shared::RobotObservation robotObservation(observation);
        preFilterWorker_->interruptPreFiltering(true);
        VectorParticles preFilteredParticles = preFilterWorker_->getPreFilteredParticles();

        // New filter request
        FilterRequestPtr filterRequest = std::make_unique<FilterRequest>();

        LOGGING("pre filtered particles size: " + std::to_string(preFilteredParticles.size()));
        if (preFilteredParticles.size() == 0) {
            WARNING("Couldn't update pre filtered particles. Size is 0");
            return false;
        }

        // Get the next belief node based on the action and observation
        abt::BeliefNode* nextBelief = currentBelief_->createOrGetChild(*(currentAction_.get()), robotObservation);

        // Fill in the particles of the next belief that were obtained during planning
        std::vector<abt::State const*> nextStates = nextBelief->getStates();
        for (size_t i = 0; i != nextStates.size(); ++i) {
            auto st = static_cast<const shared::RobotState*>(nextStates[i]);
            filterRequest->currentNextParticles.push_back(std::make_shared<Particle>(st->getOpptState(), st->getWeight()));
        }

        // Fill in the particles of the next belief that were obtained by the PreFilterGenerator
        filterRequest->currentNextParticles.insert(filterRequest->currentNextParticles.end(), preFilteredParticles.begin(), preFilteredParticles.end());



        filterRequest->robotEnvironment = robotPlanningEnvironment_;
        filterRequest->currentNextParticles = preFilteredParticles;
        filterRequest->allowTerminalStates = false;
        filterRequest->allowZeroWeightParticles = false;
        filterRequest->numParticles = preFilteredParticles.size();
        filterRequest->action = (abt_->getRobotModel()->getBaseAction(*(currentAction_.get()))).get();
        filterRequest->observation = observation.get();
        filterRequest->allowCollisions = robotPlanningEnvironment_->getRobot()->getCollisionsAllowed();
        filterRequest->randomEngine = robotPlanningEnvironment_->getRobot()->getRandomEngine();

        // Do the filtering of the pre-generated particles
        FilterResultPtr filterResult = abt_->getParticleFilter()->filter(filterRequest);

        if (!filterResult)
            return false;

        if (filterResult->particles.empty())
            return false;

        VectorParticles resultingParticles = filterResult->particles;
        bool beliefUpdated = abt_->updateNextBelief_(std::move(filterResult), nextBelief);
        if (!beliefUpdated)
            return false;

        // Now check how many terminal states we have after the filtering
        PropagationResultSharedPtr propResult(new PropagationResult());
        FloatType numTerminalStates = 0.0;
        for (size_t i = 0; i != resultingParticles.size(); ++i) {
            propResult->nextState = resultingParticles[i]->getState();
            if (robotPlanningEnvironment_->isValid(propResult) &&
                    robotPlanningEnvironment_->isTerminal(propResult))
                numTerminalStates += 1.0;
        }

        // If at least 95% of the particles in the next node are terminal,
        // we switch the solver
        FloatType conf = numTerminalStates / (FloatType)(resultingParticles.size());
        if (conf > 0.95) {
            LOGGING("================================================");
            LOGGING("SWITCH SOLVER PLEASE");
            LOGGING("================================================");
            switchSolver_ = true;
        }

        // Update the agent's belief.
        abt_->getSimulator()->getAgent()->updateBelief(*(currentAction_.get()), robotObservation);
        auto beliefAgent = abt_->getSimulator()->getAgent()->getCurrentBelief();
        if (beliefAgent != nextBelief)
            ERROR("Updated belief obtained from agent is different to the next belief");

        return true;
    }
};

}

#endif
