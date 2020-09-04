#ifndef _PRE_FILTER_WORKER_HPP_
#define _PRE_FILTER_WORKER_HPP_
#include "oppt/opptCore/core.hpp"

using namespace oppt;
namespace solvers
{

class PreFilterWorker
{
public:
    PreFilterWorker(RobotEnvironment* const robotEnvironment) {
        preFilterThread_ = std::make_unique<boost::thread>(&PreFilterWorker::preFilterWorker_, this);
        preFilteringRunning_ = true;
        LOGGING("Cloning planning environment");
        clonedRobotEnvironment_ = robotEnvironment->clone();
        LOGGING("Cloned planning environment");
        randGen_ = clonedRobotEnvironment_->getRobot()->getRandomEngine();
    }

    ~PreFilterWorker() {
        preFilterThread_->interrupt();
        preFilterThread_->join();
    }

    void interruptPreFiltering(const bool& interrupt) {
        if (!preFilteringRunning_) {
            return;
        }
        if (interrupt) {
            mtx_.lock();
            preFilteringInterrupted_ = true;
            mtx_.unlock();
            while (true) {
                mtx_.lock();
                if (preFilteringIsInterrupted_) {
                    mtx_.unlock();
                    break;
                }

                mtx_.unlock();
                usleep(2);
            }
        } else {
            mtx_.lock();
            preFilteringInterrupted_ = false;
            mtx_.unlock();
            while (true) {
                mtx_.unlock();
                if (!preFilteringIsInterrupted_) {
                    mtx_.unlock();
                    break;
                }

                mtx_.unlock();
                usleep(2);
            }

        }
    }

    VectorParticles getPreFilteredParticles() {
        mtx_.lock();
        VectorParticles preFilteredParticles = preFilteredParticles_;
        mtx_.unlock();
        return preFilteredParticles;
    }

    void preFilter(abt::BeliefNode *const currentBelief,
                   ActionSharedPtr& action,
                   const bool &allowTerminalStates = false,
                   const bool &allowZeroWeightParticles = false) {
        if (!currentBelief) {
            WARNING("Current belief is null. Can't pre filter");
            return;
        }

        interruptPreFiltering(true);
        preFilteredParticles_.clear();
        currentAction_ = action;
        currentBelief_ = currentBelief;
        beliefStates_ = currentBelief_->getStates();
        for (size_t i = beliefStates_.size() - 1; i > -1; --i) {
            if (beliefStates_[i] == nullptr) {
                beliefStates_.erase(beliefStates_.begin() + i);
            }
        }
        if (beliefStates_.size() == 0) {
            WARNING("No belief states. can't pre filter")
            return;
        }

        previousParticles_ = VectorParticles(beliefStates_.size(), nullptr);        
        for (size_t i = 0; i != beliefStates_.size(); ++i) {
            auto robotState = static_cast<const shared::RobotState*>(beliefStates_[i]);
            previousParticles_[i] =
                std::make_shared<Particle>(robotState->getOpptState(), robotState->getWeight());

            if (!(previousParticles_[i]))
                ERROR("PARTICLE IS NULL");            
        }
        allowTerminalStates_ = allowTerminalStates;
        allowZeroWeightParticles_ = allowZeroWeightParticles;
        LOGGING("Start filtering from belief " + std::to_string(currentBelief_->getId()));
        cout << "WITH ACTION: " << *(action.get()) << endl;
        interruptPreFiltering(false);
    }

    void setParticleFilter(Filter* const particleFilter) {
        particleFilter_ = particleFilter;
    }

    void setMinNumParticles(const unsigned int& numParticles) {
        numParticles_ = numParticles;
    }

    RobotEnvironment *const getClonedRobotEnvironment() const {
        return clonedRobotEnvironment_.get();
    }

    abt::BeliefNode *const getCurrentBelief() const {
        return currentBelief_;
    }

private:
    boost::mutex mtx_;

    bool preFilteringIsInterrupted_ = false;

    bool preFilteringInterrupted_ = false;

    std::unique_ptr<boost::thread> preFilterThread_ = nullptr;

    VectorParticles preFilteredParticles_;

    bool preFilteringRunning_ = false;

    ActionSharedPtr currentAction_ = nullptr;

    RobotEnvironmentSharedPtr clonedRobotEnvironment_ = nullptr;

    abt::BeliefNode* currentBelief_ = nullptr;

    std::vector<abt::State const*> beliefStates_;

    VectorParticles previousParticles_;    

    RandomEnginePtr randGen_ = nullptr;

    Filter* particleFilter_ = nullptr;

    long numParticles_;

    bool allowTerminalStates_ = false;

    bool allowZeroWeightParticles_ = false;

private:
    void preFilterWorker_() {
        try {
            bool preFilteringInterrupted = false;
            while (true) {
                mtx_.lock();
                preFilteringInterrupted = preFilteringInterrupted_;
                mtx_.unlock();
                if (!preFilteringInterrupted) {
                    mtx_.lock();
                    preFilteringIsInterrupted_ = false;
                    long preFilteredParticleSize = preFilteredParticles_.size();
                    mtx_.unlock();
                    if (preFilteredParticleSize < numParticles_) {
                        filter_(10);
                    }

                    boost::this_thread::interruption_point();
                } else {
                    mtx_.lock();
                    preFilteringIsInterrupted_ = true;
                    mtx_.unlock();
                    boost::this_thread::interruption_point();
                }
            }

        } catch (boost::thread_interrupted&) {}

        preFilteringIsInterrupted_ = true;
        preFilteringRunning_ = false;
    }

    void filter_(const unsigned int& numParticles) {
        if (currentAction_) {
            if (!currentBelief_) {
                WARNING("CurrentBelief is null");
                return;
            }
            FilterRequestPtr filterRequest(new FilterRequest());
            filterRequest->robotEnvironment = clonedRobotEnvironment_.get();
            filterRequest->previousParticles = previousParticles_;            
            filterRequest->numParticles = numParticles;
            filterRequest->allowTerminalStates = false;
            filterRequest->allowZeroWeightParticles = allowZeroWeightParticles_;
            if (filterRequest->previousParticles.size() == 0) {
                cout << "Belief id: " << currentBelief_->getId() << endl;
                ERROR("Previous particles are empty");
            }

            filterRequest->action = currentAction_.get();
            filterRequest->randomEngine = randGen_;
            auto filterResult = static_cast<ParticleFilter*>(particleFilter_)->propagateParticles(filterRequest);
            mtx_.lock();
            preFilteredParticles_.insert(preFilteredParticles_.end(), 
                filterResult->particles.begin(), 
                filterResult->particles.end());            
            if (filterResult->particles.size() == 0)
                WARNING("No particles");
            mtx_.unlock();
        }
    }

};
}

#endif
