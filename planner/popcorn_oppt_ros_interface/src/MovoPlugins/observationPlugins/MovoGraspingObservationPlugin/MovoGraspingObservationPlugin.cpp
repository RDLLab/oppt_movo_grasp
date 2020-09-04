/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */


#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

// Local includes
#include "MovoGraspingObservationPluginOptions.hpp"
#include "MovoGraspingObservation.hpp"


// Include other plugin types
#include "MovoUserData.hpp"
#include "MovoUtils.hpp"

namespace oppt
{
class MovoGraspingObservationPlugin: public ObservationPlugin
{
    // Member variables of the plugin
private:
    const RobotEnvironment* robotEnvironment_;
    MovoGraspingObservationPluginOptions* observationOptions_ = nullptr;
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;
    std::unique_ptr<Distribution<FloatType>> objectPositionDistribution_;
    std::unique_ptr<Distribution<FloatType>> jointAngleErrorDistribution_;
    bool objectPoseRandomized_ = false;
    bool isExecPlugin_ = false;
    unsigned int observationSpaceDimension_ = 0;
    VectorFloat left_;
    VectorFloat right_;

public :
    MovoGraspingObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~MovoGraspingObservationPlugin() = default;



    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        parseOptions_<MovoGraspingObservationPluginOptions>(optionsFile);
        robotEnvironment_ = robotEnvironment;
        observationOptions_ =  static_cast<MovoGraspingObservationPluginOptions*>(options_.get());

        // Create the error distributions for the object's location (x,y), and the joint angles of the right arm
        makeErrorDistribution();
        observationSpaceDimension_ =
            robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions();
        return true;
    }


    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        // Create data struct for result and populate it
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        VectorFloat actionVec = observationRequest->action->as<VectorAction>()->asVector();
       
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat observationVector(observationSpaceDimension_, 0);

        // Error vector sampled from the error distributions of each dimension
        Vectordf errorVector = getErrorVector(observationRequest);
    
        // We don't perceive an object pose when we haven't executed a scan action
        observationVector[0] = std::numeric_limits<FloatType>::quiet_NaN();
        observationVector[1] = std::numeric_limits<FloatType>::quiet_NaN();
        
        
        
        // Make the observation for the joint angles
        for (size_t i = 2; i != 8; ++i) {
            observationVector[i] = stateVec[i+4] + errorVector(i);
        }

        // Make the observation for hand open/closed and grasped/not grasped flags (THESE ARE DETERMINISTIC)
        observationVector[observationVector.size() - 2] = stateVec[stateVec.size() - 2];
        observationVector[observationVector.size() - 1] = stateVec[stateVec.size() - 1];
        observationResult->observation = std::make_shared<MovoGraspingObservation>(observationVector);


        return observationResult;

    }


    /* Calculate the likelihood of seen an observation O given that the result landed on a state s' via an action a P(o|s',a)***/
    virtual FloatType calcLikelihood(const RobotStateSharedPtr& state,
                                     const Action *action,
                                     const Observation *observation) const override {
        ObservationRequestSharedPtr observationRequest(new ObservationRequest());
        VectorFloat zZero(observationSpaceDimension_, 0);
        observationRequest->currentState = state;
        observationRequest->action = action;
        observationRequest->errorVector = zZero;
        ObservationResultSharedPtr observationResult = robotEnvironment_->getRobot()->makeObservationReport(observationRequest);
        VectorFloat actionVec = action->as<VectorAction>()->asVector();
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        VectorFloat nominalObservationVec = observationResult->observation->as<VectorObservation>()->asVector();
        VectorFloat actObservationVector = observation->as<VectorObservation>()->asVector();



        /********************* CHECK FOR PERFECT SENSING IN THE GRASPING AND OPEN/CLOSE STATE OF THE HAND ******************/
        // If the observation indicates that we have a grasp, but the state says no grasp, return zero likelihood
        if (actObservationVector[actObservationVector.size() - 1] != stateVec[stateVec.size() - 1]) {
            return 0.0;
        }

        // If the observation indicates that the hand is closed, but the state says no closed, return zero likelihood
        if (actObservationVector[actObservationVector.size() - 2] != stateVec[stateVec.size() - 2]) {
            return 0.0;
        }


        //The object is assumed to be within a bounded are of uncertainty
       if (!(std::isnan(actObservationVector[0]))) {
            for (size_t i = 0; i != 2; ++i) {
                if ((actObservationVector[i] > nominalObservationVec[i] + right_[i]) ||
                        (actObservationVector[i] < nominalObservationVec[i] - right_[i])) {     
                    LOGGING("OBJECT POSE IS OUT OF UNCERTAINTY AREA");               
                    return 0.0;
                }
            }
        }


        // Now calc the likelihood of the joint angles
        VectorFloat diffVector(6, 0);
        for (size_t i = 0; i != diffVector.size(); ++i) {
            diffVector[i] = nominalObservationVec[i + 2] - actObservationVector[i + 2];
        }


        FloatType pdf = jointAngleErrorDistribution_->pdf(diffVector);
        // Return 0 if pdf is really small
        if (pdf < 1e-6) {
            LOGGING("OBSERVATION PROB IS TOO SMALL");
            printVector(nominalObservationVec, "NOMINAL");
            printVector(actObservationVector, "ACT");
            printVector(diffVector, "DIFF");
            return 0.0;
        }

        return pdf;
    }



private:
    Vectordf getErrorVector(const ObservationRequest* observationRequest) const {
        Vectordf errorVector;
       
        errorVector = Matrixdf::Zero(8, 1);
        Vectordf objectPositionError = objectPositionDistribution_->sample(1);
        Vectordf jointAnglesError = jointAngleErrorDistribution_->sample(1);
        for (size_t i = 0; i != 2; ++i) {
            errorVector[i] = objectPositionError[i];
        }
        for (size_t i = 0; i != 6; ++i) {
            errorVector[i + 2] = jointAnglesError[i];
        }

        return errorVector;
    }



    bool makeErrorDistribution() {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();

        // Make the uniform distribution of the object position
        left_ = VectorFloat({ -observationOptions_->objectPositionErrorMargin, -observationOptions_->objectPositionErrorMargin});
        right_ = VectorFloat({observationOptions_->objectPositionErrorMargin, observationOptions_->objectPositionErrorMargin});
        objectPositionDistribution_ = std::make_unique<UniformDistribution<FloatType>>(left_, right_, randomEngine);
        jointAngleErrorDistribution_ =
            std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));

        // Make the Gaussian distribution of the joint angles
        Matrixdf mean = Matrixdf::Zero(6, 1);
        Matrixdf covarianceMatrix = Matrixdf::Identity(6, 6);
        for (size_t i = 0; i != 6; ++i) {
            covarianceMatrix(i, i) = observationOptions_->jointAngleCovariance;
        }

        jointAngleErrorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        jointAngleErrorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
    }

};

OPPT_REGISTER_OBSERVATION_PLUGIN(MovoGraspingObservationPlugin)

}
