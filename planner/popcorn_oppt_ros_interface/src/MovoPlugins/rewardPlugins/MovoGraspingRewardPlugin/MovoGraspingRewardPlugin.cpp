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

// Plugin includes
#include "MovoGraspingRewardOptions.hpp"
#include "MovoUserData.hpp"
#include "MovoUtils.hpp"


namespace oppt
{
class MovoGraspingRewardPlugin: public RewardPlugin
{
public :
    MovoGraspingRewardPlugin():
        RewardPlugin() {

    }

    virtual ~MovoGraspingRewardPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment,
                      const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;
        parseOptions_<MovoGraspingRewardOptions>(optionsFile);
        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
        
        return getRewardBeforeGrasp(propagationResult);
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-static_cast<MovoGraspingRewardOptions*>(options_.get())->illegalMovePenalty,
                              static_cast<MovoGraspingRewardOptions*>(options_.get())->exitReward);
    }

    virtual void setUserData(const OpptUserDataSharedPtr& userData) {
    }

private:
    const RobotEnvironment* robotEnvironment_;

private:
    FloatType getRewardBeforeGrasp(const PropagationResultSharedPtr& propagationResult) const {
        auto userData =
            static_cast<MovoGraspingTransitionPluginUserData*>(propagationResult->nextState->getUserData().get());
        auto userDataPrevious = static_cast<MovoGraspingTransitionPluginUserData*>(propagationResult->previousState->getUserData().get());
        
        VectorFloat previousStateVec = propagationResult->previousState->as<VectorState>()->asVector();
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();

        FloatType r = -static_cast<MovoGraspingRewardOptions*>(options_.get())->stepPenalty;


         /*** If the grasp was achieved. Return the exit reward ***/
        if(stateVec[stateVec.size() - 1] == 1)
        {
            r += static_cast<MovoGraspingRewardOptions*>(options_.get())->graspReward;
            return r;
        }


        /*************** Initial penalty for taking a step ******************/
        

        bool tableCollision=false;
        bool fingerCollision=false;
        if (userData->collisionReport->collides)
        {
            for (auto &collisionPair: userData->collisionReport->collisionPairs) 
                {
                    if ((collisionPair.first.compare("box1::box1Link") == 0) || (collisionPair.second.compare("box1::box1Link") == 0))
                    {
                        tableCollision=true;
                        std::cout << "Collision with table in particle" << std::endl;
                        break;  
                    }
                }
        }

        
        /*************** Additional penalties ******************************/
        // Additional penalty when robot collides with the table 
        if (tableCollision) {
            r -= 3.0 * static_cast<MovoGraspingRewardOptions*>(options_.get())->illegalMovePenalty;
        }

        if (userData->illegalFingerCollision)
        {
            r -= 12.0 * static_cast<MovoGraspingRewardOptions*>(options_.get())->illegalMovePenalty;   
        }

        /*************** Addtional penalty for not opening the hand after a failed grasp *******************/
        if (previousStateVec[previousStateVec.size() - 2] == 1 && previousStateVec[previousStateVec.size() - 1] == 0 &&
                actionVec[actionVec.size() - 1] != -1) {
            r -= static_cast<MovoGraspingRewardOptions*>(options_.get())->illegalHandClosedPenalty;
        }

        /************ Additional penalty for opening hand when having a good grasp ***********************/
        if ((previousStateVec[previousStateVec.size() - 1] == 1) && (actionVec[actionVec.size() - 1] == -1)) {
            r -= static_cast<MovoGraspingRewardOptions*>(options_.get())->illegalHandOpenPenalty;
        }


        /*** Additional penalty for openning the gripper when there is no grasp ***/
        if(actionVec[actionVec.size() - 1] == -1 && previousStateVec[previousStateVec.size() - 2] != 1) {
             r -= static_cast<MovoGraspingRewardOptions*>(options_.get())->illegalHandOpenPenalty;
        } 


        /*********** Addtional penaly for rotating out of limits for the end effector **********************/
    #ifdef GZ_GT_7
        auto handOrientation = userData->knuckleLinkPose.Rot().Euler();
        auto handOrientationX = handOrientation.X();
        auto handOrientationY = handOrientation.Y();
    #else
        auto handOrientation = userData->knuckleLinkPose.rot.GetAsEuler();
        auto handOrientationX = handOrientation.x;
        auto handOrientationY = handOrientation.y;
    #endif

        if (std::fabs(handOrientationX) > static_cast<MovoGraspingRewardOptions*>(options_.get())->xRotationLimit ||
                std::fabs(handOrientationY) > static_cast<MovoGraspingRewardOptions*>(options_.get())->yRotationLimit) {
            r -= static_cast<MovoGraspingRewardOptions*>(options_.get())->rotationPenalty;
        }
        

        /*********** Additional penalty for having hand far from object ************************/
        // FloatType distance = sqrt((userData->distY * userData->distY) + (userData->distX * userData->distX));
        // if (distance > static_cast<MovoGraspingRewardOptions*>(options_.get())->maxHorizontalGraspDist) 
        // {
        //     r -= static_cast<MovoGraspingRewardOptions*>(options_.get())->yDistPenalty;
        // }


       return r;
    }   

};

OPPT_REGISTER_REWARD_PLUGIN(MovoGraspingRewardPlugin)

}
