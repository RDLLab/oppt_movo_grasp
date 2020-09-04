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
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "MovoUserData.hpp"
#include "MovoUtils.hpp"
#include "MovoGraspingTerminalOptions.hpp"
#include <boost/timer.hpp>

namespace oppt
{
class MovoGraspingTerminalPlugin: public TerminalPlugin
{
public :
    MovoGraspingTerminalPlugin():
        TerminalPlugin() {

    }

    virtual ~MovoGraspingTerminalPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;
        parseOptions_<MovoTerminalOptions>(optionsFile);        
  
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        ValidityReportSharedPtr validityReport(new ValidityReport(propagationResult->nextState));
        auto userData = static_cast<MovoGraspingTransitionPluginUserData*>(propagationResult->nextState->getUserData().get());
        validityReport->satisfiesConstraints = true;
        validityReport->isValid = true;
        if (userData->collisionReport->collides) {
            validityReport->collided = true;
            validityReport->isValid = false;
        }

        return validityReport;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {


        auto userData = static_cast<MovoGraspingTransitionPluginUserData*>(propagationResult->nextState->getUserData().get());

        // Checking for collision with table

        if (userData->collisionReport->collides) {

            for (auto &collisionPair: userData->collisionReport->collisionPairs) 
            {

                if ((collisionPair.first.compare("box1::box1Link") == 0) || (collisionPair.second.compare("box1::box1Link") == 0))
                    {

                        return true;
                    }
            }
        }        

        // Terminal when we're doing a grasp?
        VectorFloat stateVec =
            propagationResult->nextState->as<VectorState>()->asVector();
        if(stateVec[stateVec.size() - 1] == 1){
            return true;
        }

        // Terminal when we open the hand while having a grasp
        if (userData->illegalHandOpening)
            return true;



        return false;
    }

private:
    const RobotEnvironment* robotEnvironment_;   


};

OPPT_REGISTER_TERMINAL_PLUGIN(MovoGraspingTerminalPlugin)

}




