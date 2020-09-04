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
#ifndef _MOVO_TWO_GRASPING_HEURISTIC_PLUGIN_HPP_
#define _MOVO_TWO_GRASPING_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "MovoUserData.hpp"
#include "MovoUtils.hpp"
#include "MovoGraspingHeuristicOptions.hpp"

#include <fstream>
#include <cmath>

namespace oppt
{
class MovoGraspingHeuristicPlugin: public HeuristicPlugin
{

private:
	const RobotEnvironment* robotEnvironment_ = nullptr;

	std::string optionsFile_;

	gazebo::physics::Link* knuckleLink_ = nullptr;

	VectorFloat workspaceHeuristicsNoWall;
	VectorFloat workspaceHeuristicsWall;

public:
	MovoGraspingHeuristicPlugin():
		HeuristicPlugin() {

	}

	virtual ~MovoGraspingHeuristicPlugin() = default;

	virtual bool load(RobotEnvironment* const robotEnvironment,
	                  const std::string& optionsFile) override {
		robotEnvironment_ = robotEnvironment;
		optionsFile_ = optionsFile;
		parseOptions_<MovoGraspingHeuristicOptions>(optionsFile);
		knuckleLink_ = getLinkPtr("right_knuckle_link", robotEnvironment_);;
		if (!knuckleLink_)
			ERROR("Knuckle link could not be found");
		
		return true;
	}

	virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
		auto userData = static_cast<MovoGraspingTransitionPluginUserData *>(heuristicInfo->currentState->getUserData().get());
		VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
		VectorFloat actionVec = heuristicInfo->action->as<VectorAction>()->asVector();
		VectorFloat objectPosition({stateVec[0], stateVec[1], stateVec[2]});

		// Check if grasp has been achieved
		// If grasp return exit reward
		if(stateVec[stateVec.size() - 1] ==  1){
			return static_cast<MovoGraspingHeuristicOptions *>(options_.get())->exitReward;
		}

		// Use approaching heuristic until the arm is close to the object
	#ifdef GZ_GT_7
		VectorFloat knuckleLinkPosition{userData->knuckleLinkPose.Pos().X(), userData->knuckleLinkPose.Pos().Y(), userData->knuckleLinkPose.Pos().Z()};
	#else
		VectorFloat knuckleLinkPosition({userData->knuckleLinkPose.pos.x, userData->knuckleLinkPose.pos.y, userData->knuckleLinkPose.pos.z});
	#endif
		
		FloatType distPos = math::euclideanDistance(knuckleLinkPosition.data(), objectPosition.data(), 3);
		FloatType discountFactor = heuristicInfo->discountFactor;


		auto currentStep = heuristicInfo->currentStep;
		FloatType maxHeuristicValue = static_cast<MovoGraspingHeuristicOptions *>(options_.get())->maxHeuristicValue;
		FloatType heuristicScalingFactor = static_cast<MovoGraspingHeuristicOptions *>(options_.get())->heuristicScalingFactor;
		//FloatType heuristic =
		//    std::pow(discountFactor, currentStep) * maxHeuristicValue * exp(-1.0 * (heuristicScalingFactor * distPos));
		FloatType heuristic = maxHeuristicValue * exp(-1.0 * (heuristicScalingFactor * distPos));	
		

		return heuristic;

	}

	virtual HeuristicPluginSharedPtr clone(RobotEnvironment* const robotEnvironment) const override {
		std::shared_ptr<HeuristicPlugin> clonedPlugin(new MovoGraspingHeuristicPlugin());
		clonedPlugin->load(robotEnvironment, optionsFile_);
		return clonedPlugin;
	}


	

};

OPPT_REGISTER_HEURISTIC_PLUGIN(MovoGraspingHeuristicPlugin)

}

#endif
