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
#ifndef _MovoGrasping_REWARD_OPTIONS_HPP_
#define _MovoGrasping_REWARD_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{

class MovoGraspingRewardOptions: public PluginOptions
{
public:
    MovoGraspingRewardOptions() = default;
    virtual ~MovoGraspingRewardOptions() = default;

    // Declaring Class Variables

    FloatType illegalMovePenalty = 0.0;    

    FloatType failedGraspPenalty = 0.0;

    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0;

    FloatType graspReward = 0.0;

    FloatType xRotationLimit = 0.0;

    FloatType yRotationLimit = 0.0;

    FloatType rotationPenalty = 0.0;

    FloatType illegalHandClosedPenalty = 0.0;

    FloatType illegalHandOpenPenalty = 0.0;

    unsigned int preferredMotionDirection = 0;

    FloatType yDistPenalty = 0.0;

    FloatType maxHorizontalGraspDist = 0.0;

    FloatType maxVerticalGraspDist = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addDefaultRewardOptions(parser.get());
        return std::move(parser);
    }

    static void addDefaultRewardOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "illegalMovePenalty",
                                     &MovoGraspingRewardOptions::illegalMovePenalty);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "stepPenalty",
                                     &MovoGraspingRewardOptions::stepPenalty);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "exitReward",
                                     &MovoGraspingRewardOptions::exitReward);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "graspReward",
                                     &MovoGraspingRewardOptions::graspReward);        
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "failedGraspPenalty",
                                     &MovoGraspingRewardOptions::failedGraspPenalty);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "xRotationLimit",
                                     &MovoGraspingRewardOptions::xRotationLimit);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "yRotationLimit",
                                     &MovoGraspingRewardOptions::yRotationLimit);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "rotationPenalty",
                                     &MovoGraspingRewardOptions::rotationPenalty);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "illegalHandClosedPenalty",
                                     &MovoGraspingRewardOptions::illegalHandClosedPenalty);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "illegalHandOpenPenalty",
                                     &MovoGraspingRewardOptions::illegalHandOpenPenalty);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "yDistPenalty",
                                     &MovoGraspingRewardOptions::yDistPenalty);
        parser->addOption<FloatType>("transitionPluginOptions",
                                        "maxHorizontalGraspDist",
                                        &MovoGraspingRewardOptions::maxHorizontalGraspDist);

        parser->addOption<FloatType>("transitionPluginOptions",
                                        "maxVerticalGraspDist",
                                        &MovoGraspingRewardOptions::maxVerticalGraspDist);

    }

};

}

#endif
