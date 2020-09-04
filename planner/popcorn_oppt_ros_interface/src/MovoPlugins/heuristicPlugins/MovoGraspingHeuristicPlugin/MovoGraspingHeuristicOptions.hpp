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
#ifndef _MOVO_HEURISTIC_OPTIONS_HPP_
#define _MOVO_HEURISTIC_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class MovoGraspingHeuristicOptions: public PluginOptions
{
public:
    MovoGraspingHeuristicOptions() = default;
    virtual ~MovoGraspingHeuristicOptions() = default;

    // Class variable declaration

    FloatType illegalMovePenalty = 0.0;    

    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0;

    FloatType maxHeuristicValue = 0.0;

    FloatType heuristicScalingFactor = 0.0;

    VectorFloat enteringPose;

    VectorFloat exitPose;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addMovoGraspingHeuristicOptions(parser.get());
        return std::move(parser);
    }

    static void addMovoGraspingHeuristicOptions(options::OptionParser* parser) {

        // Reading values from configuration file

        parser->addOption<FloatType>("rewardPluginOptions",
                                     "illegalMovePenalty",
                                     &MovoGraspingHeuristicOptions::illegalMovePenalty);        
        parser->addOption<FloatType>("rewardPluginOptions", "stepPenalty", &MovoGraspingHeuristicOptions::stepPenalty);
        parser->addOption<FloatType>("rewardPluginOptions", "exitReward", &MovoGraspingHeuristicOptions::exitReward);
        parser->addOption<FloatType>("heuristicPluginOptions",
                                     "maxHeuristicValue",
                                     &MovoGraspingHeuristicOptions::maxHeuristicValue);
        parser->addOption<FloatType>("heuristicPluginOptions",
                                     "heuristicScalingFactor",
                                     &MovoGraspingHeuristicOptions::heuristicScalingFactor);
        parser->addOption<VectorFloat>("heuristicPluginOptions",
                                     "enteringPose",
                                     &MovoGraspingHeuristicOptions::enteringPose);
        parser->addOption<VectorFloat>("heuristicPluginOptions",
                                     "exitPose",
                                     &MovoGraspingHeuristicOptions::exitPose);
    }

};
}

#endif
