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
#ifndef _DEFAULT_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _DEFAULT_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class MovoGraspingTransitionPluginOptions: public PluginOptions
{
public:
    MovoGraspingTransitionPluginOptions() = default;

    virtual ~MovoGraspingTransitionPluginOptions() = default;

    /** @brief The process error the actions are affected to */
    VectorFloat errorMargins;

    // Link Names
    std::string handLinkName = "";

    std::string gripperLinkName = "";

    std::string targetLinkName = "";

    // Transition specific values

    FloatType maxHorizontalGraspDist = 0.0;  
    FloatType maxVerticalGraspDist = 0.0;
        

    VectorFloat jointCovariances = VectorFloat();

    VectorUInt actionSpaceDiscretization;

    FloatType fingerClosingAngle = 0.0;

    unsigned int stepDuration = 0;

    FloatType failedGraspProbability = 0.0;

    FloatType objectWidth = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addDefaultTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addDefaultTransitionPluginOptions(options::OptionParser* parser) {
        // Action discretization options
        VectorUInt defVec;
        parser->addOptionWithDefault<VectorUInt>("ABT","actionDiscretization",
        &MovoGraspingTransitionPluginOptions::actionSpaceDiscretization, defVec);
        parser->addOption<VectorFloat>("transitionPluginOptions",
                                       "errorMargins",
                                       &MovoGraspingTransitionPluginOptions::errorMargins);
        
        // Link name of the right hand gripper 
        parser->addOption<std::string>("transitionPluginOptions",
                                       "gripperLinkName",
                                       &MovoGraspingTransitionPluginOptions::gripperLinkName);

        // Link name of the grasping target
        parser->addOption<std::string>("transitionPluginOptions",
                                     "targetLinkName",
                                     &MovoGraspingTransitionPluginOptions::targetLinkName);
        // Transition specific values
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "maxHorizontalGraspDist",
                                     &MovoGraspingTransitionPluginOptions::maxHorizontalGraspDist); 

        parser->addOption<FloatType>("transitionPluginOptions",
                                     "maxVerticalGraspDist",
                                     &MovoGraspingTransitionPluginOptions::maxVerticalGraspDist);  


        parser->addOption<FloatType>("transitionPluginOptions",
                                     "fingerClosingAngle",
                                     &MovoGraspingTransitionPluginOptions::fingerClosingAngle);

        parser->addOption<unsigned int>("transitionPluginOptions",
                                        "stepDuration",
                                        &MovoGraspingTransitionPluginOptions::stepDuration);

        parser->addOption<FloatType>("transitionPluginOptions",
                                     "failedGraspProbability",
                                     &MovoGraspingTransitionPluginOptions::failedGraspProbability);

        parser->addOption<VectorFloat>("transitionPluginOptions",
                                     "jointCovariances",
                                     &MovoGraspingTransitionPluginOptions::jointCovariances); 

    }
};
}

#endif
