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
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_OPTIONS_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{

/*** Class representing a parser for the initial belief options ***/
class MovoInitialBeliefOptions: public PluginOptions
{
public:
    // Constructor
    MovoInitialBeliefOptions() = default;
    // Inherited destructor
    virtual ~MovoInitialBeliefOptions() = default;

    // Lower and Upper Bounds
    VectorFloat lowerBound;

    VectorFloat upperBound;

    // Left shoulder joint variables
    FloatType linear = 0.0;

    // Right arm joint variables    
    FloatType pan_joint_angle = 0.0;

    FloatType tilt_joint_angle = 0.0;

    FloatType right_shoulder_pan_joint_angle = 0.0;

    FloatType right_shoulder_lift_joint_angle = 0.0;

    FloatType right_arm_half_joint_angle = 0.0;

    FloatType right_elbow_joint_angle = 0.0;

    FloatType right_wrist_spherical_1_joint_angle = 0.0;

    FloatType right_wrist_spherical_2_joint_angle = 0.0;

    FloatType right_wrist_3_joint_angle = 0.0;

    FloatType right_gripper_finger1_joint_angle = 0.0;

    FloatType right_gripper_finger2_joint_angle = 0.0;

    FloatType right_gripper_finger3_joint_angle = 0.0;
    // Lower and upper joint bounds for joint angles
    VectorFloat lowerJointBound;

    VectorFloat upperJointBound;

    // Lower and upper position bounds for the object
    VectorFloat lowerObjectBound;

    VectorFloat upperObjectBound;


    // Link names
    std::string tableLinkName = "";


    // Bool for definition of simulation run or not
    bool simulation;
 
    // Create a parser for options in configuration file
    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addDefaultInitialBeliefOptions(parser.get());
        return std::move(parser);
    }

    // Parse options from configuration file
    static void addDefaultInitialBeliefOptions(options::OptionParser* parser) {
        // Populate initial belief bounds
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                     "lowerBound",
                                     &MovoInitialBeliefOptions::lowerBound);

        parser->addOption<VectorFloat>("initialBeliefOptions",
                                     "upperBound",
                                     &MovoInitialBeliefOptions::upperBound);

        // Populate error bounds for the object
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "lowerObjectPositionErrorBound",
                                       &MovoInitialBeliefOptions::lowerObjectBound);
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "upperObjectPositionErrorBound",
                                       &MovoInitialBeliefOptions::upperObjectBound);

        // Populate error joint bounds for the joint values
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "lowerJointPositionErrorBound",
                                       &MovoInitialBeliefOptions::lowerJointBound);
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "upperJointPositionErrorBound",
                                       &MovoInitialBeliefOptions::upperJointBound);

        // Populate joint variables with their corresponding assignation in the configuration file
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "linear",
                                     &MovoInitialBeliefOptions::linear);

        // Right arm joint variables
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "pan_joint_angle",
                                     &MovoInitialBeliefOptions::pan_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "tilt_joint_angle",
                                     &MovoInitialBeliefOptions::tilt_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_shoulder_pan_joint_angle",
                                     &MovoInitialBeliefOptions::right_shoulder_pan_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_shoulder_lift_joint_angle",
                                     &MovoInitialBeliefOptions::right_shoulder_lift_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_elbow_joint_angle",
                                     &MovoInitialBeliefOptions::right_elbow_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_wrist_spherical_1_joint_angle",
                                     &MovoInitialBeliefOptions::right_wrist_spherical_1_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_wrist_spherical_2_joint_angle",
                                     &MovoInitialBeliefOptions::right_wrist_spherical_2_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_wrist_3_joint_angle",
                                     &MovoInitialBeliefOptions::right_wrist_3_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_gripper_finger1_joint_angle",
                                     &MovoInitialBeliefOptions::right_gripper_finger1_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_gripper_finger2_joint_angle",
                                     &MovoInitialBeliefOptions::right_gripper_finger2_joint_angle);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "right_gripper_finger3_joint_angle",
                                     &MovoInitialBeliefOptions::right_gripper_finger3_joint_angle);


        // Link names
        parser->addOption<std::string>("initialBeliefOptions",
                                       "tableLinkName",
                                       &MovoInitialBeliefOptions::tableLinkName);


        // Simulation Definition
        parser->addOption<bool>("initialBeliefOptions",
                                       "simulation",
                                       &MovoInitialBeliefOptions::simulation);
    }
};
}

#endif
