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
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_

#include "oppt/plugin/Plugin.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "MovoGraspingInitialBeliefOptions.hpp"
#include "MovoUserData.hpp"
#include "MovoUtils.hpp"
#include <boost/timer.hpp>

namespace oppt
{

class MovoInitialBeliefPlugin: public InitialBeliefPlugin
{

// Private member variables
private:
    RobotEnvironment *robotEnvironment_ = nullptr;

    std::unique_ptr<Distribution<FloatType>> uniformDistribution_;

    std::unordered_map<std::string, gazebo::physics::Joint *> robotJoints_;

    bool initialStateReceived_ = false;

    boost::mutex mtx_;

    std::unordered_map<std::string, FloatType> initialJointAngles_;

    std::unique_ptr<Distribution<FloatType>> jointPositionErrorDistribution_ = nullptr;

    std::unique_ptr<Distribution<FloatType>> objectPositionErrorDistribution_ = nullptr;

    unsigned int stateSpaceDim_;

    // Vector to store initial set or received state vector
    VectorFloat initialStateVec_;

    // Frame and link names
    std::string tableFrame_ = "";


public:
    MovoInitialBeliefPlugin():
        InitialBeliefPlugin()
    {

    }

    virtual ~MovoInitialBeliefPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment,
                      const std::string& optionsFile) override {
        // Load robot environment and initial belief options
        robotEnvironment_ = robotEnvironment;
        parseOptions_<MovoInitialBeliefOptions>(optionsFile);
        auto options = static_cast<MovoInitialBeliefOptions*>(options_.get());
        // Verify state space matches specified initial bounds
        stateSpaceDim_ = robotEnvironment->getRobot()->getStateSpace()->getNumDimensions();
        // Check for configuration errors
        if (options->lowerBound.size() != stateSpaceDim_)
            ERROR("Lower bound for the uniform distribution doesn't match state space dimension");
        if (options->upperBound.size() != stateSpaceDim_)
            ERROR("Upper bound for the uniform distribution doesn't match state space dimension");
        for (size_t i = 0; i != options->lowerBound.size(); ++i) {
            if (options->lowerBound[i] > options->upperBound[i])
                ERROR("Lower bound for initial belief must be smaller than upper bound");
        }

        // Get engine with random seed to generate the probabilistic distributions
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();

        /********* CREATE UNIFORM DISTRIBUTIONS ****************/
        // Create a uniform distribution for the initial belief bounds of the state vector
        uniformDistribution_ =
            std::make_unique<UniformDistribution<FloatType>>(options->lowerBound, options->upperBound, randomEngine);

        // Create uniform distribution for the initial belief bounds of the object position
        objectPositionErrorDistribution_ =
            std::make_unique<UniformDistribution<FloatType>>(options->lowerObjectBound, options-> upperObjectBound, randomEngine);

        // Create uniform distribution for the initial arm joint angles
        jointPositionErrorDistribution_ =
            std::make_unique<UniformDistribution<FloatType>>(options->lowerJointBound, options->upperJointBound, randomEngine);

        // Assign table frame name
        tableFrame_ = options->tableLinkName;

        // Retrieve and populate the map of gazebo joint pointers corresponding to the left arm joints
        getRobotJoints();

        // Emulate an initial state configuration from gazebo if the planner is ran in simulation #NOTE: Make sure that the appropiate flag is set in the cfg file
        if(options->simulation){
            initialGazeboConfig();
        }

        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        // Wait for data to be populated to operate
        bool initialStateReceived = false;
        while (!initialStateReceived) {
            mtx_.lock();
            initialStateReceived = initialStateReceived_;
            mtx_.unlock();
        }

        // The initialStateVec_ is defined separately depending on if the program is run on simulation or on real execution
        VectorFloat initialStateSample = initialStateVec_;


        // Add some noise to the vector representing the starting state 
        addNoiseToInitialVec(initialStateSample);


        // First construct an intial world state based in the intial state vector
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateSample, false);

        // Then get the initial world state
        GazeboWorldStatePtr initialWorldState_ = robotEnvironment_->getGazeboInterface()->getInitialWorldState();

        // Create RobotStateSharedPointer from state vector
        RobotStateSharedPtr initialState = std::make_shared<VectorState>(initialStateSample);
        initialState->setGazeboWorldState(initialWorldState_);

        
        // Prepare user data
        RobotStateUserDataSharedPtr userData(new MovoGraspingTransitionPluginUserData());

        // Pose of the right knuckle link
        GZPose gripperPose = LinkWorldPose(getLinkPtr("right_knuckle_link", robotEnvironment_));

        static_cast<MovoGraspingTransitionPluginUserData*>(userData.get())->knuckleLinkPose = gripperPose;
        static_cast<MovoGraspingTransitionPluginUserData*>(userData.get())->knucklePoseVector = getVecFromPose(gripperPose);
        static_cast<MovoGraspingTransitionPluginUserData*>(userData.get())->collisionReport =
            std::make_shared<CollisionReport>();
        initialState->setUserData(userData);
        return initialState;
    }



    /*** Takes a vector representing the initial state of the problem and adds noise to it according to the internal noise distributions ***/
    void addNoiseToInitialVec(VectorFloat& rawInitStateVec){
        // Double check that the vector is of adequate size
        if (rawInitStateVec.size() != stateSpaceDim_){
            ERROR("Movo Initial Belief Plugin :: can't add noise to provided vector");
        }

        // Add the appropiate noise to the vector
        // Add some initial noise to the object's initial (x and y position coordinates)
        Vectordf objectPositionError = objectPositionErrorDistribution_->sample(1);
        rawInitStateVec[0] += objectPositionError[0];
        rawInitStateVec[1] += objectPositionError[1];

        /*** Add some error to the joint values included in the state vector ***/
        VectorFloat jointAngles(6, 0);
        auto jointAngleError = jointPositionErrorDistribution_->sample(1);
        VectorString jointNames({"right_shoulder_pan_joint",
                                 "right_shoulder_lift_joint",
                                 "right_elbow_joint",
                                 "right_wrist_spherical_1_joint",
                                 "right_wrist_spherical_2_joint",
                                 "right_wrist_3_joint"
                                });

        
        // Store the noisy angles and populate them into the state vector
        for (size_t i = 0; i != jointNames.size(); ++i) {
            auto joint = robotJoints_.at(jointNames[i]);
            FloatType resultingAngle = initialJointAngles_.at(jointNames[i]);
            //#CHECK_FOR_BUG
            joint->SetPosition(0, resultingAngle);
            jointAngles[i] = resultingAngle + jointAngleError(i);
            // Populate values into state vector (offset of 6 from the objects position)
            rawInitStateVec[6 + i]  = resultingAngle;
        }

        return;
    }



    /*** Records relevant information from the initial belief plug in to be used globally. The poses of the objects in the initial
    view of the world are set accordingly after information is received from the perception ***/
    virtual void setUserData(const OpptUserDataSharedPtr& userData) override {
        userData_ = userData;
        initialStateVec_ = VectorFloat(robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions(), 0.0);
        auto ud = static_cast<MovoInitialBeliefPluginUserData *>(userData.get());

        GZQuaternion q(ud->objectOrientation.w(), ud->objectOrientation.x(), ud->objectOrientation.y(), ud->objectOrientation.z());
        printVector(ud->objectPosition, "intial object position");


        // Setting initial vector based on user data
        initialStateVec_[0] = ud->objectPosition[0];
        initialStateVec_[1] = ud->objectPosition[1];
        initialStateVec_[2] = ud->objectPosition[2]; // offset to avoid object collision with table



        // Setting orientation
    #ifdef GZ_GT_7
        auto eulerAngles = q.Euler();
        initialStateVec_[3] = eulerAngles.X();
        initialStateVec_[4] = eulerAngles.Y();
        initialStateVec_[5] = eulerAngles.Z();

    #else 
        auto eulerAngles = q.GetAsEuler();
        initialStateVec_[3] = eulerAngles.x;
        initialStateVec_[4] = eulerAngles.y;
        initialStateVec_[5] = eulerAngles.z;
    #endif



        
        // Set joints in planning initial belief according to initial observation
        for (size_t i = 0; i != ud->jointAngles.size(); ++i) {
            LOGGING("Set joint '" + ud->jointNames[i] + "' to " + std::to_string(ud->jointAngles[i]));
            robotJoints_.at(ud->jointNames[i])->SetPosition(0, ud->jointAngles[i]);
            if (ud->jointNames[i].find("right_gripper_finger1_joint") != std::string::npos) {
                robotJoints_.at("right_gripper_finger2_joint")->SetPosition(0, ud->jointAngles[i]);
                robotJoints_.at("right_gripper_finger3_joint")->SetPosition(0, ud->jointAngles[i]);
            }
            initialJointAngles_[ud->jointNames[i]] = ud->jointAngles[i];
        }
        

        

        if (ud->gripperClosed)
            initialStateVec_[initialStateVec_.size() - 2] = 1.0;
        if (ud->graspEstablished)
            initialStateVec_[initialStateVec_.size() - 1] = 1.0;

        // Remove the current table
        if (robotEnvironment_->getPrefix().find("exec") != std::string::npos) {
            EnvironmentChangeSharedPtr tableRemovedChange(new BodyRemovedChange(tableFrame_, true));
            robotEnvironment_->addEnvironmentChange(tableRemovedChange);
        }

        // Re-add the table with the right pose and dimensions
        VectorFloat newTablePosition{ud->tablePosition[0], ud->tablePosition[1] - 0.2, ud->tablePosition[2]};
    
        printVector(ud->tableDimensions, "TABLE DIMENSIONS");
        printVector(ud->tablePosition, "TABLE POSITION");


        if (robotEnvironment_->getPrefix().find("exec") != std::string::npos) {
            std::string sdfString = getBoxSDF("box1", newTablePosition, ud->tableDimensions);
            EnvironmentChangeSharedPtr tableAddChange(new BodyAddedChange(sdfString, true));
            robotEnvironment_->addEnvironmentChange(tableAddChange);
        }
        
        // Sets the initial world state of gazebo world with the robot having the joints as described in intialStateVec
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateVec_, false);
        mtx_.lock();
        initialStateReceived_ = true;
        mtx_.unlock();
    }

// Private functions
private:
    // Retrieve robot joint pointers and create a map between joint names and their corresponding pointers
    void getRobotJoints() {
        auto joints = robotEnvironment_->getGazeboInterface()->getJoints();
        for(auto& joint : joints){
            robotJoints_[joint->GetName()] = joint;
         
        }
    }


    /*** Simulates the initial state set up of the actual operating robot. (In the real execution, the initial state is set up by the popcorn interface OPPTInitBeliefAction)
    Configure the initial look on the simulation world of how the robot should start and record the corresponding joint angles of the robot in radians***/
    void initialGazeboConfig() {
        // Sample an initial state vector
        initialStateVec_ = toStdVec<FloatType>(uniformDistribution_->sample(1).col(0));
        auto options = static_cast<MovoInitialBeliefOptions *>(options_.get());
        // Set the gazebo position joints to the desired angles
        // Since joints can either be prismatic or rotational, the angle set reflects the displacement of the link connected to the joint
        robotJoints_.at("linear_joint")->SetPosition(0, options->linear);
        // Set joint angles for the right arm
        robotJoints_.at("pan_joint")->SetPosition(0, options->pan_joint_angle);
        robotJoints_.at("tilt_joint")->SetPosition(0, options->tilt_joint_angle);
        robotJoints_.at("right_shoulder_pan_joint")->SetPosition(0, options->right_shoulder_pan_joint_angle);
        robotJoints_.at("right_shoulder_lift_joint")->SetPosition(0, options->right_shoulder_lift_joint_angle);
        robotJoints_.at("right_arm_half_joint")->SetPosition(0, options->right_arm_half_joint_angle);
        robotJoints_.at("right_elbow_joint")->SetPosition(0, options->right_elbow_joint_angle);
        robotJoints_.at("right_wrist_spherical_1_joint")->SetPosition(0, options->right_wrist_spherical_1_joint_angle);
        robotJoints_.at("right_wrist_spherical_2_joint")->SetPosition(0, options->right_wrist_spherical_2_joint_angle);
        robotJoints_.at("right_wrist_3_joint")->SetPosition(0, options->right_wrist_3_joint_angle);
        robotJoints_.at("right_gripper_finger1_joint")->SetPosition(0, options->right_gripper_finger1_joint_angle);
        robotJoints_.at("right_gripper_finger2_joint")->SetPosition(0, options->right_gripper_finger2_joint_angle);
        robotJoints_.at("right_gripper_finger3_joint")->SetPosition(0, options->right_gripper_finger2_joint_angle);

        // Populate joint - angle map for each joint
    #ifdef GZ_GT_7
        // Right arm joints
        initialJointAngles_["linear_joint"] = robotJoints_.at("linear_joint")->Position(0);
        initialJointAngles_["right_shoulder_pan_joint"] = robotJoints_.at("right_shoulder_pan_joint")->Position(0);
        initialJointAngles_["right_shoulder_lift_joint"] = robotJoints_.at("right_shoulder_lift_joint")->Position(0);
        initialJointAngles_["right_elbow_joint"] = robotJoints_.at("right_elbow_joint")->Position(0);
        initialJointAngles_["right_wrist_spherical_1_joint"] = robotJoints_.at("right_wrist_spherical_1_joint")->Position(0);
        initialJointAngles_["right_wrist_spherical_2_joint"] = robotJoints_.at("right_wrist_spherical_2_joint")->Position(0);
        initialJointAngles_["right_wrist_3_joint"] = robotJoints_.at("right_wrist_3_joint")->Position(0);
        initialJointAngles_["right_gripper_finger1_joint"] = robotJoints_.at("right_gripper_finger1_joint")->Position(0);
        initialJointAngles_["right_gripper_finger2_joint"] = robotJoints_.at("right_gripper_finger2_joint")->Position(0);
        initialJointAngles_["right_gripper_finger3_joint"] = robotJoints_.at("right_gripper_finger3_joint")->Position(0);
    #else  
        // Right arm joints
        initialJointAngles_["linear_joint"] = robotJoints_.at("linear_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_shoulder_pan_joint"] = robotJoints_.at("right_shoulder_pan_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_shoulder_lift_joint"] = robotJoints_.at("right_shoulder_lift_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_elbow_joint"] = robotJoints_.at("right_elbow_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_wrist_spherical_1_joint"] = robotJoints_.at("right_wrist_spherical_1_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_wrist_spherical_2_joint"] = robotJoints_.at("right_wrist_spherical_2_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_wrist_3_joint"] = robotJoints_.at("right_wrist_3_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_gripper_finger1_joint"] = robotJoints_.at("right_gripper_finger1_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_gripper_finger2_joint"] = robotJoints_.at("right_gripper_finger2_joint")->GetAngle(0).Radian();
        initialJointAngles_["right_gripper_finger3_joint"] = robotJoints_.at("right_gripper_finger3_joint")->GetAngle(0).Radian();
    #endif

        // Update synchronisation variable
        mtx_.lock();
        initialStateReceived_ = true;
        mtx_.unlock();

    }





    // Add a table with the correct positions of the sdf
    std::string getBoxSDF(const std::string &boxName,
                          const VectorFloat &tablePosition,
                          const VectorFloat &dimensions,
                          const bool &visualOnly = false) {
        std::string tableString = "<sdf version='1.6'>";
        tableString += "<model name='" + boxName + "'>";
        tableString += "<pose frame=''>";

        tableString += std::to_string(tablePosition[0]) +
                       " " +
                       std::to_string(tablePosition[1]) +
                       " " +
                       std::to_string(tablePosition[2]) +
                       " 0.0 0.0 0.0";

        tableString += "</pose>";
        tableString += "<static>1</static>";
        tableString += "<link name='" + boxName + "Link'>";
        tableString += "<visual name='" + boxName + "Visual'>";
        tableString += "<geometry>";
        tableString += "<box>";
        tableString += "<size>";
        tableString += std::to_string(dimensions[0]) + " ";
        tableString += std::to_string(dimensions[1]) + " ";
        tableString += std::to_string(dimensions[2]);
        tableString += "</size>";
        tableString += "</box>";
        tableString += "</geometry>";
        tableString += "<material>";
        tableString += "<ambient>";
        tableString += "0.7 0.7 0.7 1.0";
        tableString += "</ambient>";
        tableString += "</material>";
        tableString += "</visual>";
        if (!visualOnly) {
            tableString += "<collision name='" + boxName + "Collision'>";
            tableString += "<geometry>";
            tableString += "<box>";
            tableString += "<size>";
            tableString += std::to_string(dimensions[0]) + " ";
            tableString += std::to_string(dimensions[1]) + " ";
            tableString += std::to_string(dimensions[2]);
            tableString += "</size>";
            tableString += "</box>";
            tableString += "</geometry>";
            tableString += "</collision>";
        }
        tableString += "</link>";
        tableString += "</model>";
        tableString += "</sdf>";
        return tableString;
    }



    // Retrieve a vector from a pose
    VectorFloat getVecFromPose(const GZPose& poseData) const{
        VectorFloat result(6, 0);

    #ifdef GZ_GT_7
        auto orientation = poseData.Rot().Euler();
        result[0] = poseData.Pos().X();
        result[1] = poseData.Pos().Y();
        result[2] = poseData.Pos().Z();
        result[3] = orientation.X();
        result[4] = orientation.Y();
        result[5] = orientation.Z();
    #else
        auto orientation = poseData.rot.GetAsEuler();
        result[0] = poseData.pos.x;
        result[1] = poseData.pos.y;
        result[2] = poseData.pos.z;
        result[3] = orientation.x;
        result[4] = orientation.y;
        result[5] = orientation.z;
    #endif

        return result;
    }





    // Auxiliary functions to getting and setting world poses with different apis
    GZPose LinkWorldPose(const gazebo::physics::Link* link) const{
    // Returns link world pose according to gazebo api enabled
    #ifdef GZ_GT_7
        return link->WorldPose();
    #else 
        return link->GetWorldPose();
    #endif

    }
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(MovoInitialBeliefPlugin)

}

#endif
