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
#define _USE_MATH_DEFINES
#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "oppt/opptCore/typedefs.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"  
#include <boost/timer/timer.hpp>
#include <cmath>

// Plugin includes
#include "MovoUserData.hpp"
#include "MovoUtils.hpp"
#include "MovoGraspingTransitionOptions.hpp"
#include "MovoGraspingActionSpaceDiscretizer.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"


namespace oppt
{
class MovoGraspingTransitionPlugin: public TransitionPlugin
{
private: 
    // Pointer to the robot environment
    const RobotEnvironment* robotEnvironment_;
    // Unorder map of joints
    std::unordered_map<std::string, gazebo::physics::Joint*> movoJointMap_;
    // Unordered map of links
    std::unordered_map<std::string, gazebo::physics::Link*> movoLinkMap_;
    // Unorder map collision Objects associated with each link
    std::unordered_map<std::string, OpptCollisionObject*> collisionLinkMap;
    // List of upper angle limits for joints
    VectorFloat jointUpperPositionLimits;
    // List of lower angle limits for joints
    VectorFloat jointLowerPositionLimits;

    /******** LINKS *********/
    // A link pointer to the cylindrical object
    gazebo::physics::Link* objectLink_;
    // A link pointer top the right nuckle link
    gazebo::physics::Link* rightKnuckleLink_ = nullptr;

    // A link pointer to the right end effector link
    gazebo::physics::Link* rightEELink_ = nullptr;

    // Angle at which fingers are brought to closing position
    FloatType fingerClosingAngle_ = 0.0;

    // Duration of a step in execution 
    unsigned int stepDuration_ = 1000;

    // Probability of obtaining a grasp when the necessary conditions are met
    FloatType failedGraspProbability_ = 0.0;
    
    // Maximum vertical distances at which a grasp is suspected to be successful

    FloatType maxHorizontalGraspDist_ = 0.02;
    FloatType maxVerticalGraspDist_ = 0.02;


    // A float type distribution for the modelled error
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

    // flag to check if plugin is for execution
    bool isExecPlugin_ = false;

    /************ COLLISION CHECKING ********************/
    /**
     * @brief Shared pointer to the oppt::CollisionObject of the object we want to grasp
     */
    std::string cupName_;
    Body* targetBody_ = nullptr;


    // Function pointers to custom defined collision functions of interest
    /**
     * @brief Function that is used to determine if the knuckle link collides
     * with the object we want to grasp. This is a requirement for a successful grasp
     */
    std::function<bool ()> knuckleObjectCollisionFunction_ = nullptr;

    


public:
    // Default Constructor
    MovoGraspingTransitionPlugin():
        TransitionPlugin() {
    }
    // Default destructor
    virtual ~MovoGraspingTransitionPlugin() = default;

    // Setup method to retrieve information for the plugin
    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        // Parse transition options from config file
        parseOptions_<MovoGraspingTransitionPluginOptions>(optionsFile);
        auto movoOptions = static_cast<MovoGraspingTransitionPluginOptions*>(options_.get());

        // Overwrite action space with custom one for individual actions
        auto actionSpace = robotEnvironment->getRobot()->getActionSpace();
        auto actionSpaceDiscretization = movoOptions->actionSpaceDiscretization;
        if (actionSpaceDiscretization.size() == 0)
            ERROR("Size of action space discretization must be greater than 0");
        std::shared_ptr<ActionSpaceDiscretizer> robotActionSpaceDiscretizer(new MovoGraspingActionSpaceDiscretizer(actionSpace,
                actionSpaceDiscretization));
        actionSpace->setActionSpaceDiscretizer(robotActionSpaceDiscretizer);


        // Create Joint pointers map and populate joint limits
        robotEnvironment_ = robotEnvironment;

        // Parse nlink name associated with the object to grasp
        cupName_ = movoOptions->targetLinkName;

        // Obtain a pointer to the body of the cylinder in the scene
        targetBody_ = robotEnvironment_->getScene()->getBody(cupName_);
        auto bodies = robotEnvironment_->getScene()->getBodies();


        // Create maps for the robot's joint and link ptrs and their corresponding names
        makeMovoJointMap();
        makeMovoLinkMap();
        makeMovoLinkCollisionMap();

        /*** Retrieve pointers to links of interest in the robot ***/
        // Link to the object attempted to grasp
        objectLink_ = getLinkPtr("cylinderLink", robotEnvironment_);
        // Get a pointer to the knuckle link of the right arm
        rightKnuckleLink_ = getLinkPtr("right_knuckle_link", robotEnvironment_);

        // Get a pointer to the right end effector link of the right arm
        rightEELink_ = getLinkPtr("right_ee_link", robotEnvironment_);

        /*** Check for failures  in retriving link pointers ***/
        if (!objectLink_)
            ERROR("Link '" + "cylinderLink" + "' could not be found");
        if (!rightKnuckleLink_)
            ERROR("Right knuckle link not found");

        // Setup a collision function to be used for links
        setupCollisionFunctions();


        

        /*** Defined grasping distances for suspected successful grasping ***/

        // If the distance of the hand in the x,y-coordinates to the cup is larger than
        // 'maximumGraspingZDistance', a grasp attempt won't be successful
        maxHorizontalGraspDist_ = movoOptions->maxHorizontalGraspDist;

        // If the distance of the hand in the z-coordinates to the cup is larger than
        // 'maximumGraspingZDistance', a grasp attempt won't be successful
        maxVerticalGraspDist_ = movoOptions->maxVerticalGraspDist;

        

        // Generate a Gaussian distribution from the options
        makeGaussianErrorDistribution(movoOptions);

        // Get the joint angles of the fingers in closed position
        fingerClosingAngle_ = movoOptions->fingerClosingAngle;

        // Get the duration an action takes to execute in the execution plugin
        stepDuration_ = movoOptions->stepDuration;

        // Get the probability that a grasp fails, despite all grasping conditions being met
        failedGraspProbability_ = movoOptions->failedGraspProbability;


        // Check if this is the plugin for execution
        isExecPlugin_ = false;
        if (robotEnvironment_->getPrefix().find("exec") != std::string::npos) {
            isExecPlugin_ = true;
        }


        return true;
    }

    // Method that returns a transition of the form T(s, a, s')
    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {

        /*** Create propagation result packaging container ***/
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        propagationResult->previousState = propagationRequest->currentState.get();
        propagationResult->action = propagationRequest->action;

        /*** RAW Action and inputState Vectors ***/
        VectorFloat inputStateVec = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();

        /*** Variables of interest to be packed into user data structure ***/
        bool illegalHandOpening{false};
 
        // // Save the pose of the current knuckle to track how it has moved
        // GZPose relPoseBefore = objectLink_->GetWorldPose() - rightKnuckleLink_->GetWorldPose();
        

        // Update the gazebo world interface to check for kinematic information regarding this states
        robotEnvironment_->getGazeboInterface()->setWorldState(propagationRequest->currentState->getGazeboWorldState().get(), true);
        robotEnvironment_->getGazeboInterface()->setStateVector(inputStateVec);

        // Vector to store the next state (s') copied from the input State
        VectorFloat resultingState(inputStateVec);

        // Indicates if the hand was closed in the previous state (s)
        bool handIsClosed = (inputStateVec[inputStateVec.size() - 2] == 1);
        // Indicates if the previous state (s) had a grasp of the object
        bool alreadyGrasped = (inputStateVec[inputStateVec.size() - 1] == 1) && handIsClosed;
        

        // // // /******* APPLY GIVEN ACTION AND UPDATE THE RESULTING STATE VECTOR ACCORDINGLY ****/
        FloatType robotAction = actionVec[actionVec.size() - 1];
        // Check if action is of type "joint increment/decrements"
        if (robotAction == 0) {
            

            /**************** MOVE RIGHT ARM ********************/
            /* Increment/Decrement joint values of the right arm according to the delta values in the given action and some Gaussian noise
               The arm pose is updated in gazebo after call to this function */
            moveRightArm(inputStateVec, resultingState, actionVec); 

            // update right arm from movement
            //updateGazeboJoints(resultingState);

        } else if (robotAction == 1) {
        /*************** CHECK FOR CLOSE GRIPPER ACTION *******************/
            bool graspSuccess{false};

            // Avoid redudant closing if hand was closed. Otherwise, hand is closed and should not check for GRASPED state
            if (!handIsClosed) {
                // Hand was opened. Close hand
                
                closeRightHand(resultingState);
                // Check to see if grasping is achieved after closing right hand
                graspSuccess = tryGrasp(resultingState);

                // Update relevant information on successful grasp
                if(graspSuccess)
                {
                    resultingState[resultingState.size() - 1] =  1;
                } 

            }

        } else if (robotAction == -1) {
            /**************** OPEN GRIPPER ********************/
            // Update the gripper joint values to reflect the robot openning the hand and check if the opening action was "illegal"
            illegalHandOpening = openRightHand(inputStateVec, resultingState);
        } 
            
        
        //  /******************* END OF ACTION UPDATE *************************************/
        // // Update object's pose in state information
        VectorFloat objectPoseVec = getLinkPoseVec(objectLink_);
        for (size_t i = 0; i < 6; ++i) {
            resultingState[i] = objectPoseVec[i];
        }


       



        /********** PACK THE RESULTING STATE VECTOR AND THE RELEVANT USER DATA ******************/
        propagationResult->nextState = std::make_shared<oppt::VectorState>(resultingState);


        // Set Gazebo World for the next State Vector
        propagationResult->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));



        // // Record relative pose after
        // GZPose relPoseAfter = objectLink_->GetWorldPose() - rightKnuckleLink_->GetWorldPose();

        // // Compute illegal finger collision
        // bool illegalFingerCollision = checkIllegalFingerCollision(relPoseBefore, relPoseAfter);

        //Attach relevant information to new result structure
        auto userDataNew =
            makeUserDataGrasping(resultingState, illegalHandOpening);
        propagationResult->nextState->setUserData(userDataNew);

        propagationResult->collisionReport =
            static_cast<MovoGraspingTransitionPluginUserData *>(propagationResult->nextState->getUserData().get())->collisionReport;


        if (isExecPlugin_)
            boost::this_thread::sleep_for(boost::chrono::milliseconds(stepDuration_));


        return propagationResult;
    }








/**************************************************************** HELPER FUNCTIONS ************************************************************************************************************/
private:



    /*** Creates and returns a structure with the relevant transition information to be used outside of the plugin ***/
    RobotStateUserDataSharedPtr makeUserDataGrasping(const VectorFloat &stateVec, const bool &illegalHandOpening) const {
        // Structure where result is stored
        RobotStateUserDataSharedPtr userData(new MovoGraspingTransitionPluginUserData());
        auto ud = static_cast<MovoGraspingTransitionPluginUserData*>(userData.get());

        // /********* HAND AND OBJECT POSES INFORMATION ***********************/
        GZPose gripperPose = LinkWorldPose(rightKnuckleLink_);
        GZPose rightEEPose = LinkWorldPose(rightEELink_);
        VectorFloat objectPoseVec = getLinkPoseVec(objectLink_);
        VectorFloat gripperPoseVec = getLinkPoseVec(rightKnuckleLink_);
        VectorFloat rightEEPoseVec = getVecFromPose(rightEEPose);
        #ifdef GZ_GT_7
        auto objectPoseWithRespectToEndEffectorFrame = objectLink_->WorldPose() - rightKnuckleLink_->WorldPose();
        #else
        auto objectPoseWithRespectToEndEffectorFrame = objectLink_->GetWorldPose() - rightKnuckleLink_->GetWorldPose();
        #endif

        // /******* POPULATE USER DATA STRUCTURE **************/
        // // Relative y distance
        ud->distY = std::fabs(objectPoseVec[1] - gripperPoseVec[1]); // std::fabs(objectPoseWithRespectToEndEffectorFrame.pos.y); 
        ud->distZ = std::fabs(objectPoseVec[2] - gripperPoseVec[2]);
        ud->distX = std::fabs(objectPoseVec[0] - gripperPoseVec[0]);

        // // Collision report check
        ud->collisionReport = robotEnvironment_->getRobot()->makeDiscreteCollisionReportDirty();

        // // Info on if the hand is currently closed
        ud->illegalHandOpening = illegalHandOpening;

        // // Store the value for illegal finger collision
        // ud->illegalFingerCollision = illegalFingerCollisionVal;


        // World pose of right ee link
        ud->rightEEPose = rightEEPose;
        ud->rightEEPoseVector = rightEEPoseVec;

        // // World pose of the knuckle link
        ud->knuckleLinkPose = gripperPose;
        ud->knucklePoseVector = gripperPoseVec;

        return userData;
    }



    // ///////////////////// FUNCTIONS TO HANDLE THE UPDATE OF THE RIGHT ARM JOINTS, RIGHT FINGER JOINTS, AND LEFT ARM JOINTS //////////////////
    // /*** Attempts a grasp from the current position and sistance to the object. The function returns wether a grasp was successful or not ***/
    bool tryGrasp(const VectorFloat &resultingState) const {
        bool graspEstablished{false};
        // Poses and positions of links of interest in vector form
        VectorFloat knucklePoseVec = getLinkPoseVec(rightEELink_);
        VectorFloat objectPoseVec = getLinkPoseVec(objectLink_);
        FloatType graspProbabilityCheck = failedGraspProbability_;
        #ifdef GZ_GT_7
        auto objectPoseWithRespectToEndEffectorFrame = objectLink_->WorldPose() - rightEELink_->WorldPose();
        FloatType xDistanceToObject = objectPoseWithRespectToEndEffectorFrame.Pos().X();
        FloatType yDistanceToObject = std::fabs(objectPoseWithRespectToEndEffectorFrame.Pos().Y());
        auto relativeXRotation = objectPoseWithRespectToEndEffectorFrame.Rot().Euler().X();
        #else
        auto objectPoseWithRespectToEndEffectorFrame = objectLink_->GetWorldPose() - rightEELink_->GetWorldPose();
        FloatType xDistanceToObject = objectPoseWithRespectToEndEffectorFrame.pos.x;
        FloatType yDistanceToObject = std::fabs(objectPoseWithRespectToEndEffectorFrame.pos.y);
        auto relativeXRotation = objectPoseWithRespectToEndEffectorFrame.rot.GetAsEuler().x;
        #endif
        
        // FloatType zDistanceToObject = std::fabs(objectPoseWithRespectToEndEffectorFrame.pos.z);

        // Calculate the object pose relative to the end-effector pose
        FloatType zDistanceToObject = std::fabs(objectPoseVec[2] - knucklePoseVec[2]);
        // FloatType zDistanceToObject = std::fabs(objectPoseWithRespectToEndEffectorFrame.pos.z);
        
        // FloatType xDistanceToObject = std::fabs(objectPoseVec[0] - knucklePoseVec[0]);
    
        //cout << "ZDistance is" << zDistanceToObject << std::endl;
        // First check of the knuckle link collides with the object
        bool knuckleObjectCollision = knuckleObjectCollisionFunction_();

        

        // if (xDistanceToObject > maxHorizontalGraspDist_)
        //     cout<<"=====================> x distance not satisfied"<<endl;
        
        // if (std::fabs(relativeXRotation) > 0.18)
        //     cout<<"=====================> orientation not satisfied"<<endl;

        // if (zDistanceToObject > maxVerticalGraspDist_)
        //     cout<<"=====================> vertical distance not satisfied"<<endl;
        // Sample a grasp trial if the grasping conditions are met

        //cout<<"x distance : "<<xDistanceToObject<<endl;

        if (xDistanceToObject <= maxHorizontalGraspDist_ && std::fabs(relativeXRotation) <= 0.18 && zDistanceToObject <= maxVerticalGraspDist_ && knuckleObjectCollision)
        {
            std::uniform_real_distribution<FloatType> fgd(0.0, 1.0);
            auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
            FloatType sample = fgd(*(randomEngine.get()));

            if (sample > graspProbabilityCheck){
                graspEstablished = true;
            }
        }

        return graspEstablished;
    }



    // bool checkIllegalFingerCollision( GZPose& relPoseBefore,  GZPose& relPoseAfter) const{
    //     bool insideGrasperOldRelX = (relPoseBefore.pos.x < 0.09 && relPoseBefore.pos.x > -0.09);
    //     bool insideGrasperNewRelX = (relPoseAfter.pos.x < 0.09 && relPoseAfter.pos.x > -0.09);


    //     // Print poses to ensure they are different
    //     std::cout << "REL BEFORE" << relPoseBefore.pos.x << ", " << relPoseBefore.pos.y << std::endl;

    //     std::cout << "REL AFTER" << relPoseAfter.pos.x << ", " << relPoseAfter.pos.y << std::endl;



    //     // Ccheck if the relative pose of the object indicates that the object 
    //     if(insideGrasperOldRelX || insideGrasperNewRelX){
    //         if(relPoseBefore.pos.y > 0.06) {
    //             if(relPoseAfter.pos.y <= 0.06){
    //                 return true;
    //             }
    //         }

    //         if(relPoseBefore.pos.y < -0.06) {
    //             if(relPoseAfter.pos.y >= -0.06) {
    //                 return true;
    //             }
    //         }
    //     }

    //     return false;


    // }

    /*** Updates the robot joint values corresponding to the robot respectively ***/
    void moveRightArm(VectorFloat& inputState, VectorFloat& resultingState, VectorFloat actionVec) const 
    {
        // Move right hand
        // Only modify right hand associated joints
        // Sample an error vector
        auto error = errorDistribution_->sample(1);
        for (size_t i = 0; i < 6; ++i) 
        {
            // Offset to start at the right arm joints
            resultingState[i + 6] = inputState[i + 6] + actionVec[i]+ error(i);
        
            // Check that new angles are within limits
            if (resultingState[i + 6] > jointUpperPositionLimits[i + 1]) {
                resultingState[i + 6] = jointUpperPositionLimits[i + 1];
            }
            if (resultingState[i + 6] < jointLowerPositionLimits[i + 1]) {
                resultingState[i + 6] = jointLowerPositionLimits[i + 1];
            }
        }

        // Update joint position values in the Gazebo simulation
        movoJointMap_.at("right_shoulder_pan_joint")->SetPosition(0, resultingState[6]);
        movoJointMap_.at("right_shoulder_lift_joint")->SetPosition(0, resultingState[7]);
        movoJointMap_.at("right_elbow_joint")->SetPosition(0, resultingState[8]);
        movoJointMap_.at("right_wrist_spherical_1_joint")->SetPosition(0, resultingState[9]);
        movoJointMap_.at("right_wrist_spherical_2_joint")->SetPosition(0, resultingState[10]);
        movoJointMap_.at("right_wrist_3_joint")->SetPosition(0, resultingState[11]);
        // make sure extra joint is 0
        movoJointMap_.at("right_arm_half_joint")->SetPosition(0, 0);
    }





    /*** Updates the robot right hand finger joints for an opening or closing motion ***/
    bool openRightHand(VectorFloat& inputState, VectorFloat& resultingState) const {
        bool illegalHandOpening{false};
        bool alreadyGrasped = (inputState[inputState.size() - 1] == 1);
        bool handIsClosed = (inputState[inputState.size() - 2] == 1);
        // If the grasp and the hand closed flags are set, then there was a grasp
        if (handIsClosed && alreadyGrasped) {
            illegalHandOpening = true;
        }

        // Update resulting vector indicating that hand is opened
        resultingState[resultingState.size() - 2] = 0.0;

        // Opening motion
        movoJointMap_.at("right_gripper_finger1_joint")->SetPosition(0, 0.0);
        movoJointMap_.at("right_gripper_finger2_joint")->SetPosition(0, 0.0);
        movoJointMap_.at("right_gripper_finger3_joint")->SetPosition(0, 0.0);

        return illegalHandOpening;
    }



    /*** Updates the robot's right hand finger joints for a closing motion ***/
    void closeRightHand(VectorFloat& resultingState) const {
        // Update resulting state
        resultingState[resultingState.size() - 2] = 1.0;
        // Closing motion
        movoJointMap_.at("right_gripper_finger1_joint")->SetPosition(0, fingerClosingAngle_);
        movoJointMap_.at("right_gripper_finger2_joint")->SetPosition(0, fingerClosingAngle_);
        movoJointMap_.at("right_gripper_finger3_joint")->SetPosition(0, fingerClosingAngle_);
    }





    /*** Sets up the update positions of the joint angles in the gazebo interface ***/
    void updateGazeboJoints(VectorFloat& resultingState) const {
        // Set for now
        movoJointMap_.at("right_shoulder_pan_joint")->SetPosition(0, resultingState[6]);
        movoJointMap_.at("right_shoulder_lift_joint")->SetPosition(0, resultingState[7]);
        movoJointMap_.at("right_elbow_joint")->SetPosition(0, resultingState[8]);
        movoJointMap_.at("right_wrist_spherical_1_joint")->SetPosition(0, resultingState[9]);
        movoJointMap_.at("right_wrist_spherical_2_joint")->SetPosition(0, resultingState[10]);
        movoJointMap_.at("right_wrist_3_joint")->SetPosition(0, resultingState[11]);

        // FIX EXTRA JOINTS FOR BOTH ARMS FOR NOW
        movoJointMap_.at("right_arm_half_joint")->SetPosition(0,0);
    }


    // /*** Defines custom functions that can be used for collision checking for different partarms of the robot ***/
    void setupCollisionFunctions() {
    
        /*********** DEFINITION OF COLLISION FUNCTIONS FOR CUSTOM LINKS ******************************/       
        /** Generic function to test for collision between the right arm link and the collision
        object given by the formal "target object" parameter**/
        auto knuckleObjectCollisionFunction = [this]() {
            // Check for knuckle collisions
            VectorString rightKnucleLinkNames({"MovoSevenDOF::right_knuckle_link"}); 
            return isCollidingWithTarget(rightKnucleLinkNames);
        };

        knuckleObjectCollisionFunction_ = knuckleObjectCollisionFunction;

    }



    /************** Checks collision between the links associated with the formal input parameter "linkNames", and the "target object" ********/
    bool isCollidingWithTarget(VectorString linkNames){
        // List of collision objects associated all links of the right arm
        VectorCollisionObjectPtr linkCollisionObjects;
        oppt::CollisionRequest collisionRequest;

        // Update the pose of the collision object 
        // Setup collision object structure for the target object
        geometric::Pose objectPose = geometric::Pose(LinkWorldPose(objectLink_));
        // Set worldPose
        targetBody_->setWorldPose(objectPose);
        // Update internal collision object in body
        targetBody_->updateCollisionObject();

        // Set up collision object structure for links of the robot
        auto linkScenePoses = robotEnvironment_->getGazeboInterface()->getLinksCollisionWorldPosesDirty(linkNames);
        for (size_t j = 0; j != linkScenePoses.size(); ++j) {
            fcl::Vec3f translationVector(linkScenePoses[j].position.x(),
                             linkScenePoses[j].position.y(),
                             linkScenePoses[j].position.z());
            fcl::Quaternion3f fclQuat(linkScenePoses[j].orientation.w(),
                                      linkScenePoses[j].orientation.x(),
                                      linkScenePoses[j].orientation.y(),
                                      linkScenePoses[j].orientation.z());

            fcl::Transform3f trans(fclQuat, translationVector);
            auto currentCollisionObjectPtr = collisionLinkMap.at(linkNames[j]);
            currentCollisionObjectPtr->getCollisionObject()->setTransform(trans);
            currentCollisionObjectPtr->getCollisionObject()->computeAABB();
            linkCollisionObjects.push_back(currentCollisionObjectPtr);
        }

        collisionRequest.collisionObjects = linkCollisionObjects;
        collisionRequest.enableContact = false;
        auto collisionReport = targetBody_->collides(&collisionRequest);

        return collisionReport->collides;
    }





    //////////////////////// HELPER FUNCTIONS TO QUERY THE GAZEBO INTERFACE FOR LINKS AND JOINTS ///////////////////////////
    // Create a map to the different useful joints of the robot arms and populate the lsits of joint limits
    void makeMovoJointMap() {
        movoJointMap_["linear_joint"] = nullptr;
        // Right arm joints
        movoJointMap_["right_shoulder_pan_joint"] = nullptr;
        movoJointMap_["right_shoulder_lift_joint"] = nullptr;
        movoJointMap_["right_arm_half_joint"] = nullptr;
        movoJointMap_["right_elbow_joint"] = nullptr;
        movoJointMap_["right_wrist_spherical_1_joint"] = nullptr;
        movoJointMap_["right_wrist_spherical_2_joint"] = nullptr;
        movoJointMap_["right_wrist_3_joint"] = nullptr;
        movoJointMap_["right_gripper_finger1_joint"] = nullptr;
        movoJointMap_["right_gripper_finger2_joint"] = nullptr;
        movoJointMap_["right_gripper_finger3_joint"] = nullptr;



        // Populate the joint map with their correspoding pointers
        auto joints = robotEnvironment_->getGazeboInterface()->getJoints();
        for (auto & joint : joints) {
            std::string jointName = joint->GetName();

            if (movoJointMap_.find(jointName) != movoJointMap_.end()){
                movoJointMap_.at(jointName) = joint;
            }
        }

        // Populate the upper and lower joint limit lists for each arm
        jointUpperPositionLimits = VectorFloat(7, 0);
        jointLowerPositionLimits = VectorFloat(7, 0);


    #ifdef GZ_GT_7
        jointUpperPositionLimits[0] = movoJointMap_.at("linear_joint")->UpperLimit(0);
        // Upper limit for right arm joints
        jointUpperPositionLimits[1] = movoJointMap_.at("right_shoulder_pan_joint")->UpperLimit(0);
        jointUpperPositionLimits[2] = movoJointMap_.at("right_shoulder_lift_joint")->UpperLimit(0);
        jointUpperPositionLimits[3] = movoJointMap_.at("right_elbow_joint")->UpperLimit(0);
        jointUpperPositionLimits[4] = movoJointMap_.at("right_wrist_spherical_1_joint")->UpperLimit(0);
        jointUpperPositionLimits[5] = movoJointMap_.at("right_wrist_spherical_2_joint")->UpperLimit(0);
        jointUpperPositionLimits[6] = movoJointMap_.at("right_wrist_3_joint")->UpperLimit(0);

        // Lower limit for right arm joints
        jointLowerPositionLimits[0] = movoJointMap_.at("linear_joint")->LowerLimit(0);
        jointLowerPositionLimits[1] = movoJointMap_.at("right_shoulder_pan_joint")->LowerLimit(0);
        jointLowerPositionLimits[2] = movoJointMap_.at("right_shoulder_lift_joint")->LowerLimit(0);
        jointLowerPositionLimits[3] = movoJointMap_.at("right_elbow_joint")->LowerLimit(0);
        jointLowerPositionLimits[4] = movoJointMap_.at("right_wrist_spherical_1_joint")->LowerLimit(0);
        jointLowerPositionLimits[5] = movoJointMap_.at("right_wrist_spherical_2_joint")->LowerLimit(0);
        jointLowerPositionLimits[6] = movoJointMap_.at("right_wrist_3_joint")->LowerLimit(0);
    #else
        jointUpperPositionLimits[0] = movoJointMap_.at("linear_joint")->GetUpperLimit(0).Radian();
        jointUpperPositionLimits[1] = movoJointMap_.at("right_shoulder_pan_joint")->GetUpperLimit(0).Radian();
        jointUpperPositionLimits[2] = movoJointMap_.at("right_shoulder_lift_joint")->GetUpperLimit(0).Radian();
        jointUpperPositionLimits[3] = movoJointMap_.at("right_elbow_joint")->GetUpperLimit(0).Radian();
        jointUpperPositionLimits[4] = movoJointMap_.at("right_wrist_spherical_1_joint")->GetUpperLimit(0).Radian();
        jointUpperPositionLimits[5] = movoJointMap_.at("right_wrist_spherical_2_joint")->GetUpperLimit(0).Radian();
        jointUpperPositionLimits[6] = movoJointMap_.at("right_wrist_3_joint")->GetUpperLimit(0).Radian();

        jointLowerPositionLimits[0] = movoJointMap_.at("linear_joint")->GetLowerLimit(0).Radian();
        jointLowerPositionLimits[1] = movoJointMap_.at("right_shoulder_pan_joint")->GetLowerLimit(0).Radian();
        jointLowerPositionLimits[2] = movoJointMap_.at("right_shoulder_lift_joint")->GetLowerLimit(0).Radian();
        jointLowerPositionLimits[3] = movoJointMap_.at("right_elbow_joint")->GetLowerLimit(0).Radian();
        jointLowerPositionLimits[4] = movoJointMap_.at("right_wrist_spherical_1_joint")->GetLowerLimit(0).Radian();
        jointLowerPositionLimits[5] = movoJointMap_.at("right_wrist_spherical_2_joint")->GetLowerLimit(0).Radian();
        jointLowerPositionLimits[6] = movoJointMap_.at("right_wrist_3_joint")->GetLowerLimit(0).Radian();
    #endif


    }


    // Create a map to the different useful joints of the robot arms and populate the lsits of joint limits
    void makeMovoLinkMap() {
        // Populate the joint map with their correspoding pointers
        std::vector<gazebo::physics::Link*> links = robotEnvironment_->getGazeboInterface()->getLinks();
        for (auto& currentLink : links) {
            std::string unscopedName = getScopingIndexName(currentLink->GetName(), 1);
            movoLinkMap_.insert(std::pair<std::string, gazebo::physics::Link*>(unscopedName, currentLink));
        }
    }


    // Create a map to the different useful link collision objects 
    void makeMovoLinkCollisionMap() {
        //  List of scence collision objects as described in the SDF files
        auto robotCollisionObjects = robotEnvironment_->getScene()->getRobotCollisionObjects();
        // Populate data structs with collision object pointers
        for (auto& robotCollisionObject : robotCollisionObjects) {

            std::string unscopedName = getScopingIndexName(robotCollisionObject->getName(),1);
            
            // Add collision object to map
            collisionLinkMap.insert(std::pair<std::string, OpptCollisionObject*>(unscopedName, robotCollisionObject));

        }

    }

    // Generates a multivariate gaussian distribution from the option parameters
    void makeGaussianErrorDistribution(MovoGraspingTransitionPluginOptions* options) {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        errorDistribution_ =
            std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        unsigned int actionSpaceDimension =
            robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions();


        // ##TODO:DOUBLE CHECK HERE

        Matrixdf mean = Matrixdf::Zero(actionSpaceDimension, 1);

        Matrixdf covarianceMatrix = Matrixdf::Identity(actionSpaceDimension, actionSpaceDimension);
        for (size_t i = 0; i != actionSpaceDimension; ++i) {
            covarianceMatrix(i, i) = options->jointCovariances[i];
        }

        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
    }



    // Retrieve the pose of the given link pointer
    VectorFloat getLinkPoseVec(const gazebo::physics::Link*  link) const {
        if (!link){
            return VectorFloat();
        }

        GZPose linkPose = LinkWorldPose(link);

        return getVecFromPose(linkPose);
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



    // Auxiliary function to getting and setting world poses with different apis
    inline GZPose LinkWorldPose(const gazebo::physics::Link* link) const{
    // Returns link world pose according to gazebo api enabled
    #ifdef GZ_GT_7
        return link->WorldPose();
    #else 
        return link->GetWorldPose();
    #endif

    }


    FloatType diffAngles(const FloatType& a1, const FloatType& a2) const {
        return atan2(sin(a1 - a2), cos(a1 - a2));
    }



    /*** Function to retrieve either the scoped(0) or unscoped(1) part of a link or collision object name ***/
    std::string getScopingIndexName(std::string name, int index) const{
        std::string result;
            if (name.find("::") != std::string::npos) {
                VectorString nameElems;
                split(name, "::", nameElems);
                std::vector<std::string> list;
                list.push_back(nameElems[0]);
                list.push_back(nameElems[1]);

                std::string joined = boost::algorithm::join(list, "::");

                result = joined;
            }
        return result;
    }



};

OPPT_REGISTER_TRANSITION_PLUGIN(MovoGraspingTransitionPlugin)

}
