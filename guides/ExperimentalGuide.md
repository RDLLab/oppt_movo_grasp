## User Input to demo:
For this demo we require the user to provide the dimensions of the height of the table used, and the height of the cydrical object 
used. This input is provided via a yaml file located in the movo module at "movo/popcorn_vision/config/popcorn_tabletop_objects.yaml" or [here](https://github.com/RDLLab/oppt_movo_grasp/blob/master/movo/popcorn_vision/config/popcorn_tabletop_objects.yaml) to allow the manipulator to estimate a good height to operate at.

To provide the input information via the yaml file described above, modify the dimensions of the "tube" for the target object, and "table" for the table used in the experiment.

The following highlights the relevant sections within the yaml file that need to be modified to set the input dimensions.
```
# http://docs.ros.org/api/shape_msgs/html/msg/SolidPrimitive.html
# uint8 type
# uint8 BOX=1
# uint8 SPHERE=2
# uint8 CYLINDER=3
# uint8 CONE=4

popcorn_tabletop_objects:
  tube:
    primitive_type: 3
    dimension:
      radius: 0.0315
      height: 0.135

popcorn_tabletop_object_supports:
  table:
    primitive_type: 1
    dimension:
      box_x: 0.600
      box_y: 1.200
      box_z: 0.740

```


## Using a different target object:
To use a different target object, provide the appropiate dimensions for the input of the demo as described in the section above.
Additionally, the dimensions of the new object would need to be updated on the planner's representation of the environment.
This update needs to be reflected on "planner/popcorn_oppt_ros_interface/src/MovoModels/MovoGraspingEnvironment/MovoGraspingEnvironment.sdf" or [here](https://github.com/RDLLab/oppt_movo_grasp/tree/master/planner/popcorn_oppt_ros_interface/src/MovoModels/MovoGraspingEnvironment)


The relevant section within the environment model file to input the cylinder's dimensions of the different target object is shown below. Modify the *radius* and *length* tags accordingly.
```
  <collision name='objCollision'>
            <geometry>
              <cylinder>
                <radius>0.0315</radius>
                <length>0.135</length>         
              </cylinder>
            </geometry>
          </collision>
 ```

