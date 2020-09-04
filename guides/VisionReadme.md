## Vision Module: 
The demo comes with its own version of vision module implementation. The module is designed in such a way that it will detect the object, table (support) and any other obstacle. Please note that for this version of the demo, it is advised that you keep the navigation path free of any obstacles. The vision module detects the x and y co-ordinate of the object location on the support plan and the z co-ordinate is calculated using the dimension of the table and object model provided by the user.

It is understandable that some users would want to plug in their own vision module. The demo allows its own vision module to be replaced by a custom user built module using the following steps : 

1. The custom vision module must we wrapped using a python interface. The interface needs to be an implementation of the abstract class in [../movo/popcorn_manager/scripts/Vision.py](../movo/popcorn_manager/scripts/Vision.py). The abstract class implementation requires you to define a getVision function in your vision interface wrapper which is called in the [../movo/popcorn_manager/scripts/demo_main.py](../movo/popcorn_manager/scripts/demo_main.py)
2. Place your version of the Vision Interface.py file in [../movo/popcorn_manager/scripts](../movo/popcorn_manager/scripts)
3. Replace the filename and classname in the [../movo/popcorn_manager/scripts/config.ini](../movo/popcorn_manager/scripts/config.ini) file with the file and class that you want to use

The getVision function of the vision interface is designed to return the location and orientation of the target object and table(support). It is supposed to return two lists, the details of which are as follows : 

<pre><code>
getVision()
.
.
.
return initial_objs,obstacles

------------------------------------------------------
type(initial_objs) = python list
type(obstacles) = python list

initial_objs is supposed to have 2 elements. Let initial_objs[0] be target_object and initial_objs[1] be support.
type(target_object) = python list
type(support) = python list

Both target_object and support contain just one element, representing the collision object for the target object and table respectively.

type(target_object[0]) = moveit_msgs/CollisionObject
type(support[0]) = moveit_msgs/CollisionObject

type(target_object[0].primitive_poses) = geometry_msgs/Pose
type(support[0].primitive_poses) = geometry_msgs/Pose

target_obj[0].primitive_poses.position.x = x coordinate of target object location
target_obj[0].primitive_poses.position.y = y coordinate of target object location
target_obj[0].primitive_poses.position.z = z coordinate of target object location

support[0].primitive_poses.position.x = x coordinate of table location
support[0].primitive_poses.position.y = y coordinate of table location
support[0].primitive_poses.position.z = z coordinate of table location


You can design the structure of the obstacles list in a similar way, but for the purpose of this demo you can leave it empty as well.

</code></pre>

For example :  

Assume you have your own vision module implemented and you have a python wrapper module named My_vision_interface.py with a class defined as MyVisionInterface. First of all you will need to edit your python script so that it implements the abstract class in [../movo/popcorn_manager/scripts/Vision.py](../movo/popcorn_manager/scripts/Vision.py). This means that you will have to implement the the aforementioned getVision function within the class MyVisionInterface with the required return type as well. Remember, its not necessary that the second list (containing obstacles) is populated (you can return an empty list). </br>
You need to place your wrapper file My_vision_interface.py in [../movo/popcorn_manager/scripts](../movo/popcorn_manager/scripts). All the dependencies for the vision module can be placed as a separate package [outside popcorn_manager](../movo) (just as we have popcorn_vision). Finally, as mentioned in the list above, you will need to edit the [../movo/popcorn_manager/scripts/config.ini](../movo/popcorn_manager/scripts/config.ini) as follows :

<pre><code>
[Vision]
filename = My_vision_interface
class = MyVisionInterface
</code></pre>

