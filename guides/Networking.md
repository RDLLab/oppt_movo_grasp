## Networking Guide
In order to be able to interface the MOVO module with the Planner module, the modules must be able to talk to each other.
Some useful references for this are
[ROS Multiple Machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
[ROS Networking](http://wiki.ros.org/ROS/NetworkSetup)
[Kinova Setup](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos)

In this demo, the networking between the modules is done via a ROS Network.

Following the guide at [ROS networking](http://wiki.ros.org/ROS/Tutorials/MultipleMachines),
first we have to tell each computer system about the other.

In this demo, we follow KINOVA's ROS networking conventions and use *"movo2"* computer from MOVO as the *ROS_MASTER*, and connect the *plannerPC* to *"movo2"* as a *ROS_SLAVE*.


## plannerPC
Here, we need to add the ip address of the computer we want to talk to. In this example, the addresses for both computers on MOVO are added. However, for the demo, only communication with *"movo2"* is necessary.

### Establishing a connection to "movo2"
```bash

# Add ip entries into host file. You will need sudo access for this
planner@plannerPC:~$ sudo gedit /etc/hosts
```
Inside the hosts file, make sure to add the following mappings
``` 
    10.66.171.2 movo1
    10.66.171.1 movo2
```

Check if pings can be sent to *"movo2"*.

``` bash
# Ping movo2
planner@plannerPC:~$ ping movo2
```

If *"movo2"* was not reacheble, check your physical network link between the *plannerPC* and *"movo2"*.

### Setting the plannerPC as the ROS_SLAVE
Once the plannerPC is ready to communicate with "movo2", it needs to be set up as a ROS_SLAVE. To do this execute

``` bash
# Point ROS_MASTER_URI to MOVO2
planner@plannerPC:~$ export ROS_MASTER_URI=http://MOVO2:11311/
```


## MOVO2 Computer:
Following similar steps to the plannerPC, *movo2* needs to enter the ip-address of plannerPC into its hosts file.

### Establishing a connection to "plannerPC"

``` bash
movo@movo2:~$ sudo gedit /etc/hosts
```

Inside the hosts file, enter the ip-address and name of the computer used as the plannerPC. Following our convetion, this would look like
```
<planner_pc_ip_address> plannerPC
```
Check if pings can be sent to *plannerPC*

``` bash
# Ping movo2
movo@movo2:~$ ping plannerPC
```

If *"plannerPC"* was not reacheble, check your physical network link between the *plannerPC* and *"movo2"*.
Otherwise, both machines should now be connected, and ready to communicate throught the ROS Network.


## Testing the ROS Network
To verify that the ROS Network between the system has been set up, on the ROS_SLAVE (*plannerPC*), run
``` bash
planner@plannerPC:~$ rostopic list
```
If information from the ROS_MASTER is available (e.g the ROS Nodes running on *movo2*),
plannerPC, has been established as a ROS_SLAVE to the network.




