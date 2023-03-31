Francesco Huber - 28.03.23

Robotics Assignment - 1

firstname.lastname.zip.

Instructions: 
'Tn' represents a terminal, we need 5 of them, all terminals need to be in 'ws_src'.
To run the simulation the following commands need to be called in order:
- in T2: run 'colcon build'

- in T3: run ->. install/local_setup.bash

- in T1: run -> ros2 run turtlesim turtlesim_node

- in T5: run -> ros2 service call /spawn turtlesim/srv/Spawn "{x: 8,y: 8, theta: 0.2, name: 'turtle3'}"
                ros2 service call /spawn turtlesim/srv/Spawn "{x: 2,y: 2, theta: 0.2, name: 'turtle4'}"
                ros2 service call /spawn turtlesim/srv/Spawn "{x: 8,y: 2, theta: 0.2, name: 'turtle5'}"

- in T4: run 'ros2 run turtlesim turtle_teleop_key'

- in T3: run 'ros2 run usi_angry_turtle usi_angry_node'

In order to control the teleop turtle, we need to stay on the fourth terminal
where our arrowkeys are recognized as directional input.

Turtle States:
The drawing turtle is initialized with the state "Drawing", it will write 'usi' if left at a distance of 3 from the teleop turtle.
Conversely if interrupted it will get "Angry" and chase the player till the player remains in the distance.

At the beginning of the simulation the user can spawn any number of turtles, ideally with different coordinates and names, 
the drawing turtle will chase anyone under a certain dist and then kill it.
Please keep 'turtle' in the names of spawned turtles otherwise the system won't recognize them as enemies.

As the code stands right now, all 4 tasks are achieved.
