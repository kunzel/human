Package Description
=====================
This package contains human_following and its msgs

The main function of this package is making the robot following human beings and recording their activities

The State Machine inside the package contains:

  Start stage - Wandering stage - Following stage - Local-searching stage
  
Getting Start
=========================
Before you can launch this package, you need:
    
    (on bob)
    roscore
    bring_up
    
    (on bobl)
    openni
    scitos_ptu
    people_track

To start this human_following module, simplly launch:
    
    roslaunch human_following human_following_server.launch
    roslaunch human_following human_following_client.launch
    
Configurations
==========================
To change the settings of this package, find:
    
    /launch/following_param.launch
    
Parameters you can change are listed below:

    envr      (system mode): sim/real
    wandering_mode         : normal/wait
    time (how long it runs): (int)
    wander_area            : (polygon)
    follow_area            : (polygon)
    max_t_frames(tolerence): (int)
    alpha                  : (double)
    distance(safe distance): (double)
    
    
