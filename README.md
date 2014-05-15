# MATLAB Spring 2014 – Research Plan 

> * Group Name: intersection_crowd_behavior
> * Group participants names: Forni Luca; Jenelten Fabian
> * Project Title: Simulation and optimization of pedestrian flow at an intersection (FS2014)


## Introduction

Everybody who once walked past a buisy intersection knows the struggle of constantly trying 
to maintain his moving direction amongst people walking in every direction possible.
It is hard to not get influenced by other pedestrians, which by doing so leads to a loss of the proper
path.
In our simulation we want to test and analyze the dynamics of a crowded intersection and determine if 
the flow of people through this intersection can be maximized by applying rules that already exist in 
traffic regulations for vehicles on the road. 
We managed to apply some ground rules to each pedestrian so that it can look around to judge the 
environmental situation dictated by other pedestrians around him and take action , i.e. not colliding 
with other people on his way to his specific target finding a way through the crowd to his destination
point.
It can be observed that this individually based behavior can become problematic if the broadness of
the road shrinks to a critical point or the density of the pedestrians arises to a point where jams
are formed and groups of people get stuck decreasing furthermore the fluidity of future passages by
other pedestrians through that jam. We think that applying some non-individualistic ground rules can
significantly improve the dynamics of the crowd and the speed at which a person can cross the 
intersection.


## Fundamental question

Which, if there is any, ground rule or law that is currently active in vehicle traffic for regulating
the flow at an intersection can be applyed to pedestrians and help improve what was an individualistic
organisation.


## Expected results

We expect to see an improvement in fluidity in the crowd due to universal laws. Everybody will follow 
the ground rule first, only after that the individual behavior comes into play. 


## Methods

We used a agent agent based simulation for our simulation. This is applied to a metod for simulating 
pedestrians called "social forces". This simple method states that In reality we tend to conserve our 
nearest personal space to ourselves. We don’t feel comfortable sharing this space with other people, 
except the conditions of our environment absolutely forces us to. It is possible to implement this 
idea by applying forces to each agent-to-agent interaction. If the distance between agents is large 
the force will have little or no impact on the behavior of singles. According to that the more two 
agents get close to one another the more this force increases and tends the two agents to increase 
the distance between each other. In this way we can simultaneously evaluate the behavior of the 
individual and the avoidance of objects, i.e. agents, on its path.


## References

[1]  Ramin Mehran, Alexis Oyama, Mubarak Shah (2009), "Abnormal crowd behavior detection using 
social force model." 
