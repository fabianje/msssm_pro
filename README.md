# MATLAB Spring 2014 – Research Plan 

> * Group Name: intersection_crowd_behavior
> * Group participants names: Forni Luca; Jenelten Fabian
> * Project Title: Simulation and optimization of pedestrian flow at an intersection (FS2014)

## General Introduction
Every one knows the de-accelerating effect of overcrowded pedestrian streets and intersections:
People crossing a street or walking the opposite direction jam the way of other people. Accordingly,
if many people try to cross the street and if many people walk on different directions at the same
time, the dynamic gets more and more inert. Sometimes even total blocked pedestrian streets occur.
The purpose of our project will be to model and simulate this behaviour. By changing the conditions
we will try to optimize its dynamic. Using the synthesized data we will try to find a realizable
solution that can be convert to real urban problems.

## The Model
Intersection:
Our model will contain an intersection with exactly four branches. 

Pedestrians:
Each pedestrian will have an idea of where he want to go. He will follow a preferable trajectory. He can, however, decide to leave this trajectory and
even change its direction if necessary. The pedestrian will be in contact with his environment and with other pedestrians by the so called social forces.


Environment:
> * boundary of the intersection: pedestrian cannot pass this borders.

Initial conditions:
> * number of branches: four
> * shape of the intersection
> * size of the streets and the intersection

Measurements:
> * Average time effort to reach the final target.
> * Average velocity.
> * Averaged number of pedestrians walking slower than 0.2 m/s. This value indicates the loss of dynamic.
> * Averaged velocity of all pedestrian with preferred velocity >1.5. This value compared with the average velocity answers the question whether "stressing" does help or not.
> * Grid matrix with velocity_distribution.

Control "Parameters" (optimization methods)
> * Generating rate: probability for a pedestrian to appear at one branch of the intersection
> * Additional leading elements such as roundabouts, traffic lights and additional borders

## Fundamental Questions
> * Which optimization method can relieve overcrowded intersections and yielding a better pedestrian flow?
> * Which method is the most efficient one?
> * Is the natural human self organization better than using optimization methods?

## Expected Results
First of all, we will find a clear correlation between density and speed. Other results might involve some optimization structures:
> * With a low pedestrian density the system with the fastest dynamic will be the natural, uncontrolled intersection.
Each controlling element will lead to a decreasing flow rate.  
> * If the pedestrian density reaches a specific value, the average velocity and the
pedestrian flow can be increased using roundabouts. Traffic lights might improve the dynamic slightly.
> * Controlling the trajectories (using additional boundaries) will probably yield in the best results.

## Research Methods
We will use an agent based simulation using a social force method. A social force is a physical meaningless force that 
indicates the "need" to arrive to somewhere or to evade something or someone. The forces will be derived by estimating a reasonable
magnitude with a logical direction. The magnitude will in the first step depend on unknown parameters. To find those parameters we will
rebuilt a straight road of an experimental video [2] and adjust the parameters until the pedestrians walk close to the pedestrians 
trajectory in reality. Of course, such an optical evaluation will not produce the best possible model (as a professional video tracking would be),
but we expect an sufficient realistic pedestrian behaviour.


## References
> *[1]  Ramin Mehran, Alexis Oyama, Mubarak Shah (2009), "Abnormal crowd behavior detection using 
social force model." 
> *[2] Pedestrian-dynamics experiment -- lane formation in counter flow: http://www.youtube.com/watch?v=J4J__lOOV2E
