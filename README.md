# MATLAB Spring 2014 – Research Plan (Template)
(text between brackets to be removed)

> * Group Name: intersection_crowd_behavior
> * Group participants names: Forni Luca; Jenelten Fabian
> * Project Title: Intersection Crowd Behavior

## General Introduction

Every one knows the deaccelerating effect of overcrowded pedestrian streets and intersection: People crossing a street or walking the oposite direction jam the way of other people. Accordingly, if many people try to cross the street and if many people walk on different directions at the same time the dynamic gets more and more inert.
The purpose of our project will be to model and simulate that behavior. By changing the conditions we will try to optimate its dynamic. Usgint the synthesized data we will try to find a realizable solution that can be convert to real urban problems


## The Model
Intersection:
Our model will contain an intersection with an variable number of branches.

Pedestrians:
Each pedestrian will have an idea of where he want to go but not of how he will reach his target. Thus, he will decides his next step continuously. The pedestrian will be in contact with his enviroment and will be able to comminicate with other pedestrians.
The people will, in general, follow both directions, forwards and backwars.


Environment:
> * disturbence such as crossing cars or trams.


Initial conditions:
> * number of people starting at the beginning of each branch
> * size of the streets and the intersection
> * number of brachnes

measurements:
> * mass flow trough each branch
> * avarage velocity that the people have to cross the intersection
> * critical pedestrian density (density at which the avarge velocity is less than 10% of walking speed)

Control "Parameters" (optimization methods)
> * Additional leading elements such as roundabouts, traffic lights, guide trajectories




## Fundamental Questions

> * Which optimization method can relieve overcrowded intersections and yielding a better pedestrian flow?
> * Which method is the most efficient one?
> * Are those optimizations realizable?
> * Is the number of brachnes influencing the result?
> * How does disturbaces influence the systems dynamic?


## Expected Results

First of all, we will find a clear correlation between density and speed. Other results might be:
With a low pedestrian density the the system with the fastest dynamic will be the natural uncontrolled intersection. Each controlling element will lead to a decreasing flow rate.  
If the pedestrian density reaches a specific value, the avarage velocity and the pedestrian flow can be increased using roundabouts. Traffic lights might improve the dynamic slightly.
Controlling the trajectories will yield in the best reusults.  This method is a theoretical solution only since people can not be forced to follow a specific trajectory. 



## References 

Papers:
> * Analytical Approach to Continuous and Intermittent Bottleneck Flows -- Dirk Helbing and Anders Johansson
> * Dynamics of crowd disasters: An empirical study -- Dirk Helbing and Anders Johansson
> * Simulating dynamical features of escape panic -- Dirk Helbing, Illes Farkas³ & Tamas Vicsek

Projects from previous years:
> * Intersections with pedestrians -- Marcel Arikan, Nuhro Ego, Ralf Kohrt --  https://github.com/nuhro/Intersection-Problem


## Research Methods


We will use the Cellular Automata model as the major method. 


