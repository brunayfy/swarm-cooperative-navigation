# Pesquisa TCC

## Middleware architectures:**

Among the most popular ROS [140], OPRoS [100], OpenRTM [13], Orocos [151], and Gen oM [73].ArGos

## **Modeling the physical environment:**

Static or dynamic Previously known map or unknown to be mapped by the robots Low fidelity environment to simulate bugs during field tests.

## **Robot communications:**

Mix of individual information and collective shared informationMultifunction robots: beacon pheromones and followerIndividualist vs collective foragerBlockchain -Eduardo Castell’o Ferrer MITIn Beckers et al. (1994) and Svennebring and Koenig (2004) authors prove that **stigmergic**-based works (pheromone trails) have shown to efficiently coordinate a team of robots and to allow them to quickly explore a given terrain.

## **Movement**

ORCA- Optimal Reciprocal Collision Avoidance Virtual Group Velocity Obstacles (VGVO)Brazil Nut Effect - segregation algorithmParticle Swarm Optimization (PSO) is a population-basedmetaheuristic that iteratively tries to improve a set of candidatesolutions represented by particles in the solution space.

## **Tasks:**

Foraging, pattern formation, aggregation, hole avoidance, self deployment and chain formation.



![img](https://lh3.googleusercontent.com/Jk2eaLPBIxdN-lrN0JJrZ9MGhFicOuIwtRiVC9e_N89fOMzBkvTQOFAg9IJ036gfd81AxUvZiCpENGNOvhlw78nqVPPVXnLzY54BCa_M6oaJcsNoJoawCJUKzXHjWimW2Tb_1CES)



# Cooperative Navigation in Robotic Swarms

T : a target robot

S :  robot of the swarm that can service the task, needs to navigate to a given target robot T.

Task:  S have to find T through cooperative support from the other robots in the swarm when no environment maps or external localization systems are available to the robots. The other robots offer help through communication but do not deviate from their task. (differs from the typical ones previously considered in swarm navigation, in which some robots adapt their own behavior (or even stand still playing the role of environment landmarks) to support the navigation of other robots, or, more in general, where all robots are involved in solving a single task cooperatively)

## Algorithm

 Each robot A coming in communication range of a target robot T , and receiving its periodic broadcasts, stores information about T in a local data structure, which we call a **navigation table**. This information consists of a sequence number, indicating the relative age of the message, and a distance value, which is an estimate of the navigation distance to T . As A moves around, it updates the information in its navigation table, and periodically broadcasts it to neighboring robots. This way, navigation information can travel through the (possibly intermittently connected) mobile ad hoc network (MANET) formed among the swarm of robots by being carried on board of the mobile robots. A searching robot S receiving new navigation information from a robot B, compares this new information to previously received navigation information, and moves towards B’s location if the new information is better















