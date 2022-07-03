# Dynamic Swarm Disablement (DSD)

Motivated by the use of robots for pest control
in agriculture, this work introduces the Multi-Robot Dynamic
Swarm Disablement problem, in which a team of robots is
required to disable a swarm of agents (for example, locust
agents) passing through an area, while minimizing the swarm
members’ accumulated time (equivalent to the accumulated
damage they cause) in the area. Showing that the problem
is hard even in naive settings, we turn to examine algorithms
seeking to optimize the robots’ performance against the swarm
by exploiting the known movement pattern of the swarm agents.
Motivated by the poor performance when a weak group of
robots attempts to catch a large swarm of agents, whether it is
a significant numerical minority or poor speed gaps, we suggest
the use of blocking lines: the robots form lines that block the
agents along their movement in the environment. We show by
both theoretical analysis and rigorous empirical evaluation in
different settings that these algorithms outperform common
task-assignment-based algorithms, especially for limited robots
versus a large swarm.


TODO add link to paper

## Deterministic movement

### Full-blockage case

#### StaticLine

![StaticLinePlanner](readme_gifs/StaticLine_rm.gif)

#### TravelingLine

![StaticLinePlanner](readme_gifs/TravelingLine_rm.gif)

#### SeparateTraveling

![StaticLinePlanner](readme_gifs/SeparateTraveling_rm.gif)

### Partial-blockage

#### StaticLineLack

![StaticLinePlanner](readme_gifs/StaticLineLack_rm.gif)

#### SeparateStaticLack

![StaticLinePlanner](readme_gifs/SeparateStaticLack_rm.gif)

#### AdditiveStaticLack

![StaticLinePlanner](readme_gifs/AdditiveStaticLack_rm.gif)
