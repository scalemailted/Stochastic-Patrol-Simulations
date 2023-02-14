# Simulations for Stochastic Patrol Strategies

---

## Intelligent Agent Design - Objective 
> Suppose you are given a 6m x 5m target area to look for adversarial intruders. Write and implement an algorithm for a quadrotor UAV so that it can patrol the target area without revealing its strategy to intruders.
>
> *Constraint: Don’t use an area coverage planning strategy, it might be too easy to decode for intruders.*
--- 

## Motivation - Two Considerations

The motivation behind this project was to find a solution to the conflicting demands of unpredictable patrols and guaranteed coverage of the observable space. In many real-world scenarios, it is important to have a patrol scheme that is unpredictable to prevent pattern recognition and exploitation. At the same time, it is equally important to ensure that the entire domain space is surveyed.

Traditional approaches, such as using a completely deterministic method, fail to meet the requirement of unpredictability, while a purely stochastic approach does not guarantee that the entire space will be covered. This project aimed to bridge this gap by devising a non-predictable patrol strategy that still guarantees full coverage of the observable space. The end goal was to provide a solution that balances the two conflicting demands, ensuring both unpredictability and guaranteed coverage.

---
## Approach - Stochastic + Deterministic

The patrol approach employs a combination of random sampling and weighted path selection to ensure a complete survey of the patrol space. The random sampling method used is Roulette Wheel Selection, which guarantees that every area of the space is surveyed. The approach immediately repeats the survey when all cells have been visited in a completely different fashion, where no two cycles are similar.

The path to the waypoint takes into consideration weights that maximize the discovery of the unvisited spaces while also allowing the observer to revisit previously visited cells. The weights assigned to each cell are a combination of a penalty factor and a factor that accounts for the unvisited cells in the space. The penalty factor is applied to cells that have already been visited, while the unvisited cells factor encourages the observer to visit unvisited areas.

To allow for an even more comprehensive survey and optimize each pass, the observer's path to the waypoint is not restricted to a straight line. The paths are allowed to be curvy, which helps to reduce the likelihood of a patterned survey and ensures a more thorough exploration of the space. 

The combination of random sampling, weighted path selection, and curvy paths provides a flexible and efficient approach for patrolling a designated space.

---

## API Documentation

📚 **[API - Python + CoppelliaSim](API.md)**

---


## CoppeliaSim Simulation for Stochastic Patrol Plan

![CoppeliaSim Demo](./Assets/coppeliasim-demo.gif)


---


## JavaScript Demo for Stochastic Patrol Plan

[![Demo: Patrol Alone](./Assets/jsdemo-alone.gif)](https://scalemailted.github.io/Stochastic-Patrol-Simulations/WebApp/Patrol-Alone/)
[![Demo: Patrol Adversary](./Assets/jsdemo-adversary.gif)](https://scalemailted.github.io/Stochastic-Patrol-Simulations/WebApp/Patrol-Adversarial/)




