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

## API Documentation

[API - Python + CoppelliaSim](API.md)

---


## CoppeliaSim Simulation for Stochastic Patrol Plan

![CoppeliaSim Demo](./Assets/coppeliasim-demo.gif)


---


## JavaScript Demo for Stochastic Patrol Plan

[![Demo: Patrol Alone](./Assets/jsdemo-alone.gif)](https://scalemailted.github.io/Stochastic-Patrol-Simulations/WebApp/Patrol-Alone/)
[![Demo: Patrol Adversary](./Assets/jsdemo-adversary.gif)](https://scalemailted.github.io/Stochastic-Patrol-Simulations/WebApp/Patrol-Adversarial/)




