# Buoyancy-Controlled Ocean Exploration (BCOE)

The goal of this project is to demonstrate the feasibility of ocean navigation using buoyancy-controlled
glider-like probes by adapting classical path planning approaches to limited-actuation flow fields and
demonstrating the feasibility of exploring the ocean in this way. If this goal is reached, we intend to
expand the scope of our project to find the shortest trajectories that explore the most ocean area.
Additionally, we want to demonstrate the possibility of traveling to a specified goal location or region,
e.g. to send data, for recovery, or to investigate a site of interest.

## Installation and Usage

Make a new python 3.9 conda environment and install whatever packages are necessary (TBD)

## Project Outline

Traditional gliders utilize hydrodynamic forces to efficiently travel long distances via actively-
controlled buoyancy. However, this requires wings and complicated control algorithms, and doesn't
scale down well to ultra-small, ultra-cheap probes \[CITATION NEEDED\]. Previous works have explored
purely drifting navigation on the surface of water, as well as navigation with a maximum speed in any
direction \[Citations from LIT REVIEW\]. However, a buoyancy-controlled drifting probe which can only
travel up and down is much easier to construct than one which can travel in any direction, but
nevertheless much more capable than a purely passive probe. This project aims to explore the abilities and
limitations of specifically these types of limited-actuation probes.

## Code Structure

We will apply the RRT/RRT*/Whatever-else-we-get-around-to algorithm to explore the set of states reachable from a given start configuration and whether path planning to specific goals is possible, in both synthetic and real-world flow fields. The main project components will be as follows:

1. Main: Loads dataset(s), sets up problem, runs planner, visualizes results
2. Planner: RRT or similar. Runs RRT(*), and when adding a new node, it picks a random actuation (up or down) and runs it through the model.
3. Model: Flow field and integrator. Input: an (array of) points and a delta t. Output: where those points ended up. Can be either synthetic or using a real dataset.

## Class Structure

This program will use classes. There should be a main class of which all the other things are methods, and maybe subclasses. The main class can include all sorts of random crap that needs to get passed down deeply, like the dataset etc.
