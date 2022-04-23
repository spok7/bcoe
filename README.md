# Buoyancy-Controlled Ocean Exploration (BCOE)

The goal of this project is to demonstrate the feasibility of ocean navigation using buoyancy-controlled
glider-like probes by adapting classical path planning approaches to limited-actuation flow fields and
demonstrating the feasibility of exploring the ocean in this way. If this goal is reached, we intend to
expand the scope of our project to find the shortest trajectories that explore the most ocean area.
Additionally, we want to demonstrate the possibility of traveling to a specified goal location or region,
e.g. to send data, for recovery, or to investigate a site of interest.

## Project Outline

Traditional gliders utilize hydrodynamic forces to efficiently travel long distances via actively-
controlled buoyancy. However, this requires wings and complicated control algorithms, and doesn't
scale down well to ultra-small, ultra-cheap probes. Previous works have explored
purely drifting navigation on the surface of water, as well as navigation with a maximum speed in any
direction. However, a buoyancy-controlled drifting probe which can only
travel up and down is much easier to construct than one which can travel in any direction, but
nevertheless much more capable than a purely passive probe. This project aims to explore the abilities and
limitations of specifically these types of limited-actuation probes.

## Installation

For ease of use, install and use the conda_env.yaml file either through the anaconda navigator or with `conda env create -f path/to/conda_env.yaml`.
Alternatively, install the packages in conda_env.yaml manually through pip on python version 3.9.12.

Download the monthly regularized ocean dataset here from [here](https://www2.atmos.umd.edu/%7Eocean/index_files/soda3.12.2_mn_download_b.htm) and after extracting, put the 2017 dataset file in bcoe/datasets/soda3.12.2/ to work with the default values. Alternatively, change line 25 of code/dataset_processing/soda_dataloader.py as needed. By modifying this line, you can change the dataset to any SODA regularized ocean dataset.

## Usage

It's easiest to test the code's functions within its specific files. Below are examples of using the visualization functions.

If you wish to do more complex operations, the code is filled with docstrings and comments to help you along.

### SODA Dataloader (code\dataset_processing\soda_dataloader.py)

```python
soda = SODA() # initialize the dataset

# the basic 2D draw map command; the parameters can be combined as needed
soda.draw_map() # draws a map of the surface level currents
soda.draw_map(resolution='i') # same as above but with better resolution
soda.draw_map(currents=False) # same as above but without currents
soda.draw_map(data=[(lon1,lat1,col1,siz1),(lon2,lat2,col2,siz2),...] # same as above but also draws points at locations with given colour and size

soda.draw_3D_map() # draws a 3D map of the Gulf Stream
soda.draw_3D_map([lon1,lon2,lat1,lat2]) # draws a 3D map of the given bounded area

graph = soda.make_graph() # generate a graph from the current dataset; takes over 2h
graph = soda.make_graph(month) # same as above, but at the specified month
np.save("graph", graph) # saves the graph as 'graph.npy'; useful

soda.ds.close() # close the dataset
```

### Dijkstra's Algorithm (code\dijkstra\dijkstra.py)

```python
soda = SODA() # initialize the dataset
graph = np.load("graph.npy", allow_pickle=True) # load your saved graph

start = [lon, lat]
goal = [lon, lat]

# plot a path from the start to the goal in purple if it exists and print the path cost
# else plot all of the explored nodes
# to test reachability from a certain point, set goal to None
graph_dijkstra(soda, graph, start, goal)

soda.ds.close() # clost the dataset
```

## Code Structure

We will apply the RRT/RRT*/Whatever-else-we-get-around-to algorithm to explore the set of states reachable from a given start configuration and whether path planning to specific goals is possible, in both synthetic and real-world flow fields. The main project components will be as follows:

1. Main: Loads dataset(s), sets up problem, runs planner, visualizes results
2. Planner: RRT or similar. Runs RRT(*), and when adding a new node, it picks a random actuation (up or down) and runs it through the model.
3. Model: Flow field and integrator. Input: an (array of) points and a delta t. Output: where those points ended up. Can be either synthetic or using a real dataset.

## Class Structure

This program will use classes. There should be a main class of which all the other things are methods, and maybe subclasses. The main class can include all sorts of random crap that needs to get passed down deeply, like the dataset etc.
