import numpy as np
import random
import matplotlib.pyplot as plt
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'dataset_processing'))
from soda_dataloader import SODA

class Node:
    def __init__(self, parent=None, cost=0):
        self.parent = parent
        self.cost = cost
        
    def __str__(self):
        return str(self.cost)
    
    def __eq__(self, other):
        return self.cost == other.cost

class Dijkstra:
    def __init__(self, graph, start, goal):
        self.graph = graph
        self.start = start
        self.goal = goal
        
    def find_leaves(self, idx, cost=0):
        """A very complicated function to retrieve possible leaf nodes.

        Args:
            idx (tuple(x,y)): The tuple index of the current node.

        Returns:
            list: A list of key,value pairs corresponding to the leaf nodes.
        """
        
        # retrieve possible leaves with their related costs
        leaves = np.array([[-1,1], [0,1], [1,1], [1,0], [1,-1], [0,-1], [-1,-1], [-1,0]]) + np.array(idx)
        costs = self.graph[idx[1], idx[0]] + cost
        
        # wrap around on the x
        leaves[leaves[:,0]==-1] += [self.graph.shape[1], 0]
        leaves[leaves[:,0]==self.graph.shape[1]] *= [0, 1]
        
        # keep all y values within boundaries and costs less than infinity
        mask = (leaves[:,1] >= 0) & (leaves[:,1] < self.graph.shape[0]) & (costs != np.inf)
        leaves = leaves[mask]
        costs = costs[mask]
        
        return [(tuple(l), Node(idx, c)) for l, c in zip(leaves, costs)]
        
    def run(self):
        visited = {self.start:Node()}
        leaves = {}
        leaves.update(self.find_leaves(self.start))
        
        while leaves:
            # iterate through all of the leaf nodes (nodes with unvisited members)
            best_idx, best_node = None, None
            for idx, n in leaves.items():
                if not best_idx or n.cost < best_node.cost:
                    best_idx, best_node = idx, n
            
            del leaves[best_idx]
            
            # skip over visited values, since they already have the lowest cost
            if best_idx in visited:
                continue
            
            # add node to visited values, and return them if the node was the goal node
            visited[best_idx] = best_node
            if best_idx == self.goal:
                return visited
            
            
            new_leaves = self.find_leaves(best_idx, best_node.cost)
            for idx, n in new_leaves:
                if idx not in visited and (idx not in leaves or n.cost < leaves[idx].cost):
                    leaves[idx] = n
                    
        return visited
    
    def get_paths(self, nodes):
        path_x, path_y = [], []
        ci = self.goal
        cn = nodes[ci]
        while cn.parent:
            path_x.append(ci[0])
            path_y.append(ci[1])
            ci = cn.parent
            cn = nodes[ci]
        path_x.append(ci[0])
        path_y.append(ci[1])
        return path_x[::-1], path_y[::-1]

def fake_data(x, y):
    graph = np.ones((y, x, 8)) * np.inf
    
    for i in range(x):
        for j in range(y):
            indices = []
            directions = random.randint(1, 8)
            while len(indices) < directions:
                index = random.randint(0, 7)
                if index in indices:
                    continue
                indices.append(indices)
                graph[j, i, index] = random.randint(1, 9)
    
    return graph
        
if __name__ == "__main__":
    random.seed(42)
    # graph = fake_data(5, 4)
    # print(graph)
    # print(graph.shape, graph[1,0])
    # D = Dijkstra(graph, (0, 1), (2, 3))
    # # print(D.find_leaves((4, 3)))
    # # print(D.find_leaves((0, 0)))
    # # print(D.find_leaves((0, 1)))
    # nodes = D.run()
    # print("Success:", D.goal in nodes)
    
    # path_x, path_y = D.get_paths(nodes)
    
    # plt.plot(path_x, path_y)
    # plt.show()
    
    soda = SODA()
    graph = np.load("D:\\sync\\documents\\academics\\c. grad school\\s2-AER1516-planning-for-robotics\\project\\graph.npy", allow_pickle=True)
    
    st_johns = [-48, -48, 44, 44]
    miami = [-80, -80, 25, 25]
    st_johns_idx = soda.return_bounded_area(st_johns)[::2]
    miami_idx = soda.return_bounded_area(miami)[::2]
    # soda.draw_map(data=[([miami[0], st_johns[0]], [miami[2], st_johns[2]], 'r', 10)])
    print(miami_idx, st_johns_idx)
    D = Dijkstra(graph, miami_idx, st_johns_idx)
    nodes = D.run()
    if D.goal in nodes:
        print("Success!")
        print("Cost:", nodes[D.goal].cost)
        path_x, path_y = D.get_paths(nodes)
        reached_x, reached_y, reached_cost = [], [], []
        for (x, y), n in nodes.items():
            reached_x.append(x)
            reached_y.append(y)
            reached_cost.append(n.cost)
        sgx, sgy = soda.convert_to_lon_lat([D.start[0], D.goal[0]], [D.start[1], D.goal[1]])
        px, py = soda.convert_to_lon_lat(path_x, path_y)
        rx, ry = soda.convert_to_lon_lat(reached_x, reached_y)
        
        soda.draw_map(data=[(rx, ry, reached_cost, 20),
                            (px, py, 'purple', 30),
                            (sgx, sgy, 'red', 50)], currents=True)
    else:
        print("Failure")
        print(nodes.keys())
    
        
    # D = Dijkstra(graph, miami_idx, None)
    # nodes = D.run()
    # reached_x, reached_y, reached_cost = [], [], []
    # for (x, y), n in nodes.items():
    #     reached_x.append(x)
    #     reached_y.append(y)
    #     reached_cost.append(n.cost)
    # rx, ry = soda.convert_to_lon_lat(reached_x, reached_y)
    # sx, sy = soda.convert_to_lon_lat([D.start[0]], [D.start[1]])
    # soda.draw_map(data=[(sx, sy, "red", 50),(rx, ry, "purple", 30)], currents=True)
        
        
    
        
        