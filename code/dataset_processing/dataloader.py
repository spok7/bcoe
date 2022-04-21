from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import numpy as np

class Dataloader:

    def __init__(self):
        raise NotImplementedError

    def query(self, point):
        raise NotImplementedError
    
    def generate_coast_segments(self):
        plt.ioff()
        m = Basemap(projection='robin',lon_0=0,resolution='c')
        coordinates = np.vstack(m.drawcoastlines().get_segments())
        return m(coordinates[:,0],coordinates[:,1],inverse=True)