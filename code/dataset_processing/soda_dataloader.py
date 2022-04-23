import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'dataset_processing'))

# from dataloader import Dataloader
from dataloader import Dataloader
from netCDF4 import Dataset


from mpl_toolkits.basemap import Basemap
import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import interpn

from time import perf_counter

class SODA(Dataloader):
    
    def __init__(self, year="2017"):
        """Initialize the SODA dataset with the given year.

        Args:
            year (str, optional): Selects the year of dataset to use. Defaults to "2017".
        """
        filename = os.path.join(os.path.dirname(__file__), f'../../datasets\soda3.12.2\soda3.12.2_mn_ocean_reg_{year}.nc')
        self.ds = Dataset(filename)
        
        self.start = (0.25, -74.75) # 74.75S, 0.25E
        self.inc = 0.5
        self.x_range = np.concatenate((np.arange(self.start[0], 180, self.inc),
                                       np.arange(-180 + self.start[0]%1, self.start[0], self.inc)))
        self.y_range = np.arange(self.start[1], 90, self.inc) 
        self.z_range = [ 5.03355, 15.10065, 25.21935, 35.35845, 45.57635, 55.86325, 66.26175, 76.80285, 87.57695, 98.62325,
                        110.0962, 122.1067, 134.9086, 148.7466, 164.0538, 181.3125, 201.2630, 224.7773, 253.0681, 287.5508,
                        330.0078, 382.3651, 446.7263, 524.9824, 618.7031, 728.6921, 854.9935, 996.7153, 1152.376, 1319.997,
                        1497.562, 1683.057, 1874.788, 2071.252, 2271.323, 2474.043, 2678.757, 2884.898, 3092.117, 3300.086,
                        3508.633, 3717.567, 3926.813, 4136.251, 4345.864, 4555.566, 4765.369, 4975.209, 5185.111, 5395.023]
        
        # print(self.ds["u"])
        
        self.coastal_lons, self.coastal_lats = self.generate_coast_segments()
        self.coastal_lons, self.coastal_lats = np.array([[x, y] for x, y in zip(self.coastal_lons, self.coastal_lats) if self.point_within_bounds([x, y])]).T
        
    def point_within_bounds(self, point=[0,0]):
        """Returns True if the given 2D or 3D point is within the dataset bounds.

        Args:
            point (list, optional): The 2D or 3D point list. Defaults to [0,0].

        Returns:
            bool: Whether the point lies within the bounds of the dataset
        """
        if point[0] < -180 or point[0] > 180 or point[1] < -74.75 or point[1] > 89.75:
            return False
        if len(point) > 2 and (point[2] < 5.03355 or point[2] > 5395.023):
            return False
        return True
            
    def query(self, point, month=0):
        """Returns a (dx, dy, dz) velocity vector for a given point (x, y, z).

        Args:
            point (tuple): A (x, y, z) point that corresponds to longitude, latitude, and depth respectively.
                The point must be constrained to the following ranges:
                    x: [-180, 180]
                    y: [-74.75, 89.75]
                    z: [5.03355, 5395.023]
            month (int, optional): The given month (0-11) to query the dataset. Defaults to 0 (January).

        Returns:
            (dx, dy, dz): A velocity vector describing the current at the given point. Found through interpolation.
        """
        point = list(point)
        
        u = self.ds['u'][month]
        v = self.ds['v'][month]
        
        # change the range to increasing values so interpolation works smoothly
        if point[0] < 0.25:
            point[0] += 360
            
        x_range = np.concatenate((self.x_range, [0.25]))
        x_range[360:] += 360
        
        # add edge case
        u = np.concatenate((u, u[:,:,0:1]), axis=2)
        v = np.concatenate((v, v[:,:,0:1]), axis=2)
        
        dx = interpn((self.z_range, self.y_range, x_range), u, point[::-1])[0]
        dy = interpn((self.z_range, self.y_range, x_range), v, point[::-1])[0]
        dz = 0
        return dx, dy, dz

    def draw_map(self, data=None, currents=True, resolution='c'):
        """Plots currents on a world map and can add waypoints. Saves the result.

        Args:
            data (list, optional): A list of tuples of longitudes, latitudes, colours, and sizes. Defaults to None.
            size (int, optional): The waypoint indicator size. Defaults to 1.
            currents (bool, optional): Flag enabling current plotting.
            resolution (char, optional): Resolution at which to plot at. Defaults to 'c' which is crude. Change to 'i' for better quality.
        """

        plt.figure(figsize=(10,10))
        m = Basemap(lat_ts=10,resolution=resolution)
        
        m.drawcoastlines()
        m.fillcontinents(color='coral',lake_color='aqua')
        m.drawparallels(np.arange(-90.,91.,30.),labels=[1,0,0,0])
        m.drawmeridians(np.arange(-180.,181.,60.),labels=[0,0,0,1],rotation=45)
        m.drawmapboundary(fill_color='aqua')
        
        if currents:
            x, y = np.meshgrid(self.x_range, self.y_range)
            m.quiver(x, y, self.ds["u"][0, 0], self.ds["v"][0, 0], latlon=True, zorder=9)
        
        for lons, lats, col, size in data:\
            m.scatter(lons, lats, marker='D', c=col, latlon=True, zorder=10, s=size)
        
        plt.title("Current Map")
        plt.savefig('current_map.png', format='png', dpi=500)
        plt.show()
        
    
    def return_bounded_area(self, bounds):
        """Returns indices that conain the ranges of longitudes and latitudes.

        Args:
            bounds (tuple): Min and max longitude and latitude values of the form (x1, x2, y1, y2).

        Returns:
            tuple: Min and max dataset indexed vales of the form (lx, hx, ly, hy). 
        """
        x1, x2, y1, y2 = bounds
        lx = np.argmax((self.x_range >= x1) & (self.x_range < x1 + self.inc)) if x1 <= 179.75 else 360
        hx = np.argmax((self.x_range <= x2) & (self.x_range > x2 - self.inc)) + 1 if x2 >= -179.75 else 360
        ly = np.argmax((self.y_range >= y1))
        hy = np.argmax((self.y_range <= y2) & (self.y_range > y2 - self.inc)) + 1
        return lx, hx, ly, hy
        
    def draw_3D_map(self, bbox=[-88, -7, 23, 64]):
        """Plots currents of a world map in 3D bounded to the given area. Saves the result.

        Args:
            bbox (list, optional): A bounding box [x1,x2,y1,y2] to which the plot is constrained. Defaults to [-88, -7, 23, 64] (gulf stream).
        """
        
        x1, x2, y1, y2 = self.return_bounded_area(bbox)

        # TODO: fix bug when x values range from negative to positive
        u = self.ds['u'][0, :, y1:y2, x1:x2]
        v = self.ds['v'][0, :, y1:y2, x1:x2]
        w = np.zeros(u.shape)
        x,y,z = np.meshgrid(self.y_range[y1:y2],self.z_range,self.x_range[x1:x2])

        fig = plt.figure(figsize=(16,14))
        ax = fig.add_subplot(projection='3d')

        bm = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[2],
                    urcrnrlon=bbox[1], urcrnrlat=bbox[3],
                    projection='cyl', resolution='c', fix_aspect=False, ax=ax)

        ax.add_collection3d(bm.drawcoastlines(linewidth=0.25))
        ax.add_collection3d(bm.drawcountries(linewidth=0.35))
        ax.view_init(azim=300, elev=50)
        ax.set_xlabel('Longitude (°E)', labelpad=20)
        ax.set_ylabel('Latitude (°N)', labelpad=20)
        ax.set_zlabel('Depth (m)', labelpad=20)

        # add meridian and parallel gridlines
        lon_step = 5
        lat_step = 5
        meridians = np.arange(bbox[0], bbox[1] + lon_step, lon_step)
        parallels = np.arange(bbox[2], bbox[3] + lat_step, lat_step)
        ax.set_yticks(parallels)
        ax.set_yticklabels(parallels)
        ax.set_xticks(meridians)
        ax.set_xticklabels(meridians)
        #ax.set_zticks(d)
        #ax.set_zticklabels(d)

        # uncomment these two lines if things get too laggy and comment the third line below
        # skip=(slice(None,None,1),slice(None,None,1))
        # ax.quiver(z[skip],x[skip],y[skip],u[skip],v[skip],w[skip], length=0.1, normalize=False)
        ax.quiver(z,x,y,u,v,w, length=0.1, normalize=False)
        ax.set_zlim(0., 150)
        plt.savefig('3d_current_map.png')
        
    def convert_to_lon_lat(self, x, y):
        """Converts a dataset x, y index to a longitude and latitude value.

        Args:
            x (int): Index to convert to longitude.
            y (int): Index to convert to latitude.

        Returns:
            tuple: Longitude, Latitude
        """
        return [self.x_range[i] for i in x], [self.y_range[j] for j in y]
        
    def make_graph(self, month=0):
        """Creates a directed graph for the dataset as a (330,720,8) array.
        Takes 2.5h to run.

        Args:
            month (int, optional): Selects the month for which to use. Defaults to 0 (January).

        Returns:
            np.array: A (330,720,8) array of values corresponding to the latitudes, longitudes, and edge weights.
        """
        start_time = perf_counter() # time in ms 1000/s
        graph = np.ones((330,720,8)) * np.inf
        for y in range(330):
            for x in range(720):
                for z in range(50):
                    u, v = self.ds["u"][month,z,y,x], self.ds["v"][month,z,y,x]
                    if not u or not v:
                        continue
                    # set the cost to the inverse of the speed of the current
                    cost = 1/np.linalg.norm([u, v])
                    
                    # discretize the angle to 8 directions
                    rad = np.arctan2(v, u) # -pi to pi
                    dir = None
                    if rad <= np.pi/8 and rad > -np.pi/8: # right octant
                        dir = 3
                    elif rad <= 3*np.pi/8 and rad > np.pi/8: # top right octant
                        dir = 2
                    elif rad <= 5*np.pi/8 and rad > 3*np.pi/8: # top octant
                        dir = 1
                    elif rad <= 7*np.pi/8 and rad > 5*np.pi/8: # top left octant
                        dir = 0
                    elif rad <= -7*np.pi/8 or rad > 7*np.pi/8: # left octant
                        dir = 7
                    elif rad <= -5*np.pi/8 and rad > -7*np.pi/8: # bottom left octant
                        dir = 6
                    elif rad <= -3*np.pi/8 and rad > -5*np.pi/8: # bottom octant
                        dir = 5
                    elif rad <= -1*np.pi/8 and rad > -3*np.pi/8: # bottom right octant
                        dir = 4
                    else:
                        return ValueError
                    
                    #update accordingly
                    if cost < graph[y, x, dir]:
                        graph[y, x, dir] = cost

            if y % 10 == 0: # for timekeeping
                print(y, "iterations finished in", perf_counter() - start_time, "ms")
                start_time = perf_counter()
        return graph


if __name__ == "__main__":
    soda = SODA()
    # soda.draw_mercator(lons=[-76.9219820, 0, 128, -128],lats=[38.9719980, 0, -30, -60])
    # soda.draw_map(list(np.arange(90)), list(np.arange(90)))
    # soda.draw_map()
    # soda.draw_3D_map(bbox=[75, 100, 5, 25])
    # print(soda.ds['u'][0,0,10,10], soda.ds['v'][0,0,10,10], 0)
    # print(soda.query([5.25, -69.75, 5.03355]))
    
    # print(soda.ds['u'][0,0,10,0], soda.ds['v'][0,0,10,0], 0)
    # print(soda.query([0.25, -69.75, 5.03355]))
    # print(soda.ds['u'][0,0,10,719], soda.ds['v'][0,0,10,719], 0)
    # print(soda.query([-0.25, -69.75, 5.03355]))
    # print(soda.query([0, -69.75, 5.03355]))
    # print(soda.query([0, 0, 0.8726646259971648]))
    # test = np.load("graph.npy", allow_pickle=True)
    # print(test.shape)
    # print(test)
    graph = soda.make_graph()
    np.save("graph", graph)
    print(graph)
    print(graph.shape)

    soda.ds.close()
