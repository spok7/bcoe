from posixpath import lexists
from dataloader import Dataloader
from netCDF4 import Dataset
import os

from mpl_toolkits.basemap import Basemap
import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import interpn

class SODA(Dataloader):
    
    def __init__(self):
        filename = os.path.join(os.path.dirname(__file__), '../../datasets\soda3.12.2\soda3.12.2_mn_ocean_reg_2017.nc')
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

    def query(self, point, month=0):
        
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

    def draw_map(self, lats=None, lons=None):

        plt.figure(figsize=(10,10))
        m = Basemap(lat_ts=10,resolution='c') # change to i
        # mercator stuff below if we end up needing it
        # bbox = [-75, 89.75, -180, 180]
        # m = Basemap(projection='merc',llcrnrlat=bbox[0],urcrnrlat=bbox[1],\
        #             llcrnrlon=bbox[2],urcrnrlon=bbox[3],lat_ts=10,resolution='c') # change to i
        
        m.drawcoastlines()
        m.fillcontinents(color='coral',lake_color='aqua')
        # m.drawparallels(np.arange(bbox[0],bbox[1],(bbox[1]-bbox[0])/5),labels=[1,0,0,0])
        # m.drawmeridians(np.arange(bbox[2],bbox[3],(bbox[3]-bbox[2])/5),labels=[0,0,0,1],rotation=45)
        m.drawparallels(np.arange(-90.,91.,30.),labels=[1,0,0,0])
        m.drawmeridians(np.arange(-180.,181.,60.),labels=[0,0,0,1],rotation=45)
        m.drawmapboundary(fill_color='aqua')
        

        if lats and lons:
            m.scatter(lons, lats, marker='D', color='r', latlon=True, zorder=10)
        else:
            x, y = np.meshgrid(self.x_range, self.y_range)
            # print(x.shape, y.shape, self.ds["u"][0, 0].shape, self.ds["v"][0, 0].shape)
            m.quiver(x, y, self.ds["u"][0, 0], self.ds["v"][0, 0], latlon=True, zorder=10)
        
        plt.title("Current Map")
        plt.savefig('current_map.png', format='png', dpi=500)
        plt.show()
        
    
    def return_bounded_area(self, bounds):
        x1, x2, y1, y2 = bounds
        lx = np.argmax((self.x_range >= x1) & (self.x_range < x1 + self.inc)) if x1 <= 179.75 else 360
        hx = np.argmax((self.x_range <= x2) & (self.x_range > x2 - self.inc)) + 1 if x2 >= -179.75 else 360
        ly = np.argmax((self.y_range >= y1))
        hy = np.argmax((self.y_range <= y2) & (self.y_range > y2 - self.inc)) + 1
        return lx, hx, ly, hy
        
    def draw_3D_map(self, bbox=[-88, -7, 23, 64]):
        
        # extent = [75, 100, 5, 25]
        x1, x2, y1, y2 = self.return_bounded_area(bbox)

        u = self.ds['u'][0, :, y1:y2, x1:x2]
        v = self.ds['v'][0, :, y1:y2, x1:x2]
        w = np.zeros(u.shape)
        x,y,z = np.meshgrid(self.y_range[y1:y2],self.z_range,self.x_range[x1:x2])

        #Create a 3d normal figure
        fig = plt.figure(figsize=(16,14))
        ax = fig.add_subplot(projection='3d')

        #Draw the earth map using Basemap
        # Define lower left, upper right longitude and latitude respectively
        # Create a basemap instance that draws the Earth layer
        bm = Basemap(llcrnrlon=bbox[0], llcrnrlat=bbox[2],
                    urcrnrlon=bbox[1], urcrnrlat=bbox[3],
                    projection='cyl', resolution='c', fix_aspect=False, ax=ax)

        # Add Basemap to the figure
        ax.add_collection3d(bm.drawcoastlines(linewidth=0.25))
        ax.add_collection3d(bm.drawcountries(linewidth=0.35))
        ax.view_init(azim=300, elev=50)
        ax.set_xlabel('Longitude (°E)', labelpad=20)
        ax.set_ylabel('Latitude (°N)', labelpad=20)
        ax.set_zlabel('Depth (m)', labelpad=20)

        # Add meridian and parallel gridlines
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

        skip=(slice(None,None,1),slice(None,None,1))
        ax.quiver(z[skip],x[skip],y[skip],u[skip],v[skip],w[skip], length=0.1, normalize=False)
        ax.set_zlim(0., 150)
        plt.savefig('3dplot.png')


if __name__ == "__main__":
    soda = SODA()
    # soda.draw_mercator(lons=[-76.9219820, 0, 128, -128],lats=[38.9719980, 0, -30, -60])
    # soda.draw_map()
    # soda.draw_3D_map()
    print(soda.ds['u'][0,0,10,10], soda.ds['v'][0,0,10,10], 0)
    print(soda.query([5.25, -69.75, 5.03355]))
    
    print(soda.ds['u'][0,0,10,0], soda.ds['v'][0,0,10,0], 0)
    print(soda.query([0.25, -69.75, 5.03355]))
    print(soda.ds['u'][0,0,10,719], soda.ds['v'][0,0,10,719], 0)
    print(soda.query([-0.25, -69.75, 5.03355]))
    print(soda.query([0, -69.75, 5.03355]))
    
    
    soda.ds.close()
