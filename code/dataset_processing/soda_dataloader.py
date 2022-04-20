from dataloader import Dataloader
from netCDF4 import Dataset
import os

from mpl_toolkits.basemap import Basemap
import numpy as np
import matplotlib.pyplot as plt

class SODA(Dataloader):
    
    def __init__(self):
        filename = os.path.join(os.path.dirname(__file__), '../../datasets\soda3.12.2\soda3.12.2_mn_ocean_reg_2017.nc')
        self.ds = Dataset(filename)
        # self.ds_start_point = (74.75S, 0.25E)

        # The netcdf v4 variable ordering is: {time, depth, latitude, longitude}
        # written out surface to bottom,
        # south to north (velocity begins at 79.968°S while tracers begin at 80.021°S),
        # and west to east (velocity begins at 279.75°E while tracers begin at 279.875°E). 

        self.depth = [ 5.03355, 15.10065, 25.21935, 35.35845, 45.57635, 55.86325, 66.26175, 76.80285, 87.57695, 98.62325,
                      110.0962, 122.1067, 134.9086, 148.7466, 164.0538, 181.3125, 201.2630, 224.7773, 253.0681, 287.5508,
                      330.0078, 382.3651, 446.7263, 524.9824, 618.7031, 728.6921, 854.9935, 996.7153, 1152.376, 1319.997,
                      1497.562, 1683.057, 1874.788, 2071.252, 2271.323, 2474.043, 2678.757, 2884.898, 3092.117, 3300.086,
                      3508.633, 3717.567, 3926.813, 4136.251, 4345.864, 4555.566, 4765.369, 4975.209, 5185.111, 5395.023]


        # print(self.ds.data_model) # NETCDF3_CLASSIC
        # for attr in self.ds.ncattrs(): # ['CDI', 'Conventions', 'history', 'filename', 'title', 'grid_type', 'grid_tile', 'nco_openmp_thread_number', 'CDO']
        #     print("---- " + attr + " ----\n" + str(self.ds.getncattr(attr)))
        # print(ds.name) # /
        # print(ds.groups) # {}
        # print(ds.dimensions) # {'xt_ocean', 'yt_ocean', 'xu_ocean', 'yu_ocean', 'st_ocean', 'sw_ocean', 'time'}

        print(self.ds["u"])

        # print(self.ds.variables)
            # longitude, latitude, longitude, latitude, tcell zstar depth, ucell zstar depth, time
            # temp(time, st_ocean, yt_ocean, xt_ocean) # sea water potential temperature [C]
            # salt(time, st_ocean, yt_ocean, xt_ocean) # practical sea water salinity [psu]
            # wt(time, sw_ocean, yt_ocean, xt_ocean) # dia-surface velocity T-points [m/sec]
            # ssh(time, yt_ocean, xt_ocean) # sea surface height above geoid; effective sea level (eta_t + patm/(rho0*g)) on T cells [m]
            # mlt(time, yt_ocean, xt_ocean) # mixed layer depth determined by temperature criteria [m]
            # mlp(time, yt_ocean, xt_ocean) # depth of potential density mixed layer [m]
            # mls(time, yt_ocean, xt_ocean) # mixed layer depth determined by salinity criteria [m]
            # net_heating(time, yt_ocean, xt_ocean) # surface ocean heat flux coming through coupler and mass transfer [Watts/m^2]
            # prho(time, st_ocean, yt_ocean, xt_ocean) # potential sea water density referenced to dbar [kg/m^3]
            # u(time, st_ocean, yu_ocean, xu_ocean) # sea water x velocity [m/sec]
            # v(time, st_ocean, yu_ocean, xu_ocean) # sea water y velocity [m/sec]
            # taux(time, yu_ocean, xu_ocean) # surface downward x stress; i-directed wind stress forcing u-velocity [N/m^2]
            # tauy(time, yu_ocean, xu_ocean) # surface downward y stress; j-directed wind stress forcing v-velocity [N/m^2]
        # print(ds.disk_format) # NETCDF3
        # print(ds.path) # /
        # print(ds.parent) # None
        # print(ds.file_format) # NETCDF3_CLASSIC
        # print(ds.data_model) # NETCDF3_CLASSIC
        # print(ds.cmptypes) # {}
        # print(ds.vltypes) # {}
        # print(ds.enumtypes) # {}
        # print(ds.keepweakref) # False

    def query(self, start_point, robot_velocity, time_interval, time_resolution, current_time=0):
        pass

    def draw_mercator(self, lats=None, lons=None):
        bbox = [-75, 89.75, -180, 180]

        plt.figure(figsize=(10,10))
        # m = Basemap(projection='merc',llcrnrlat=bbox[0],urcrnrlat=bbox[1],\
        #             llcrnrlon=bbox[2],urcrnrlon=bbox[3],lat_ts=10,resolution='c') # change to i
        m = Basemap(lat_ts=10,resolution='c')
        
        m.drawcoastlines()
        m.fillcontinents(color='coral',lake_color='aqua')
        # m.drawparallels(np.arange(bbox[0],bbox[1],(bbox[1]-bbox[0])/5),labels=[1,0,0,0])
        # m.drawmeridians(np.arange(bbox[2],bbox[3],(bbox[3]-bbox[2])/5),labels=[0,0,0,1],rotation=45)
        m.drawparallels(np.arange(-90.,91.,30.),labels=[1,0,0,0])
        m.drawmeridians(np.arange(-180.,181.,60.),labels=[0,0,0,1],rotation=45)
        
        m.drawmapboundary(fill_color='aqua')
        

        if lons and lats:
            m.scatter(lats, lons, marker='D', color='r', latlon=True, zorder=10)
        else:
            start = (0.25, -74.75) # 74.75S, 0.25E
            rx = np.concatenate((np.arange(start[0], 180, 0.5), np.arange(-180 + start[0]%1, start[0], 0.5)))
            ry = np.arange(start[1], 90, 0.5)
            x = np.tile(rx, (330, 1))
            y = np.tile(ry, (720, 1)).T
            print(x.shape, y.shape, self.ds["u"][0, 0].shape, self.ds["v"][0, 0].shape)
            m.quiver(x, y, self.ds["u"][0, 0], self.ds["v"][0, 0], latlon=True, zorder=10)
            
        
        plt.title("Mercator Projection")
        plt.savefig('coordinate_test.png', format='png', dpi=500)
        plt.show()


if __name__ == "__main__":
    soda = SODA()
    # soda.draw_mercator(lats=[-76.9219820, 0, 128, -128],lons=[38.9719980, 0, -30, -60])
    soda.draw_mercator()
    soda.ds.close()
