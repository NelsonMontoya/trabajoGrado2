import utm
import numpy as np
import matplotlib.pyplot as plt
from cpp_algorithms.bcd import bcd
from cpp_algorithms.wavefront import wavefront
from cpp_algorithms.stc import stc
from cpp_algorithms.common_helpers import get_all_area_maps, get_random_coords, get_end_coords
from cpp_algorithms.metrics import coverage_metrics,printer
from cpp_algorithms.common_helpers import plot, imshow, imshow_scatter
from classes.builGrid import Grid


do_animation = True


def visualize_path(grid_map, start, goal, path):  # pragma: no cover
    ox, oy = start
    gx, gy = goal
    cp = np.array(path)
    px, py = cp.T
   # px, py = np.transpose(np.flipud(np.fliplr(path)))

    if not do_animation:
        plt.imshow(grid_map, cmap='viridis')
        plt.plot(ox, oy, "-xy")
        plt.plot(px, py, "-r")
        plt.plot(gx, gy, "-pg")
        plt.show()
    else:
        for ipx, ipy in zip(py, px):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.imshow(grid_map, cmap='viridis')
            plt.plot(oy, ox, "-xb")
            plt.plot(py, px, "-r")
            """print(ipx, ipy)"""
            plt.plot(gy, gx, "-pg")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(1e-16)


def recalculatePath(coveragePath):
    cp = np.array(coveragePath)
    px, py = cp.T
    coverage_path_wavefront = []
    for i in range(len(px)):
        coverage_path_wavefront.append((px[i], py[i]))
    return coverage_path_wavefront



# get the gps coordinates

gps0 = (10.444601, -75.391864)
gps1 = (10.444648, -75.390153)
gps2 = (10.443347, -75.391815)
gps3 = (10.443399, -75.390110)
takeoffPoint = (10.445997, -75.391791)

step = 1
name = str(step)

grid = Grid(gps0, gps1, gps2, gps3, takeoffPoint, step)
# grid.validatePixelstoMeters()
# ---- Get The Map ----
area_maps = get_all_area_maps("./test_maps/")   # all area maps in the folder
area_map = area_maps[0]
size_area = area_map.shape
print(size_area)

grid.buildGrid(area_map.shape[0], area_map.shape[1])

# ---- Calculate Coverage Path ----
start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
end_point = get_end_coords(area_map, 1)[0]
print(start_point)
# start_point = (area_map(0),1)
coverage_path_bcd = bcd(area_map, start_point)      # calculate coverage path using bcd
# coverage_path_bcd = recalculatePath(coverage_path)
cm = coverage_metrics(area_map, coverage_path_bcd)
print('Métricas Calculadas BCD')
# printer(cm)


grid.assignDataGrid(coverage_path_bcd, 'BCD_Route')
grid.calculateStatistics()
print('Longitud Ruta :', grid.distanceOfFly, 'mts')
print('Tiempo de Vuelo :', grid.timeOfFly, 'seconds')

# grid.generatePathGPS()
# grid.generateRoute('BCD_Route.csv')
# end_point = coverage_path_bcd[-1]
# print(end_point)
# print(coverage_path_bcd)





# ---- Display The Stuff ----

#imshow(area_map, figsize=(200, 200), cmap="Blues_r")                # shows the area_map
# plot(coverage_path_bcd, alpha=0.8, color="green")
# imshow_scatter([start_point], color="black")    # show the start_point   (green)
# imshow_scatter([end_point], color="red")        # show the end_point     (red)
# print(end_point)
# cm = coverage_metrics(area_map, coverage_path)  # calculate coverage metrics
# print(coverage_path_bcd)
# visualize_path(area_map, start_point, end_point, coverage_path_bcd)

# ---- Calculate Coverage Path ----
# start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
# print(start_point)
# start_point = (area_map(0),1)
coverage_path = wavefront(area_map, start_point, end_point)      # calculate coverage path using wavefront
coverage_path_wavefront = recalculatePath(coverage_path)
# cm = coverage_metrics(area_map, coverage_path)
grid.assignDataGrid(coverage_path_wavefront, 'WAVE_Route')
grid.calculateStatistics()
print('Métricas Calculadas Wavefront')
print('Longitud Ruta :', grid.distanceOfFly, 'mts')
print('Tiempo de Vuelo :', grid.timeOfFly, 'seconds')
# grid.generatePathGPS()
# grid.generateRoute('Wavefront_Route.csv')

# print('Métricas Calculadas Wavefront')
# printer(cm)
# visualize_path(area_map, start_point, end_point, coverage_path_wavefront)
# ---- Display The Stuff ----
# imshow(area_map, figsize=(200, 200), cmap="Blues_r")                # shows the area_map
# plot(coverage_path_wavefront, alpha=1.8, color="green")
# imshow_scatter([start_point], color="black")    # show the start_point   (green)
# imshow_scatter([end_point], color="red")        # show the end_point     (red)
# print(end_point)

#    stc CPP

coverage_path_stc = stc(area_map, start_point)
# cm = coverage_metrics(area_map, coverage_path_stc)
grid.assignDataGrid(coverage_path_stc, 'STC_Route')
grid.calculateStatistics()
print('Métricas Calculadas STC')
print('Longitud Ruta :', grid.distanceOfFly, 'mts')
print('Tiempo de Vuelo :', grid.timeOfFly, 'seconds')
# grid.generatePathGPS()
# grid.generateRoute('STC_Route.csv')
# print('Métricas Calculadas STC')
# printer(cm)
# visualize_path(area_map, start_point, end_point, coverage_path_stc)
# print(end_point)
