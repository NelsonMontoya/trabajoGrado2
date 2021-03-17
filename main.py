
# import numpy as np
# import matplotlib.pyplot as plt
from cpp_algorithms.bcd import bcd
from cpp_algorithms.wavefront import wavefront
from cpp_algorithms.common_helpers import get_all_area_maps, get_random_coords, get_end_coords
from cpp_algorithms.common_helpers import plot, imshow, imshow_scatter


# ---- Get The Map ----
area_maps = get_all_area_maps("./test_maps/")   # all area maps in the folder
area_map = area_maps[0]
print(area_map)

# ---- Calculate Coverage Path ----
start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
end_point = get_end_coords(area_map, 1)[0]
print(start_point)
# start_point = (area_map(0),1)
coverage_path_bcd = bcd(area_map, start_point)      # calculate coverage path using bcd
# end_point = coverage_path_bcd[-1]
print(end_point)
print(coverage_path_bcd)

# ---- Display The Stuff ----
imshow(area_map, figsize=(200, 200), cmap="Blues_r")                # shows the area_map
plot(coverage_path_bcd, alpha=0.8, color="green")
imshow_scatter([start_point], color="black")    # show the start_point   (green)
imshow_scatter([end_point], color="red")        # show the end_point     (red)
print(end_point)
# cm = coverage_metrics(area_map, coverage_path)  # calculate coverage metrics

# ---- Calculate Coverage Path ----
# start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
print(start_point)
# start_point = (area_map(0),1)
coverage_path_wavefront = wavefront(area_map, start_point, end_point)      # calculate coverage path using wavefront
# end_point = coverage_path_wavefront[-1]
print(end_point)
print(coverage_path_wavefront)

# ---- Display The Stuff ----
imshow(area_map, figsize=(200, 200), cmap="Blues_r")                # shows the area_map
plot(coverage_path_wavefront, alpha=1.8, color="green")
imshow_scatter([start_point], color="black")    # show the start_point   (green)
imshow_scatter([end_point], color="red")        # show the end_point     (red)
print(end_point)