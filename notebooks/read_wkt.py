import matplotlib.pyplot as plt
import shapely 

geometry_file = "BUW.wkt"
with open(geometry_file) as f:
    geometry_str = f.readline()

area = shapely.from_wkt(geometry_str)
for geo in area.geoms:
    plt.plot(*geo.exterior.xy)
    for interior in geo.interiors:
        plt.plot(*interior.xy, "-k")

plt.show()