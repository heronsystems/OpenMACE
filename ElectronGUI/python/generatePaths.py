import matplotlib.pyplot as pl
import numpy as np
import scipy as sp
import scipy.spatial
import sys
from matplotlib import path
from pyproj import Proj, transform

from planPath import planPath

eps = sys.float_info.epsilon

def mercToLatLon(merc_x, merc_y):
    inProj = Proj(init='epsg:3857')
    outProj = Proj(init='epsg:4326')
    lat,lon = transform(inProj, outProj, merc_x, merc_y)
    return float(lat), float(lon)

def latLonToMerc(lat, lon):
    inProj = Proj(init='epsg:4326')
    outProj = Proj(init='epsg:3857')
    mercX, mercY = transform(inProj, outProj, lat, lon)
    return float(mercX), float(mercY)

# Function to determine minimum bounding box of the points given in coords:
def minimum_bounding_box(coords):
  min_x = 100000 # start with something much higher than expected min
  min_y = 100000
  max_x = -100000 # start with something much lower than expected max
  max_y = -100000

  for item in coords:
    if item[0] < min_x:
      min_x = item[0]

    if item[0] > max_x:
      max_x = item[0]

    if item[1] < min_y:
      min_y = item[1]

    if item[1] > max_y:
      max_y = item[1]

  return [min_x, max_x, min_y, max_y]

def in_box(hotSpots, bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= hotSpots[:, 0],
                                         hotSpots[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= hotSpots[:, 1],
                                         hotSpots[:, 1] <= bounding_box[3]))


def voronoi(hotSpots, bounding_box):
    # Select towers inside the bounding box
    i = in_box(hotSpots, bounding_box)
    # Mirror points
    points_center = hotSpots[i, :]
    points_left = np.copy(points_center)
    points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
    points_right = np.copy(points_center)
    points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
    points_down = np.copy(points_center)
    points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
    points_up = np.copy(points_center)
    points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
    points = np.append(points_center,
                       np.append(np.append(points_left,
                                           points_right,
                                           axis=0),
                                 np.append(points_down,
                                           points_up,
                                           axis=0),
                                 axis=0),
                       axis=0)
    # Compute Voronoi
    vor = sp.spatial.Voronoi(points)
    # Filter regions
    regions = []
    for region in vor.regions:
        flag = True
        for index in region:
            if index == -1:
                flag = False
                break
            else:
                x = vor.vertices[index, 0]
                y = vor.vertices[index, 1]
                if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                       bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                    flag = False
                    break
        if region != [] and flag:
            regions.append(region)
    vor.filtered_points = points_center
    vor.filtered_regions = regions
    return vor

def centroid_region(vertices):
    # Polygon's signed area
    A = 0
    # Centroid's x
    C_x = 0
    # Centroid's y
    C_y = 0
    for i in range(0, len(vertices) - 1):
        s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
        A = A + s
        C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
        C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
    A = 0.5 * A
    C_x = (1.0 / (6.0 * A)) * C_x
    C_y = (1.0 / (6.0 * A)) * C_y
    return np.array([[C_x, C_y]])


# **** Start main script **** #
argboundaryVertices = np.array([[37.886928977295476, -76.81533336639404], [37.890527558618025, -76.81533336639404], [37.890527558618025, -76.80932521820068], [37.886928977295476, -76.80932521820068], [37.886928977295476, -76.81533336639404]])
arghotSpots = np.array([[37.88865631827983, -76.8123185634613]])

boundaryVerticesArr = []
for i in xrange(0, len(argboundaryVertices)):
    boundaryMercX, boundaryMercY = latLonToMerc(float(argboundaryVertices[i][0]), float(argboundaryVertices[i][1]))
    boundaryVerticesArr.append([float(boundaryMercX), float(boundaryMercY)])
    # boundaryVerticesArr.append([merc_x(argboundaryVertices[i][0]), merc_y(argboundaryVertices[i][1])])
boundaryVertices = np.asarray(boundaryVerticesArr)

hotSpotsArr = []
for i in xrange(0, len(arghotSpots)):
    hotSpotMercX, hotSpotMercY = latLonToMerc(float(arghotSpots[i][0]), float(arghotSpots[i][1]))
    hotSpotsArr.append([float(hotSpotMercX), float(hotSpotMercY)])
    # hotSpotsArr.append([merc_x(arghotSpots[i][0]), merc_y(arghotSpots[i][1])])
hotSpots = np.asarray(hotSpotsArr)

# # Comment out to use lat/lon from above:
# hotSpots = np.array([[1, 1], [2.5, 3], [4, 4]])
# boundaryVertices = np.array([[0, 0], [0, 5], [5, 5], [5, 0], [0, 0]])


validatePointsPath = path.Path(boundaryVertices)

for i in xrange(0,len(hotSpots)):
    if validatePointsPath.contains_point(hotSpots[i]) == False:
        sys.exit("One of the selected points of interest is either on or not contained in the boundary: " + str(hotSpots[i]))

# NORMALIZE:
temp_bounding_box = minimum_bounding_box(boundaryVertices)
normFactor_x = float(temp_bounding_box[1] - temp_bounding_box[0]) #this should always be positive based on the math
normFactor_y = float(temp_bounding_box[3] - temp_bounding_box[2])

normedBoundary = []
for point in boundaryVertices:
    bnormX = float((point[0]-temp_bounding_box[0])/normFactor_x)
    bnormY = float((point[1]-temp_bounding_box[2])/normFactor_y)
    normedBoundary.append([bnormX, bnormY])
boundaryVerticesNormalized = np.array(normedBoundary)

normedHotSpots = []
for hotPoints in hotSpots:
    normX = float((hotPoints[0]-temp_bounding_box[0])/normFactor_x)
    normY = float((hotPoints[1]-temp_bounding_box[2])/normFactor_y)
    normedHotSpots.append([normX, normY])
hotSpotsNormalized = np.array(normedHotSpots)
# END NORMALIZE

print "TESTING"
print boundaryVertices
print hotSpots

print "Normalized:"
print boundaryVerticesNormalized
print hotSpotsNormalized


bounding_box = minimum_bounding_box(boundaryVerticesNormalized)
vor = voronoi(hotSpotsNormalized, bounding_box)

fig = pl.figure()
ax = fig.gca()
# Plot initial points
ax.plot(vor.filtered_points[:, 0], vor.filtered_points[:, 1], 'go', markerSize=10)
# Plot ridges points
planningRegions = []
planningHotSpots = []
for region in vor.filtered_regions:
    vertices = vor.vertices[region, :]
    ax.plot(vertices[:, 0], vertices[:, 1], 'bo')
    planningRegions.append(vertices)
    regionPath = path.Path(vertices)
    for point in hotSpotsNormalized:
        if regionPath.contains_point((point[0], point[1])):
            planningHotSpots.append(point)

print planningRegions

# Plot ridges
for region in vor.filtered_regions:
    vertices = vor.vertices[region + [region[0]], :]
    ax.plot(vertices[:, 0], vertices[:, 1], 'k-')

# Compute and plot centroids
centroids = []
for region in vor.filtered_regions:
    vertices = vor.vertices[region + [region[0]], :]
    centroid = centroid_region(vertices)
    centroids.append(list(centroid[0, :]))
    # ax.plot(centroid[:, 0], centroid[:, 1], 'c.')


# Plot boundary supplied:
ax.plot(boundaryVertices[:, 0], boundaryVertices[:, 1], 'b-')

ax.set_xlim([bounding_box[0] - 1, bounding_box[1] + 1])
ax.set_ylim([bounding_box[2] - 1, bounding_box[3] + 1])
# pl.savefig("bounded_voronoi.png")

# sp.spatial.voronoi_plot_2d(vor)
# pl.savefig("voronoi.png")


# Plan the path based on the boundaries:
vehiclePaths = []
vehiclePathsNormalized = []
pVal = 0.5
gridSpacing = 0.25
for i in xrange(0, len(planningRegions)):
#     print i
# for region in planningRegions:
    tmpWaypoints = planPath(planningRegions[i], "NorthSouth", gridSpacing, planningHotSpots[i], pVal)
    transformedPoints = []
    for point in tmpWaypoints:
        # Un-normalize first:
        newX = point[0]*normFactor_x + temp_bounding_box[0]
        newY = point[1]*normFactor_y + temp_bounding_box[2]
        # Transform to lat/lon:
        #lat,lon = mercToLatLon(newX, newY)plot
        #transformedPoints.append([lat, lon])
        transformedPoints.append([newY, newX])
    vehiclePaths.append(transformedPoints)
    vehiclePathsNormalized.append(tmpWaypoints)
    # vehiclePaths.append(tmpWaypoints)

# print vehiclePaths # THIS IS WHAT SHOULD BE PLOTTED ON LEAFLET MAP

for path in vehiclePathsNormalized:
    ax.plot(np.array(path)[:,0], np.array(path)[:,1], 'r-')

pl.title('pVal: ' + str(pVal) + " / " + "gridSpacing: " + str(gridSpacing), fontsize=20)
pl.show()
