import numpy as np
import csv
import cmath
from matplotlib import path
import scipy.spatial.distance
import matplotlib.pyplot as plt
from minimumBoundingBox import minimum_bounding_box


def planPath(boundaryPoints, pathDirection, gridSpacing, originPoint, pVal):

    ingressPt = 1
    # pathDirection = "NorthSouth"
    # p = 0.6
    p = pVal
    # gridSpacing = 0.5
    # const = 20
    m = 10 ** (- p + 1)
    # m = 0.1
    xOrigin = originPoint[0]
    yOrigin = originPoint[1]

    # Close boundary path
    xBoundary = []
    yBoundary = []
    # for i in xrange(0, len(boundaryPoints)-1):
    for point in boundaryPoints:
        xBoundary.append(point[0])
        yBoundary.append(point[1])
    xBoundary.append(xBoundary[0])
    yBoundary.append(yBoundary[0])

    # Path for WP validation:
    originalBoundaryPath = path.Path(boundaryPoints)

    # Boundary points
    xvals = []
    yvals = []
    for i in range(1, len(xBoundary)-1):
        xvals.extend(np.linspace(xBoundary[i-1], xBoundary[i], 10*gridSpacing))
        yvals.extend(np.linspace(yBoundary[i-1], yBoundary[i], 10*gridSpacing))
    xvals.extend(np.linspace(xvals[len(xvals)-1], xvals[0], 10*gridSpacing))
    yvals.extend(np.linspace(yvals[len(yvals)-1], yvals[0], 10*gridSpacing))


    # Z-plane (complex plane)
    Z = []
    for i in xrange(0,len(xvals)-1):
        tmpZ = (xvals[i] + yvals[i]*1j)
        Z.append(tmpZ)

    # R = m|r|^p
    tempOrigin = xOrigin + yOrigin*1j
    magR = []
    angR = []
    realR = []
    imagR = []
    R = []
    for i in xrange(0,len(Z)-1):
        magR.append(m * np.absolute(Z[i] - tempOrigin) ** p)
        angR.append(np.angle(Z[i] - tempOrigin))
        realR.append(magR[i] * cmath.cos(angR[i]))
        imagR.append(magR[i] * cmath.sin(angR[i]))
        R.append(realR[i] + imagR[i]*1j)
    realR.append(magR[0] * cmath.cos(angR[0]))
    imagR.append(magR[0] * cmath.sin(angR[0]))

    transformedBoundaryPoints = []
    for i in xrange(0,len(realR)-1):
        transformedBoundaryPoints.append((realR[i].real, imagR[i].real))
    transformedBoundaryPoints.append((realR[0].real, imagR[0].real))
    transformedBoundaryPath = path.Path(transformedBoundaryPoints)


    # Create smallest rectangle encompassing points:
    rect = minimum_bounding_box(realR, imagR) # returns [xmin xmax ymin ymax]
    minGridVal = rect[0].real
    maxGridVal = rect[1].real
    if rect[2].real <= rect[0].real:
        minGridVal = rect[2].real
    if rect[3].real >= rect[1].real:
        maxGridVal = rect[3].real

    meshX = np.linspace(rect[0], rect[1], 50*gridSpacing)  # (maxGridVal - minGridVal)/gridSpacing))
    meshY = np.linspace(rect[2], rect[3], 50*gridSpacing)  # (maxGridVal - minGridVal)/gridSpacing))

    # Create meshgrid in bounding box:
    if pathDirection == "NorthSouth":
        gridY,gridX = np.meshgrid(meshY.real, meshX.real)
    elif pathDirection == "EastWest":
        gridX,gridY = np.meshgrid(meshX.real, meshY.real)

    # Find which points in meshgrid are in the polygon defined by boundary{oints:
    XY = np.dstack((gridX, gridY))
    XY_flat = XY.reshape((-1, 2))
    inOut_flat = transformedBoundaryPath.contains_points(XY_flat)
    inOut = inOut_flat.reshape(gridX.shape)

    # print "IN vs. OUT:"
    # print inOut
    xInGridPts = [i[0] for i in XY_flat[inOut_flat]]
    yInGridPts = [i[1] for i in XY_flat[inOut_flat]]
    Grid = []
    for i in xrange(0,len(xInGridPts)-1):
        Grid.append(xInGridPts[i] + yInGridPts[i]*1j)


    # Lawnmower pattern:
    x_lawn = [xInGridPts[0]]
    y_lawn = [yInGridPts[0]]
    for i in xrange(1,len(xInGridPts)-1):
        if pathDirection == "NorthSouth":
            if xInGridPts[i-1] != xInGridPts[i]:
                x_lawn.append(xInGridPts[i-1])
                x_lawn.append(xInGridPts[i])
                y_lawn.append(yInGridPts[i-1])
                y_lawn.append(yInGridPts[i])
        elif pathDirection == "EastWest":
            if yInGridPts[i-1] != yInGridPts[i]:
                x_lawn.append(xInGridPts[i-1])
                x_lawn.append(xInGridPts[i])
                y_lawn.append(yInGridPts[i-1])
                y_lawn.append(yInGridPts[i])


    for i in xrange(1,len(x_lawn)-1,4):
        tempFirstXValue = x_lawn[i-1];
        tempSecondXValue = x_lawn[i];
        tempFirstYValue = y_lawn[i-1];
        tempSecondYValue = y_lawn[i];
        x_lawn[i-1] = tempSecondXValue;
        x_lawn[i] = tempFirstXValue;
        y_lawn[i-1] = tempSecondYValue;
        y_lawn[i] = tempFirstYValue;

    x_lawnExtended = []
    y_lawnExtended = []
    for i in range(1, len(x_lawn)-1):
        x_lawnExtended.extend(np.linspace(x_lawn[i-1].real, x_lawn[i].real, 50 * gridSpacing))  # (maxGridVal - minGridVal)/gridSpacing))
        y_lawnExtended.extend(np.linspace(y_lawn[i-1].real, y_lawn[i].real, 50 * gridSpacing))  # (maxGridVal - minGridVal)/gridSpacing))

    Lawn = []
    for i in range(0, len(x_lawnExtended)):
        Lawn.append(x_lawnExtended[i].real + y_lawnExtended[i].real*1j)

    # Boundary Back to Z-plane
    magZ = []
    angZ = []
    realZ = []
    imagZ = []
    for i in xrange(0,len(Z)-1):
        magZ.append(((1/m) * np.absolute(R[i])) ** (1/p))
        angZ.append(np.angle(R[i]))
        realZ.append(magZ[i] * cmath.cos(angZ[i]) + xOrigin)
        imagZ.append(magZ[i] * cmath.sin(angZ[i]) + yOrigin)
    realZ.append(magZ[0] * cmath.cos(angZ[0]))
    imagZ.append(magZ[0] * cmath.sin(angZ[0]))

    # Convert grid back to real plane
    magGridZ = []
    angGridZ = []
    realGridZ = []
    imagGridZ = []
    for i in xrange(0,len(Grid)-1):
        magGridZ.append(((1/m) * np.absolute(Grid[i])) ** (1/p))
        angGridZ.append(np.angle(Grid[i]))
        realGridZ.append(magGridZ[i] * cmath.cos(angGridZ[i]) + xOrigin)
        imagGridZ.append(magGridZ[i] * cmath.sin(angGridZ[i]) + yOrigin)

    # Convert lawnmower back to real plane
    magLawnZ = []
    angLawnZ = []
    realLawnZ = []
    imagLawnZ = []
    for i in xrange(0,len(Lawn)-1):
        # dist = np.sqrt((Lawn[i].real - tempOrigin.real)**2 + (Lawn[i].imag - tempOrigin.imag)**2)

        dist = np.absolute(Lawn[i]) - np.absolute(tempOrigin)
        # dist = scipy.spatial.distance.euclidean(Lawn[i], tempOrigin)
        # assert np.isclose(distpat, dist)

        # if (p + dist < 1) and (p + dist > 0.7):
        #     magLawnZ.append(((1 / m) * np.absolute(Lawn[i])) ** (1 / (p + dist)))
        # elif p + dist <= 0.7:
        #     magLawnZ.append(((1 / m) * np.absolute(Lawn[i])) ** (1 / 0.7))
        # else:
        #     magLawnZ.append(((1 / m) * np.absolute(Lawn[i])))
        magLawnZ.append(((1 / (m)) * np.absolute(Lawn[i])) ** (1 / (p)))

        angLawnZ.append(np.angle(Lawn[i]))
        realLawnZ.append(magLawnZ[i].real * cmath.cos(angLawnZ[i].real) + xOrigin)
        imagLawnZ.append(magLawnZ[i].real * cmath.sin(angLawnZ[i].real) + yOrigin)

    waypoints = []
    for i in xrange(0,len(realLawnZ)-1):
        if(originalBoundaryPath.contains_point([realLawnZ[i].real, imagLawnZ[i].real])):
            waypoints.append([realLawnZ[i].real, imagLawnZ[i].real])

    return waypoints

    # # Output vertices to file:
    # xCoordinates = copy_(realLawn)
    # yCoordinates = copy_(imagLawn)
    # waypoints = [xCoordinates.T, yCoordinates.T]
    # filename = char('waypoints.csv')
    # csvwrite(filename, waypoints)

    # ANIMATED PLOT FOR TESTING:
    # plt.axis([-15, 25, -15, 15])
    # plt.ion()
    # for i in xrange(0, len(xvals)-1):
    #     plt.scatter(xvals[i], yvals[i])
    #     plt.pause(0.05)
    #
    # while True:
    #     plt.pause(0.05)
    # END ANIMATED PLOT FOR TESTING:
