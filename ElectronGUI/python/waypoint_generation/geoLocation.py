import math


class Location:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def updateLatitude(self, latitude):
        self.latitude = latitude

    def updateLongitude(self, longitude):
        self.longitude = longitude

    def latitudeRadians(self):
        return self.convertDegreesToRadians(self.latitude)

    def longitudeRadians(self):
        return self.convertDegreesToRadians(self.longitude)

    def latitude(self):
        return self.latitude

    def longitude(self):
        return self.longitude

    def altitude(self):
        return self.altitude

    def convertRadiansToDegrees(self, radianValue):
        return(radianValue * float(180.0/math.pi))

    def convertDegreesToRadians(self, degreeValue):
        return(degreeValue * float(math.pi/180.0))

    def distanceBetween(self, pointB):
        radiusEarth = 6371000
        b = Location(pointB.latitude, pointB.longitude)

        deltaLat = float(self.convertDegreesToRadians(b.latitude - self.latitude))
        deltaLng = float(self.convertDegreesToRadians(b.longitude - self.longitude))

        a = math.sin(deltaLat/2) * math.sin(deltaLat/2) + math.cos(self.latitudeRadians()) * math.cos(b.latitudeRadians()) * math.sin(deltaLng/2) * math.sin(deltaLng/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = radiusEarth * c
        return distance

    def bearingBetween(self, pointB):
        b = Location(pointB.latitude, pointB.longitude)
        y = float(math.sin(b.longitudeRadians() - self.longitudeRadians()) * math.cos(b.latitudeRadians()))
        x = float(math.cos(self.latitudeRadians()) * math.sin(b.latitudeRadians()) - math.sin(self.latitudeRadians()) * math.cos(b.latitudeRadians()) * math.cos(b.longitudeRadians() - self.longitudeRadians()))
        bearing = self.convertRadiansToDegrees(math.atan2(y, x))
        return bearing

    def getXYFromPoint(self, pointB):
        distance = self.distanceBetween(pointB)
        bearing = self.bearingBetween(pointB)
        # This should be fine for now, since we are always going to be in the first quadrant
        xComp = distance * math.sin(self.convertDegreesToRadians(bearing))
        yComp = distance * math.cos(self.convertDegreesToRadians(bearing))

        return xComp, yComp

    def getDistanceBearingFromXY(self, pointB):
        # This should be fine for now, since we are always going to be in the first quadrant
        distance = math.sqrt(pointB[0]**2 + pointB[1]**2)
        if distance > 0:
            bearing = math.acos(pointB[1]/distance)
        else:
            bearing = 0

        return distance, self.convertRadiansToDegrees(bearing)

    def getLatLonFromDistanceBearing(self, distance, bearing):
        radiusEarth = 6371000  # Radius of the Earth
        # convert Latitude and Longitude
        # into radians for calculation
        latitude = math.radians(self.latitude)
        longitude = math.radians(self.longitude)

        # calculate next latitude
        next_latitude = math.asin(math.sin(latitude) *
                                  math.cos(distance / radiusEarth) +
                                  math.cos(latitude) *
                                  math.sin(distance / radiusEarth) *
                                  math.cos(math.radians(bearing)))

        # calculate next longitude
        next_longitude = longitude + (math.atan2(math.sin(math.radians(bearing)) *
                                                 math.sin(distance / radiusEarth) *
                                                 math.cos(latitude),
                                                 math.cos(distance / radiusEarth) -
                                                 math.sin(latitude) *
                                                 math.sin(next_latitude)
                                                 )
                                      )

        # convert points into decimal degrees
        new_lat = math.degrees(next_latitude)
        new_lon = math.degrees(next_longitude)

        return new_lat, new_lon

    def compareDatum(self, pointB):
        if(self.latitude <= pointB.latitude):
            if(self.longitude <= pointB.longitude):
                return True
        return False
