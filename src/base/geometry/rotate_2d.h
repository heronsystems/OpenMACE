#ifndef ROTATE_2D_H
#define ROTATE_2D_H

#include <cmath>
#include "../math/helper_pi.h"

namespace mace {
namespace geometry {

    // TODO-PAT: Move this method to a more meaningful location? Is there another class that should house this?

    //!
    //! \brief rotatePoint_2D Rotate a point about an origin for a given angle (+ is CCW, - is CW)
    //! \param xCoord X coordinate to rotate
    //! \param yCoord Y coordinate to rotate
    //! \param angleDeg Angle in degrees to rotate
    //! \param originX Origin point X coordinate (default 0.0)
    //! \param originY Origin point Y coordinate (default 0.0)
    //! \param newX Container for rotated x coordinate
    //! \param newY Container for rotated y coordinate
    //!
    inline void rotatePoint_2D(double &newX, double &newY, const double &xCoord, const double &yCoord,
                        const double &angleDeg, const double &originX = 0.0, const double &originY = 0.0)
    {
        double tmpX, tmpY, rotatedX, rotatedY;
        // Translate point to the origin:
        tmpX = xCoord - originX;
        tmpY = yCoord - originY;

        // Rotate point about origin:
        double thetaRad = math::convertDegreesToRadians(angleDeg);
        rotatedX = tmpX*cos(thetaRad) - tmpY*sin(thetaRad);
        rotatedY = tmpY*cos(thetaRad) + tmpX*sin(thetaRad);

        // Translate back:
        newX = rotatedX + originX;
        newY = rotatedY + originY;
    }



} //end of namepsace geometry
} //end of namespace mace

#endif // ROTATE_2D_H
