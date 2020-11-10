#ifndef ACTION_SET_SURFACE_DEFLECTION_H
#define ACTION_SET_SURFACE_DEFLECTION_H


class Action_SetSurfaceDeflection
{
public:
    struct Deflection
    {
        double roll;
        double pitch;
        double yaw;
        double throttle;
    };

public:
    Action_SetSurfaceDeflection() = default;

public:
    Deflection _surfaceDeflection;
};

#endif // ACTION_SETSURFACEDEFLECTION_H
