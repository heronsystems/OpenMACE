#ifndef OCTOMAP_2D_PROJECTION_DEFINITION_H
#define OCTOMAP_2D_PROJECTION_DEFINITION_H

namespace mace{
namespace maps{

class Octomap2DProjectionDefinition
{
public:

    Octomap2DProjectionDefinition() :
        independentMapResolution(false),
        enabledCostMapLayer(false)
    {

    }

    Octomap2DProjectionDefinition(const Octomap2DProjectionDefinition &copy)
    {
        this->independentMapResolution = copy.independentMapResolution;
    }

public:
    bool isMapLayerResolutionIndependent() const
    {
        return independentMapResolution;
    }

    bool isCostMapLayerEnabled() const
    {
        return enabledCostMapLayer;
    }

public:
    Octomap2DProjectionDefinition& operator = (const Octomap2DProjectionDefinition &rhs)
    {
        this->independentMapResolution = rhs.independentMapResolution;
        return *this;
    }

    bool operator == (const Octomap2DProjectionDefinition &rhs) const
    {
        if(this->independentMapResolution != rhs.independentMapResolution){
            return false;
        }
        return true;
    }

    bool operator != (const Octomap2DProjectionDefinition &rhs) const{
        return !(*this == rhs);
    }

private:
    bool independentMapSize = true;
    bool independentMapResolution = false;
    bool enabledCostMapLayer = false;
};

} //end of namespace maps
} //end of namespace mace
#endif // OCTOMAP_2D_PROJECTION_DEFINITION_H
