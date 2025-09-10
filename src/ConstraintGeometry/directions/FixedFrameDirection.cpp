#include <ConstraintGeometry/directions/FixedFrameDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{
    void registerFixedFrameDirection(sofa::core::ObjectFactory* factory)
    {
        factory->registerObjects(
            sofa::core::ObjectRegistrationData("Computes a fixed direction to implement constraints")
            .add<FixedFrameDirection>());
    }
}  // namespace sofa::constraintGeometry
