#include <sofa/constraintGeometry/constraint/ConstraintBilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{
    void registerConstraintBilateral(sofa::core::ObjectFactory* factory)
    {
        factory->registerObjects(
            sofa::core::ObjectRegistrationData("Implements bilateral constraints between an origin and a destination geometry")
            .add<ConstraintBilateral>());
    }
}  // namespace sofa::constraintGeometry
