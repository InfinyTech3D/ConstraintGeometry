#include <sofa/constraintGeometry/constraint/ConstraintInsertion.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{
    void registerConstraintInsertion(sofa::core::ObjectFactory* factory)
    {
        factory->registerObjects(
            sofa::core::ObjectRegistrationData("Implements frictional and bilateral constraints to model needle insertion")
            .add<ConstraintInsertion>());
    }
}  // namespace sofa::constraintGeometry
