#include <sofa/constraintGeometry/constraint/ContactReponse.h>
#include <sofa/core/ObjectFactory.h>
#include <stdio.h>
#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <qopengl.h>

namespace sofa {

namespace constraintGeometry {

int ContactReponseClass = core::RegisterObject("ContactReponse")
.add<ContactReponse>();

}

}
