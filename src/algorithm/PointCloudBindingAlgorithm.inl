//#ifndef SOFA_COMPONENT_CONSTRAINT_BINDPOINTNALGORITHM_INL
//#define SOFA_COMPONENT_CONSTRAINT_BINDPOINTNALGORITHM_INL

//#include "PointCloudBindingAlgorithm.h"
//#include "ConstraintProximity.h"
//#include <sofa/defaulttype/SolidTypes.h>
//#include <sofa/core/behavior/BaseController.h>
//#include <sofa/core/behavior/MechanicalState.h>
//#include <math.h>
//#include <sofa/defaulttype/Vec.h>

//namespace sofa {

//namespace core {

//namespace behavior {

//PointCloudBindingAlgorithm::PointCloudBindingAlgorithm()
//: d_maxDist(initData(&d_maxDist, "maxDist", "Collision detection algorithm")) {}


//PariProximityVector PointCloudBindingAlgorithm::processAlgorithm(BaseGeometry * from,BaseGeometry * dest) {
//    PariProximityVector res;

//    PointGeometry * m_from = dynamic_cast<PointGeometry *>(from);
//    PointGeometry * m_dest = dynamic_cast<PointGeometry *>(dest);

//    if (m_from == NULL) return res;
//    if (m_dest == NULL) return res;

//    if (m_from->getNbPoints() == 0) return res;
//    if (m_dest->getNbPoints() == 0) return res;

//    helper::vector<defaulttype::Vector3> p1;
//    helper::vector<defaulttype::Vector3> p2;

//    for (int i=0;i<m_from->getNbPoints();i++) p1.push_back(m_from->getElement(i)->getDefaultProximity()->getPosition());
//    for (int i=0;i<m_dest->getNbPoints();i++) p1.push_back(m_dest->getElement(i)->getDefaultProximity()->getPosition());

//    helper::vector<int> bindId;
//    bindId.resize(p1.size(),-1);

//    bool change = true;

//    helper::vector<int> invBind;
//    invBind.resize(p2.size(),-1);

//    while (change) {
////        printf("NEW LOOP\n");
////        std::cout << "BIND=" << bindId << std::endl;
////        std::cout << "IBIND=" << invBind << std::endl;
//        change = false;//ne new change

//        for (unsigned p=0;p<p1.size();p++) {
//            if (bindId[p] != -1) continue;

//            defaulttype::Vector3 P = p1[p];
//            int closestId = -1;
//            double closestDist = std::numeric_limits<double>::max();

//            //Find minimal distance
//            for (unsigned i=0;i<p2.size();i++) {
//                defaulttype::Vector3 Q = p2[i];
//                double dist = (Q-P).norm();

//                if (dist < closestDist && invBind[i] == -1) {
//                    closestId = i;
//                    closestDist = dist;
////                    printf("TMP ASSO p<->i %d %d\n",p,i);
//                }
//            }

////            printf("MIN DIST %f    p=%f    %d\n",closestDist,minDist,closestId);

//            if ((d_maxDist.getValue()!=0) && (closestDist > d_maxDist.getValue())) closestId = -1;

////            printf("CLOSEST ID %d\n",closestId);

//            bindId[p] = closestId;
//        }

//        //try to remove point that are binded multiple times to the same one
//        for (unsigned i=0;i<p1.size();i++) {
//            if (bindId[i] == -1) continue;

//            int & B = bindId[i]; //current point in p1
//            int & C = invBind[bindId[i]]; // previous binded point in p2

//            if (C == -1) { // the point has not yet been associated
//                C = i;
//            } else if (C != (int) i) { // the point is already associated
//                int & A = bindId[C]; // previous binded point in p1

//                change = true; // we retry the binding because two points are associated with the same point

//                double d1 = (p1[A] - p2[C]).norm();
//                double d2 = (p1[B] - p2[C]).norm();

//                if (d2<d1) {
//                    A = -1; // invalidate A
//                    C = i; // change the invbinding
//                } else {
//                    B = -1; // invalidate A
//                }
//            }
//        }
//    }


//    for (unsigned i=0;i<bindId.size();i++) {
//        if (bindId[i] == -1) continue;

//        PariProximity pair;
//        pair.first = m_from->getElement(bindId[i])->getDefaultProximity();
//        pair.second = m_dest->getElement(invBind[i])->getDefaultProximity();

//        res.push_back(pair);
//    }

//    return res;
//}

//} // namespace controller

//} // namespace component

//} // namespace sofa

//#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
