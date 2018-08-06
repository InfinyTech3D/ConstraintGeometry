#include <forcefield/CollisionForceField.h>
#include <stdio.h>
#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <qopengl.h>

namespace constraintGeometry {

CollisionForceField::CollisionForceField()
: d_stiffness("stiffness",(double) 40.0,this)
, p_collision("collision",_REQUIRED, this)
{}

void CollisionForceField::addForce(VecDerivId f) {
    collisionAlgorithm::PairProximityVector & collision = p_collision->getCollisionPairs();

    for (unsigned i=0;i<collision.size();i++) {
        Vector3 P = collision[i].first->getPosition();
        Vector3 Q = collision[i].second->getPosition();
        Vector3 PQ = P-Q;

        if (dot(PQ,collision[i].second->getNormal())<0) {
            WriteAccessor<Vector3> force_1 = collision[i].first->getState()->write(f);
            std::map<unsigned,double> line_1 = collision[i].first->getContributions();

            for (std::map<unsigned,double>::iterator it = line_1.begin();it!=line_1.end();it++) {
                force_1[it->first] -=  PQ * it->second * d_stiffness.getValue();
            }

            WriteAccessor<Vector3> force_2 = collision[i].second->getState()->write(f);
            std::map<unsigned,double> line_2 = collision[i].second->getContributions();

            for (std::map<unsigned,double>::iterator it = line_2.begin();it!=line_2.end();it++) {
                force_2[it->first] +=  PQ * it->second * d_stiffness.getValue();
            }
        }
    }
}

void CollisionForceField::addToMatrix(BaseMatrix * /*M*/) {
//    if (d_indice1.getValue() == -1) return;
//    if (d_indice2.getValue() == -1) return;

//    MatrixOffset off1 = M->getOffset(p_state1.get());
//    MatrixOffset off2 = M->getOffset(p_state2.get());

//    unsigned i1 = d_indice1.getValue()*Vector3::size();
//    unsigned i2 = d_indice2.getValue()*Vector3::size();
//    double val = d_stiffness.getValue() * M->getKFactor();

//    M->add(i1+0,i2+0,off1,off2, val);
//    M->add(i1+1,i2+1,off1,off2, val);
//    M->add(i1+2,i2+2,off1,off2, val);

//    M->add(i2+0,i1+0,off2,off1, val);
//    M->add(i2+1,i1+1,off2,off1, val);
//    M->add(i2+2,i1+2,off2,off1, val);

//    M->add(d_indice1.getValue()*Vector3::size()+0,d_indice2.getValue()*Vector3::size()+0,off2,off1,-m_force[0]);
//    M->add(d_indice1.getValue()*Vector3::size()+1,d_indice2.getValue()*Vector3::size()+1,off2,off1,-m_force[1]);
//    M->add(d_indice1.getValue()*Vector3::size()+2,d_indice2.getValue()*Vector3::size()+2,off2,off1,-m_force[2]);

//    M->add(d_indice2.getValue()*Vector3::size()+0,d_indice1.getValue()*Vector3::size()+0,off1,off2,-m_force[0]);
//    M->add(d_indice2.getValue()*Vector3::size()+1,d_indice1.getValue()*Vector3::size()+1,off1,off2,-m_force[1]);
//    M->add(d_indice2.getValue()*Vector3::size()+2,d_indice1.getValue()*Vector3::size()+2,off1,off2,-m_force[2]);
}

void CollisionForceField::draw(const VisualParams *flag) {
    if (!flag->displayFlags().getShowForceFields()) return;
//    if (d_indice1.getValue() == -1) return;
//    if (d_indice2.getValue() == -1) return;

//    Vector3 A = p_state1->get(VecCoordId::position())[d_indice1.getValue()];
//    Vector3 B = p_state2->get(VecCoordId::position())[d_indice2.getValue()];

//    glLineWidth(3);
//    glBegin(GL_LINES);
//    glColor3f(0.8f,0.2f,0.2f); glVertex3dv(A.data());
//    glColor3f(1.0f,0.6f,0.6f); glVertex3dv(B.data());
//    glEnd(); // GL_LINES
//    glLineWidth(1);

    collisionAlgorithm::PairProximityVector & collision = p_collision->getCollisionPairs();
    for (unsigned i=0;i<collision.size();i++) {
        Vector3 P = collision[i].first->getPosition();
        Vector3 Q = collision[i].second->getPosition();


        glLineWidth(3);
        glBegin(GL_LINES);

        if (dot(P-Q,collision[i].second->getNormal())<0) {
            glColor3f(0.8f,0.2f,0.2f); glVertex3dv(P.data());
            glColor3f(1.0f,0.6f,0.6f); glVertex3dv(Q.data());
        }
        glEnd(); // GL_LINES
        glLineWidth(1);
    }
}

DECLARE_CLASS(CollisionForceField)

}

