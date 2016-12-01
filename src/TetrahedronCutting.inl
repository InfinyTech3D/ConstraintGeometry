 /******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYALGORITHMS_INL
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYALGORITHMS_INL

#include "TetrahedronCutting.h"

//#define PRINTDEBUG

#define eqEdge(X, Y, A, B)  ((X==A && Y==B) || (X==B && Y==A))

namespace sofa {

namespace component {

namespace topology {

using namespace sofa::defaulttype;
using namespace sofa::core::behavior;


template<class DataTypes>
TetrahedronCutting<DataTypes>::TetrahedronCutting()
: f_rangeCoeff( initData(&f_rangeCoeff, 0.2 ,"rangeCoeff","Coeff of created point are in [f_coeffRange,1-f_coeffRange]"))
, f_resetCut( initData(&f_resetCut, (unsigned) 0,"resetCut","Number of time step before reseting the cuts"))
, f_followCut( initData(&f_followCut, false ,"followCut","Only one cut is active"))
, f_drawActiveEdge( initData(&f_drawActiveEdge, false ,"drawActiveEdge","draw the cutted tetra"))
, f_drawAddedPoints( initData(&f_drawAddedPoints, false ,"drawAddedPoints","draw the added Points"))
{
}

template<class DataTypes>
void TetrahedronCutting< DataTypes >::init() {
    this->getContext()->get(m_container);
    this->getContext()->get(tetrahedronModifier);
    this->getContext()->get(tetrahedronGeo);

    if (m_container==NULL) serr << "didn't find the m_container" << sendl;
    if (tetrahedronModifier==NULL) serr << "didn't find the m_container" << sendl;
    if (tetrahedronGeo==NULL) serr << "didn't find the m_container" << sendl;

    nbStep=-1;

    m_container->getTrianglesInTetrahedron(0);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::subDivideTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2) {
    Coord normal=(pre2-pre1).cross(curr1-pre1);
    normal.normalize();

    CollisionDetectionBtwEdgesPlane(false,normal,pre1,pre2,curr1,curr2);

    if (intersectedEdge.size()) {
        subdivideCollisionDetection();

        //Create single point where the temporary cut is sufficient
        snapTemporaryNodes();

        //Duplicate the points using the permanent configuration
        snapPermanentNodes();
    }

    resetCuts();
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::carveTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2) {
    Coord normal=(pre2-pre1).cross(curr1-pre1);
    normal.normalize();

    CollisionDetectionBtwEdgesPlane(false,normal,pre1,pre2,curr1,curr2);

    carveCollisionDetection();
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::subDivideRestTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2) {
    Coord normal=(pre2-pre1).cross(curr1-pre1);
    normal.normalize();

    CollisionDetectionBtwEdgesPlane(true,normal,pre1,pre2,curr1,curr2);

    if (intersectedEdge.size()) {
        subdivideCollisionDetection();

        //Create single point where the temporary cut is sufficient
        snapTemporaryNodes();

        //Duplicate the points using the permanent configuration
        snapPermanentNodes();
    }

    resetCuts();
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::carveRestTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2) {
    Coord normal=(pre2-pre1).cross(curr1-pre1);
    normal.normalize();

    CollisionDetectionBtwEdgesPlane(true,normal,pre1,pre2,curr1,curr2);

    carveCollisionDetection();
}

/////////////////////////////////////////////////////
////////////////COLLISION DETECTION//////////////////
/////////////////////////////////////////////////////

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isPointInPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2, Coord& edgeDirec0, Coord& edgeDirec1, Coord& edgeDirec2, Coord& edgeDirec3, Coord& intersectedPoint) {
    Coord direcToIntersection[4];
    Coord crossProduct[4];
    Real dotProduct[4];

    direcToIntersection[0]=intersectedPoint-pre1;
    direcToIntersection[1]=intersectedPoint-pre2;
    direcToIntersection[2]=intersectedPoint-curr2;
    direcToIntersection[3]=intersectedPoint-curr1;

    crossProduct[0]=direcToIntersection[0].cross(edgeDirec0);
    crossProduct[1]=direcToIntersection[1].cross(edgeDirec1);
    crossProduct[2]=direcToIntersection[2].cross(edgeDirec2);
    crossProduct[3]=direcToIntersection[3].cross(edgeDirec3);

    crossProduct[0].normalize();
    crossProduct[1].normalize();
    crossProduct[2].normalize();
    crossProduct[3].normalize();

    dotProduct[0]=crossProduct[0]*crossProduct[1];
    dotProduct[1]=crossProduct[1]*crossProduct[2];
    dotProduct[2]=crossProduct[2]*crossProduct[3];
    dotProduct[3]=crossProduct[3]*crossProduct[0];

    return dotProduct[0]*dotProduct[1]>=0 && dotProduct[0]*dotProduct[2]>=0 && dotProduct[0]*dotProduct[3]>=0;
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::CollisionDetectionBtwEdgesPlane(bool rest, Coord & normal, Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2) {
    intersectedEdge.clear();

    //inspect the plane edge intersection
    for(int i=0;i<m_container->getNbEdges();i++) {
        bool hasIntersection;
        Coord intersection;
        if (rest) hasIntersection = tetrahedronGeo->computeRestEdgePlaneIntersection(i,pre1,normal,intersection);
        else hasIntersection = tetrahedronGeo->computeEdgePlaneIntersection(i,pre1,normal,intersection);

        if (hasIntersection) {
            Coord edgeDirec[4];

            edgeDirec[0]=pre2-pre1;
            edgeDirec[1]=curr2-pre2;
            edgeDirec[2]=curr1-curr2;
            edgeDirec[3]=pre1-curr1;

            if(isPointInPlane(pre1, pre2, curr1, curr2,edgeDirec[0],edgeDirec[1],edgeDirec[2],edgeDirec[3], intersection)) {
                Edge theEdge = m_container->getEdge(i);
                sofa::helper::vector< double > coef;

                if (rest) coef = tetrahedronGeo->computeRest2PointsBarycoefs(intersection, theEdge[0], theEdge[1]);
                else coef = tetrahedronGeo->compute2PointsBarycoefs(intersection, theEdge[0], theEdge[1]);

                Intersection inter;
                inter.eid = i;
                inter.e0 = theEdge[0];
                inter.e1 = theEdge[1];
                inter.alpha = coef[1];

                intersectedEdge.push_back(inter);
            }
        }
    }
}


template<class DataTypes>
void  TetrahedronCutting<DataTypes>::carveCollisionDetection() {
    toBeAddedPointcoefs.clear();
    toBeAddedPointAncestors.clear();
    toBeAddedTetra.clear();
    toBeAddedTetraIndex.clear();
    toBeRemovedTetraIndex.clear();

    temporaryPoints.clear();

    for(unsigned it=0;it<intersectedEdge.size();++it) {
        const Intersection & inter = intersectedEdge[it];

        const core::topology::BaseMeshTopology::TetrahedraAroundEdge & tae = m_container->getTetrahedraAroundEdge(inter.eid);

        for (unsigned i=0;i<tae.size();i++) {
            addSetVector(tae[i],toBeRemovedTetraIndex);
        }
    }

    propagateModifications();
}

/////////////////////////////////////////////////////
////////////////UTILITY FUNCTION/////////////////////
/////////////////////////////////////////////////////

template<class DataTypes>
void TetrahedronCutting<DataTypes>::propagateModifications() {
    // Add the tetra id
    unsigned nbTetra = m_container->getNbTetrahedra();
    for (unsigned i=0;i<toBeAddedTetra.size();i++) {
        toBeAddedTetraIndex.push_back(nbTetra+i);
    }

//    if (toBeAddedPointcoefs.size()) {
//        std::cout << "toBeAddedPointcoefs= " << toBeAddedPointcoefs << std::endl;
//        std::cout << "toBeAddedPointAncestors= " << toBeAddedPointAncestors << std::endl;
//    }
//    if (toBeAddedTetra.size()) {
//        std::cout << "toBeAddedTetra= " << toBeAddedTetra << std::endl;
//        std::cout << "toBeAddedTetraIndex= " << toBeAddedTetraIndex << std::endl;
//    }
//    if (toBeRemovedTetraIndex.size()) {
//        std::cout << "toBeRemovedTetraIndex= " << toBeRemovedTetraIndex << std::endl;
//    }

    //point addition
    if (toBeAddedPointcoefs.size()) {
        tetrahedronModifier->addPointsWarning(toBeAddedPointcoefs.size(), toBeAddedPointAncestors, toBeAddedPointcoefs, true);
        tetrahedronModifier->addPointsProcess(toBeAddedPointcoefs.size());
    }

    //tetrahedron addition
    if (toBeAddedTetra.size()) {
        tetrahedronModifier->addTetrahedraWarning(toBeAddedTetra.size(), toBeAddedTetra, toBeAddedTetraIndex);
        tetrahedronModifier->addTetrahedraProcess(toBeAddedTetra);
    }

    //tetrahedron removal
    if (toBeRemovedTetraIndex.size()) {
        tetrahedronModifier->removeTetrahedra(toBeRemovedTetraIndex);
    }

    if (toBeAddedPointcoefs.size() || toBeAddedTetra.size() || toBeRemovedTetraIndex.size()) {
        tetrahedronModifier->propagateTopologicalChanges();
        tetrahedronModifier->notifyEndingEvent();
        sout << "NbCutElement=" << toBeRemovedTetraIndex.size() << " NbAddedElement=" << toBeAddedTetraIndex.size() << sendl;
    }

    if (nbStep == -1) nbStep = 0;//to reset the cut
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::mergeSplittedPoints(const helper::vector<unsigned> & localBelow,const helper::vector<unsigned> & localAbove,helper::vector<unsigned> & globalBelow,helper::vector<unsigned> & globalAbove) {
    bool bb = false;
    bool ba = false;
    bool ab = false;
    bool aa = false;

    for (unsigned i=0;i<localBelow.size() && !bb;i++) {
        if (contains(localBelow[i],globalBelow)) bb = true;
    }

    for (unsigned i=0;i<localBelow.size() && !ba;i++) {
        if (contains(localBelow[i],globalAbove)) ba = true;
    }

    for (unsigned i=0;i<localAbove.size() && !ab;i++) {
        if (contains(localAbove[i],globalBelow)) ab = true;
    }

    for (unsigned i=0;i<localAbove.size() && !aa;i++) {
        if (contains(localAbove[i],globalAbove)) aa = true;
    }

    if (bb) {
        if (ba || ab) {
            serr << "local bealow and above are present both side cannot merge the indices !!!" << sendl;
            return false;
        }

        for (unsigned j=0;j<localBelow.size();j++) addSetVector(localBelow[j],globalBelow);
        for (unsigned j=0;j<localAbove.size();j++) addSetVector(localAbove[j],globalAbove);
        return true;
    }

    if (ba) {
        if (bb || aa) {
            serr << "local bealow and above are present both side cannot merge the indices !!!" << sendl;
            return false;
        }

        for (unsigned j=0;j<localBelow.size();j++) addSetVector(localBelow[j],globalAbove);
        for (unsigned j=0;j<localAbove.size();j++) addSetVector(localAbove[j],globalBelow);
        return true;
    }

    if (ab) {
        if (bb || aa) {
            serr << "local bealow and above are present both side cannot merge the indices !!!" << sendl;
            return false;
        }

        for (unsigned j=0;j<localBelow.size();j++) addSetVector(localBelow[j],globalAbove);
        for (unsigned j=0;j<localAbove.size();j++) addSetVector(localAbove[j],globalBelow);
        return true;
    }

    if (aa) {
        if (ba || ab) {
            serr << "local bealow and above are present both side cannot merge the indices !!!" << sendl;
            return false;
        }

        for (unsigned j=0;j<localBelow.size();j++) addSetVector(localBelow[j],globalBelow);
        for (unsigned j=0;j<localAbove.size();j++) addSetVector(localAbove[j],globalAbove);
        return true;
    }

    for (unsigned j=0;j<localBelow.size();j++) addSetVector(localBelow[j],globalBelow);
    for (unsigned j=0;j<localAbove.size();j++) addSetVector(localAbove[j],globalAbove);

    return true;
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::computeTetraAncesor(TetraID tid, TetraInfo & res) {
    const Tetra & tetra = m_container->getTetra(tid);

    // First determine the tetra on the original mesh
    helper::vector<unsigned> ancPoints;
    for (unsigned p=0;p<4;p++) {
        unsigned addID;
        if (isPermanantAddedPoint(tetra[p],addID)) {
            addSetVector(addedPoints[addID].a0,ancPoints);
            addSetVector(addedPoints[addID].a1,ancPoints);
        } else {
            addSetVector(tetra[p],ancPoints);
        }
    }

    if (ancPoints.size()!=4) {
        serr << "TetraAnc has no 4 points :: tetra is " << tetra << " ancPoints are " << ancPoints << " attach to = ";
        for (unsigned p=0;p<4;p++) {
            unsigned addID;
            if (isPermanantAddedPoint(tetra[p],addID)) serr << addedPoints[addID].pid << "(" << addedPoints[addID].a0 << " " << addedPoints[addID].a1 << ") ";
            else serr << tetra[p] << " ";
        }
        serr << sendl;

        return;
    }

    res.tid = tid;
    res.ancTetra[0] = ancPoints[0];
    res.ancTetra[1] = ancPoints[1];
    res.ancTetra[2] = ancPoints[2];
    res.ancTetra[3] = ancPoints[3];
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::computeTetraConfiguration(TetraInfo & res) {
    res.configuration=255; // undefined conficuration

    //Finally compute the configuration of the tetra and renumber points
    if (res.id0==4 && res.id1==0 && res.id2==0 && res.id3==0 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[res.status0[0]];
        res.renumberedId[1] = res.ancTetra[res.status0[1]];
        res.renumberedId[2] = res.ancTetra[res.status0[2]];
        res.renumberedId[3] = res.ancTetra[res.status0[3]];
        res.configuration = 0;
    } else if (res.id0==2 && res.id1==2 && res.id2==0 && res.id3==0 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[res.status1[0]];
        res.renumberedId[1] = res.ancTetra[res.status1[1]];
        res.renumberedId[2] = res.ancTetra[res.status0[0]];
        res.renumberedId[3] = res.ancTetra[res.status0[1]];
        res.configuration = 1;
    } else if (res.id0==1 && res.id1==2 && res.id2==1 && res.id3==0 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[res.status2[0]];
        res.renumberedId[1] = res.ancTetra[res.status1[0]];
        res.renumberedId[2] = res.ancTetra[res.status1[1]];
        res.renumberedId[3] = res.ancTetra[res.status0[0]];
        res.configuration = 2;
    } else if (res.id0==0 && res.id1==2 && res.id2==2 && res.id3==0 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[res.status2[0]];
        res.renumberedId[1] = res.ancTetra[res.status2[1]];
        res.renumberedId[2] = res.ancTetra[res.status1[0]];
        res.renumberedId[3] = res.ancTetra[res.status1[1]];
        res.configuration = 3;
    } else if (res.id0==3 && res.id1==0 && res.id2==0 && res.id3==0 && res.id4==1) {
        res.renumberedId[0] = res.ancTetra[res.status0[0]];
        res.renumberedId[1] = res.ancTetra[res.status4[0]];
        res.renumberedId[2] = res.ancTetra[res.status0[1]];
        res.renumberedId[3] = res.ancTetra[res.status0[2]];
        res.configuration = 4;
    } else if (res.id0==1 && res.id1==2 && res.id2==0 && res.id3==0 && res.id4==1) {
        res.renumberedId[0] = res.ancTetra[res.status1[0]];
        res.renumberedId[1] = res.ancTetra[res.status1[1]];
        res.renumberedId[2] = res.ancTetra[res.status4[0]];
        res.renumberedId[3] = res.ancTetra[res.status0[0]];
        res.configuration = 5;
    } else if (res.id0==2 && res.id1==0 && res.id2==0 && res.id3==0 && res.id4==2) {
        res.renumberedId[0] = res.ancTetra[res.status0[0]];
        res.renumberedId[1] = res.ancTetra[res.status4[0]];
        res.renumberedId[2] = res.ancTetra[res.status0[1]];
        res.renumberedId[3] = res.ancTetra[res.status4[1]];
        res.configuration = 6;
    } else if (res.id0==0 && res.id1==4 && res.id2==0 && res.id3==0 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[res.status1[0]];
        res.renumberedId[1] = res.ancTetra[res.status1[1]];
        res.renumberedId[2] = res.ancTetra[res.status1[2]];
        res.renumberedId[3] = res.ancTetra[res.status1[3]];
        res.configuration = 7;
    } else if (res.id0==0 && res.id1==3 && res.id2==0 && res.id3==1 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[res.status3[0]];
        res.renumberedId[1] = res.ancTetra[res.status1[0]];
        res.renumberedId[2] = res.ancTetra[res.status1[1]];
        res.renumberedId[3] = res.ancTetra[res.status1[2]];
        res.configuration = 8;
    } else if (res.id0==0 && res.id1==0 && res.id2==4 && res.id3==0 && res.id4==0) {
        res.renumberedId[0] = res.ancTetra[0];
        res.renumberedId[1] = res.ancTetra[2];
        res.renumberedId[2] = res.ancTetra[3];
        res.renumberedId[3] = res.ancTetra[1];
        res.configuration = 9;
    } else if (res.id0==1 && res.id1==0 && res.id2==0 && res.id3==0 && res.id4==3) {
        res.renumberedId[0] = res.ancTetra[res.status0[0]];
        res.renumberedId[1] = res.ancTetra[res.status4[0]];
        res.renumberedId[2] = res.ancTetra[res.status4[1]];
        res.renumberedId[3] = res.ancTetra[res.status4[2]];
        res.configuration = 10;
    } else if (res.id0==0 && res.id1==2 && res.id2==0 && res.id3==0 && res.id4==2) {
        res.renumberedId[0] = res.ancTetra[res.status1[0]];
        res.renumberedId[1] = res.ancTetra[res.status4[0]];
        res.renumberedId[2] = res.ancTetra[res.status1[1]];
        res.renumberedId[3] = res.ancTetra[res.status4[1]];
        res.configuration = 11;
    } else if (res.id0==0 && res.id1==2 && res.id2==1 && res.id3==0 && res.id4==1) {
        res.renumberedId[0] = res.ancTetra[res.status2[0]];
        res.renumberedId[1] = res.ancTetra[res.status1[0]];
        res.renumberedId[2] = res.ancTetra[res.status1[1]];
        res.renumberedId[3] = res.ancTetra[res.status4[0]];
        res.configuration = 12;
    }

    //std::cout << "CONFIGURATION " << res.configuration << " In tetra " << m_container->getTetra(tid) << " tetraAnc is " << res.ancTetra[0] << " " << res.ancTetra[1] << " " << res.ancTetra[2] << " " << res.ancTetra[3] << " status0=" << res.id0 << " status1=" << res.id1 << " status2=" << res.id2 << " status3=" << res.id3 << " status4=" << res.id4 << std::endl;

    //std::cout << "CONFIGURATION= " << res.configuration << " tetra " << m_container->getTetra(tid) << " res.tetraAnc is " << res.ancTetra[0] << " " << res.ancTetra[1] << " " << res.ancTetra[2] << " " << res.ancTetra[3] << " renumber is " << res.renumberedId[0] << " " << res.renumberedId[1] << " " << res.renumberedId[2] << " " << res.renumberedId[3] << " status0=" << res.id0 << " status1=" << res.id1 << " status2=" << res.id2 << " status3=" << res.id3 << " status4=" << res.id4 << std::endl;
    if (res.configuration==255) {
        serr << " Error in configuration computation of tetra " << m_container->getTetra(res.tid) << " tetraAnc is " << res.ancTetra[0] << " " << res.ancTetra[1] << " " << res.ancTetra[2] << " " << res.ancTetra[3] << " status0=" << res.id0 << " status1=" << res.id1 << " status2=" << res.id2 << " status3=" << res.id3 << " status4=" << res.id4 << sendl;
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::getTetraInfo(TetraID tid, TetraInfo & res) {
    computeTetraAncesor(tid,res);

    // Then compute the status of each point according to the number of cut on each edge
    unsigned nbCut[4] = {0,0,0,0};
    for (unsigned e0=0;e0<4;e0++) {
        for (unsigned e1=e0+1;e1<4;e1++) {
            if (isTemporaryBreakedEdge(res.ancTetra[e0],res.ancTetra[e1]) || isPermanantBreakedEdge(res.ancTetra[e0],res.ancTetra[e1])) {
                nbCut[e0]++;
                nbCut[e1]++;
            }
        }
    }

    for (unsigned p=0;p<4;p++) {
        if (isTemporaryDuplicatedPoint(res.ancTetra[p]) || isPermanantDuplicatePoint(res.ancTetra[p])) {
            //test if the ancessor is in tetra if the point is duplicated node
            res.status4[res.id4++] = p;
        } else {
            if (nbCut[p]==0) res.status0[res.id0++] = p;
            else if (nbCut[p]==1) res.status1[res.id1++] = p;
            else if (nbCut[p]==2) res.status2[res.id2++] = p;
            else if (nbCut[p]==3) res.status3[res.id3++] = p;
            else {
                serr << " The point is cutted by more than 3 edges in the same tetra !!!" << sendl;
                return;
            }
        }
    }

    computeTetraConfiguration(res);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::getTetraInfoWithoutTemporaryCut(TetraID tid, TetraInfo & res,const helper::vector<unsigned> & skipID) {
    computeTetraAncesor(tid,res);

    // Then compute the status of each point according to the number of cut on each edge
    unsigned nbCut[4] = {0,0,0,0};
    for (unsigned e0=0;e0<4;e0++) {
        for (unsigned e1=e0+1;e1<4;e1++) {
            if (isPermanantBreakedEdge(res.ancTetra[e0],res.ancTetra[e1])) {
                nbCut[e0]++;
                nbCut[e1]++;
                continue;
            }

            unsigned tmpID;
            if (isTemporaryBreakedEdge(res.ancTetra[e0],res.ancTetra[e1],tmpID)) {
                if (!contains(tmpID,skipID)) {
                    nbCut[e0]++;
                    nbCut[e1]++;
                }
            }
        }
    }

    for (unsigned p=0;p<4;p++) {
        if (isPermanantDuplicatePoint(res.ancTetra[p])) {
            res.status4[res.id4++] = p;
            continue;
        }

        unsigned tmpID;
        if (isTemporaryDuplicatedPoint(res.ancTetra[p],tmpID)) {
            if (!contains(tmpID,skipID)) {
                //test if the ancessor is in tetra if the point is duplicated node
                res.status4[res.id4++] = p;
                continue;
            }
        }

        if (nbCut[p]==0) res.status0[res.id0++] = p;
        else if (nbCut[p]==1) res.status1[res.id1++] = p;
        else if (nbCut[p]==2) res.status2[res.id2++] = p;
        else if (nbCut[p]==3) res.status3[res.id3++] = p;
        else {
            serr << " The point is cutted by more than 3 edges in the same tetra !!!" << sendl;
            return;
        }
    }

    computeTetraConfiguration(res);
}

template<class DataTypes> template<class Element>
bool TetrahedronCutting<DataTypes>::contains(Element t,const helper::vector<Element> & vector) {
    bool flag = false;
    for (unsigned k=0;k<vector.size() && !flag;k++) {
        if (vector[k] == t) flag = true; // not a new tetra
    }

    return flag;
}

template<class DataTypes> template<class Element>
bool TetrahedronCutting<DataTypes>::addSetVector(Element t,helper::vector<Element> & vector) {
    if (! contains(t,vector)) {
        vector.push_back(t);
        return true;
    }

    return false;
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::rec_serach(unsigned pid,helper::vector<unsigned> & pointSearch,helper::vector<unsigned> & linkedPoints) {
    if (addSetVector(pid,linkedPoints)) {
        const core::topology::BaseMeshTopology::VerticesAroundVertex & vav = m_container->getVerticesAroundVertex(pid);
        for (unsigned v=0;v<vav.size();v++) {
            if (contains(vav[v],pointSearch)) rec_serach(vav[v],pointSearch,linkedPoints);
        }
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::createTemporaryPoint(unsigned a0,unsigned a1,double alpha) {
    TemporaryPoint tmp;
    tmp.a0 = a0;
    tmp.a1 = a1;
    tmp.alpha = alpha;
    temporaryPoints.push_back(tmp);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::createPermanantPoint(unsigned pid,unsigned a0,unsigned a1) {
    AddedPoint add;
    add.pid = pid;
    add.a0 = a0;
    add.a1 = a1;
    add.opp = -1;
    addedPoints.push_back(add);
}

template<class DataTypes>
unsigned TetrahedronCutting<DataTypes>::createPointOnMesh(unsigned e0,unsigned e1,double alpha) {
    sofa::helper::vector< unsigned int > ancestor;
    ancestor.push_back(e0); ancestor.push_back(e1);

    sofa::helper::vector< double > coeff;
    coeff.push_back(1.0-alpha); coeff.push_back(alpha);

    toBeAddedPointAncestors.push_back(ancestor);
    toBeAddedPointcoefs.push_back(coeff);

    return m_container->getNbPoints() + toBeAddedPointAncestors.size() - 1;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isPermanantAddedPoint(unsigned p,unsigned & addID) {
    for (addID=0;addID<addedPoints.size();addID++) {
        if (addedPoints[addID].pid == p) return true; // not a new tetra
        if (addedPoints[addID].opp != -1 && ((unsigned) addedPoints[addID].opp == p)) return true;
    }
    return false;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isPermanantAddedPoint(unsigned p) {
    unsigned i;
    return isPermanantAddedPoint(p,i);
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isPermanantDuplicatePoint(unsigned p) {
    for (unsigned i=0;i<addedPoints.size();i++) {
        if (addedPoints[i].a0 == p && addedPoints[i].a0 == addedPoints[i].a1) return true; // not a new tetra
    }
    return false;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isPermanantBreakedEdge(unsigned e0,unsigned e1) {
    for (unsigned i=0;i<addedPoints.size();i++) {
        if (eqEdge(e0,e1,addedPoints[i].a0,addedPoints[i].a1)) return true;
    }
    return false;
}


template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isTemporaryBreakedEdge(unsigned e0,unsigned e1,unsigned & tmpID) {
    for (tmpID=0;tmpID<temporaryPoints.size();tmpID++) {
        if (eqEdge(e0,e1,temporaryPoints[tmpID].a0,temporaryPoints[tmpID].a1)) return true;
    }
    return false;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isTemporaryBreakedEdge(unsigned e0,unsigned e1) {
    unsigned i;
    return isTemporaryBreakedEdge(e0,e1,i);
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isTemporaryDuplicatedPoint(unsigned pid,unsigned & tmpID) {
    for (tmpID=0;tmpID<temporaryPoints.size();tmpID++) {
        if (temporaryPoints[tmpID].a0 == pid && temporaryPoints[tmpID].a0 == temporaryPoints[tmpID].a1) return true;
    }
    return false;
}


template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isTemporaryDuplicatedPoint(unsigned pid) {
    unsigned i=0;
    return isTemporaryDuplicatedPoint(pid,i);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::getTemporaryArnoudCut(unsigned e0,unsigned e1,helper::vector<unsigned> & tmpAroundID) {
    unsigned tmpID0;
    unsigned tmpID1;

    bool istmp0 = isTemporaryDuplicatedPoint(e0,tmpID0);
    bool istmp1 = isTemporaryDuplicatedPoint(e1,tmpID1);

    if (istmp0 && istmp1) return;

    if (istmp0) tmpAroundID.push_back(tmpID0);
    if (istmp1) tmpAroundID.push_back(tmpID1);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::getTemporaryArnoudDuplicate(unsigned pid,helper::vector<unsigned> & tmpAroundID) {
    for (unsigned i=0;i<temporaryPoints.size();i++) {
        if (temporaryPoints[i].a0 == pid || temporaryPoints[i].a1 == pid) {
            if (temporaryPoints[i].a0 != temporaryPoints[i].a1) tmpAroundID.push_back(i);
        }
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::deletePermanentPoints() {
    addedPoints.clear();
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::deletePermanentUnduplicatePoints() {
    helper::vector<AddedPoint> copy;
    copy.resize(addedPoints.size());
    for (unsigned i=0;i<addedPoints.size();i++) copy[i] = addedPoints[i];

    addedPoints.clear();
    for (unsigned i=0;i<copy.size();i++) {
        if (copy[i].opp != -1) addedPoints.push_back(copy[i]);
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::deleteTemporaryPoints() {
    temporaryPoints.clear();
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::deleteTemporaryPoints(const helper::vector<unsigned> & tmpVecID) {
    helper::vector<TemporaryPoint> copy;
    copy.resize(temporaryPoints.size());
    for (unsigned i=0;i<temporaryPoints.size();i++) copy[i] = temporaryPoints[i];

    temporaryPoints.clear();
    for (unsigned i=0;i<copy.size();i++) {
        if (! contains(i,tmpVecID)) temporaryPoints.push_back(copy[i]);
    }
}

/////////////////////////////////////////////////////
////////////////////DUPLICATE NODES//////////////////
/////////////////////////////////////////////////////

template<class DataTypes>
void TetrahedronCutting<DataTypes>::duplicateTemporaryPoint(unsigned pid) {
    helper::vector<unsigned> tmpAroundID;
    getTemporaryArnoudDuplicate(pid,tmpAroundID);

    deleteTemporaryPoints(tmpAroundID);

    TemporaryPoint tmp;
    tmp.a0 = pid;
    tmp.a1 = pid;
    tmp.alpha = 0.0;
    temporaryPoints.push_back(tmp);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::breakTemporaryEdge(unsigned e0,unsigned e1,double alpha) {
    helper::vector<unsigned> tmpAroundID;
    getTemporaryArnoudCut(e0,e1,tmpAroundID);

    deleteTemporaryPoints(tmpAroundID);

    unsigned tmpID;
    if (isTemporaryBreakedEdge(e0,e1,tmpID)) {
        temporaryPoints[tmpID].alpha = alpha;
    } else {
        TemporaryPoint tmp;
        tmp.a0 = e0;
        tmp.a1 = e1;
        tmp.alpha = alpha;
        temporaryPoints.push_back(tmp);
    }
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::canDuplicateNodeInTetra(unsigned p,const TetraInfo & ti) {
    if (ti.configuration == 0) {
        return true;
    } else if (ti.configuration == 1) {
        if (p == ti.renumberedId[2]) return true;
        if (p == ti.renumberedId[3]) return true;
    } else if (ti.configuration == 2) {
        if (p == ti.renumberedId[3]) return true;
    } else if (ti.configuration == 4) {
        if (p == ti.renumberedId[0]) return true;
        if (p == ti.renumberedId[2]) return true;
        if (p == ti.renumberedId[3]) return true;
    } else if (ti.configuration == 5) {
        if (p == ti.renumberedId[3]) return true;
    } else if (ti.configuration == 6) {
        if (p == ti.renumberedId[0]) return true;
        if (p == ti.renumberedId[2]) return true;
    }

    return false;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::canDuplicatePoint(unsigned p) {
    bool cutting = false;

    helper::vector<unsigned> tmpAroundID;
    getTemporaryArnoudDuplicate(p,tmpAroundID);

    const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tav = m_container->getTetrahedraAroundVertex(p);
    for (unsigned t=0;t<tav.size();t++) {
        TetraInfo ti;
        getTetraInfoWithoutTemporaryCut(tav[t],ti,tmpAroundID);

        if (! canDuplicateNodeInTetra(p,ti)) return false;

        if (ti.configuration > 0 && ti.configuration < 8) cutting = true;
    }

    if (f_followCut.getValue() && !(addedPoints.empty() && temporaryPoints.empty()) && !cutting) return false;

    return true;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::canSplitEdgeInTetra(unsigned e0,unsigned e1,const TetraInfo & ti) {
    if (ti.configuration == 0) {
        return true;
    } else if (ti.configuration == 1) {
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[2])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[1],ti.renumberedId[2])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[1],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[2],ti.renumberedId[3])) return true;
    } else if (ti.configuration == 2) {
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[2],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[1],ti.renumberedId[3])) return true;
    } else if (ti.configuration == 3) {
        if (eqEdge(e0,e1,ti.renumberedId[2],ti.renumberedId[3])) return true;
    } else if (ti.configuration == 4) {
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[2])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[2],ti.renumberedId[3])) return true;
    } else if (ti.configuration == 5) {
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[1],ti.renumberedId[3])) return true;
    } else if (ti.configuration == 6) {
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[2])) return true;
    } else if (ti.configuration == 7) {
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[2])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[0],ti.renumberedId[3])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[1],ti.renumberedId[2])) return true;
        if (eqEdge(e0,e1,ti.renumberedId[1],ti.renumberedId[3])) return true;
    }

    return false;
}

template<class DataTypes>
bool TetrahedronCutting<DataTypes>::isbreakableEdge(unsigned eid,unsigned e0,unsigned e1) {
    bool cutting = false;

    helper::vector<unsigned> tmpAroundID;
    getTemporaryArnoudCut(e0,e1,tmpAroundID);

    const TetrahedraAroundEdge & tae=m_container->getTetrahedraAroundEdge(eid);
    for (unsigned t=0;t<tae.size();t++) {
        TetraInfo ti;
        getTetraInfoWithoutTemporaryCut(tae[t],ti,tmpAroundID);

        if (! canSplitEdgeInTetra(e0,e1,ti)) return false;

        if (ti.configuration > 0 && ti.configuration < 8) cutting = true;
    }

    if (f_followCut.getValue() && !(addedPoints.empty() && temporaryPoints.empty()) && !cutting) return false;

    return true;
}

template<class DataTypes>
void  TetrahedronCutting<DataTypes>::subdivideCollisionDetection() {
    for(unsigned it=0;it<intersectedEdge.size();++it) {
        const Intersection & inter = intersectedEdge[it];

        if (inter.alpha<f_rangeCoeff.getValue()) {
            if (canDuplicatePoint(inter.e0)) duplicateTemporaryPoint(inter.e0);
        } else if (inter.alpha>1.0-f_rangeCoeff.getValue())  {
            if (canDuplicatePoint(inter.e1)) duplicateTemporaryPoint(inter.e1);
        } else {
            if (isTemporaryBreakedEdge(inter.e0,inter.e1) || isbreakableEdge(inter.eid,inter.e0,inter.e1)) breakTemporaryEdge(inter.e0,inter.e1,inter.alpha);
        }
    }
}

///////////////////////////////////////////////////////
////////////////////SNAP TEMPORARY ROUTINE/////////////
///////////////////////////////////////////////////////

template<class DataTypes>
void TetrahedronCutting<DataTypes>::findAndSubdivide(TetraList & sub,TetraList & subdividedTetra,const AddedPoint & add) {
    helper::vector<int> contains; // count how many point from intersectedEdge1 each tetra has in sub0
    contains.resize(sub.size());

    // Find the tetra which must be subdivided one more time
    for (unsigned j=0;j<contains.size();j++) {
        contains[j]=0;
        for (int i=0;i<4;i++) {
            if ((sub[j][i]==add.a0) || (sub[j][i]==add.a1)) contains[j]++;
        }
    }

    // Add the sub0 tetra to the result subdividedTetra
    for (unsigned j=0;j<contains.size();j++) {
        if (contains[j]==2) { // if the edge is in the tetra, we subdivide the tetra in two
            TetraList sub1;
            cutTetra1(sub[j],sub1,add);
            if (sub1.size()==2) {
                subdividedTetra.push_back(sub1[0]);
                subdividedTetra.push_back(sub1[1]);
            }
        } else { // however, we add it to the subdivided tetra
            subdividedTetra.push_back(sub[j]);
        }
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::cutTetra1(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0) {
    Tetra subTetraL; // low tetra
    Tetra subTetraH; // high tetra

    // we build the tetra, and we get the same orientation as tetra
    for (int i=0;i<4;i++) {
        if (tetra[i]==add0.a0) {
            subTetraH[i] = add0.a0;
            subTetraL[i] = add0.pid;
        } else if (tetra[i]==add0.a1) {
            subTetraH[i] = add0.pid;
            subTetraL[i] = add0.a1;
        } else {
            subTetraH[i] = tetra[i];
            subTetraL[i] = tetra[i];
        }
    }

    subdividedTetra.push_back(subTetraL);
    subdividedTetra.push_back(subTetraH);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::cutTetra2(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0,const AddedPoint & add1) {
    TetraList sub0;
    cutTetra1(tetra,sub0,add0);

    helper::vector<int> contains; // count how many point from intersectedEdge1 each tetra has in sub0
    contains.resize(sub0.size());

    findAndSubdivide(sub0,subdividedTetra,add1);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::cutTetra3(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0,const AddedPoint & add1,const AddedPoint & add2) {
    TetraList sub0;
    cutTetra2(tetra,sub0,add0,add1);

    findAndSubdivide(sub0,subdividedTetra,add2);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::cutTetra4(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0,const AddedPoint & add1,const AddedPoint & add2,const AddedPoint & add3) {
    TetraList sub0;
    cutTetra3(tetra,sub0,add0,add1,add2);

    findAndSubdivide(sub0,subdividedTetra,add3);
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::breakPermanantEdge(unsigned addedSize) {
    //Getting intersected edgeid
    helper::vector<EdgeID> edgeIndexes;
    helper::vector<unsigned> addIndexes;

    //only for new added point (from addedSize to addedPoints.size()
    for(unsigned it=addedSize;it<addedPoints.size();it++) {
        const AddedPoint & add = addedPoints[it];

        if (add.a0!=add.a1) {
            EdgeID eid = m_container->getEdgeIndex(add.a0,add.a1);
            edgeIndexes.push_back(eid);
            addIndexes.push_back(it);
        }
    }

    //split the tetrahedra in two
    for(unsigned it=0;it<edgeIndexes.size();++it) {
        //Getting the tetrahedron around each intersected edge
        const TetrahedraAroundEdge & tae = m_container->getTetrahedraAroundEdge(edgeIndexes[it]);
        for( unsigned int j=0;j<tae.size();j++) {
            TetraID tetraId = tae[j];

            //Check the configuration before subdividing !!!
            TetraInfo tinfo;
            getTetraInfo(tetraId,tinfo);
            if (tinfo.configuration == 255) {
                serr << "Error the configuration of this tetra is not correct, cannot subdivide this tetra" << sendl;
                continue;
            }

            //it's a new tetra which is cut
            if (addSetVector(tetraId,toBeRemovedTetraIndex)) {
                //determine the index of intersected edges in tetra
                sofa::helper::vector< unsigned > intersectionInTetra;
                const EdgesInTetrahedron & eit = m_container->getEdgesInTetrahedron(tetraId);

                for( unsigned int e=0;e<eit.size();e++) {
                    for(unsigned it2=0;it2<edgeIndexes.size();it2++) {
                        if (eit[e] == edgeIndexes[it2]) intersectionInTetra.push_back(addIndexes[it2]);
                    }
                }

                const Tetra & tetra = m_container->getTetra(tetraId);

                     if (intersectionInTetra.size()==1) cutTetra1(tetra,toBeAddedTetra,addedPoints[intersectionInTetra[0]]);
                else if (intersectionInTetra.size()==2) cutTetra2(tetra,toBeAddedTetra,addedPoints[intersectionInTetra[0]],addedPoints[intersectionInTetra[1]]);
                else if (intersectionInTetra.size()==3) cutTetra3(tetra,toBeAddedTetra,addedPoints[intersectionInTetra[0]],addedPoints[intersectionInTetra[1]],addedPoints[intersectionInTetra[2]]);
                else if (intersectionInTetra.size()==4) cutTetra4(tetra,toBeAddedTetra,addedPoints[intersectionInTetra[0]],addedPoints[intersectionInTetra[1]],addedPoints[intersectionInTetra[2]],addedPoints[intersectionInTetra[3]]);
                else std::cerr << "Error : a plane cannot cut more than 4 edges" << sendl;
            }
        }
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::recTemporarySerach(unsigned pid,helper::vector<unsigned> & pointSearch,helper::vector<unsigned> & linkedPoints) {
    if (addSetVector(pid,linkedPoints)) {
        const core::topology::BaseMeshTopology::VerticesAroundVertex & vav = m_container->getVerticesAroundVertex(pid);
        for (unsigned v=0;v<vav.size();v++) {
            if (! contains(vav[v],pointSearch)) continue;
            if (isTemporaryBreakedEdge(pid,vav[v])) continue;

            recTemporarySerach(vav[v],pointSearch,linkedPoints);
        }
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::getTemporaryPointAround(const TemporaryPoint & tmp,helper::vector<unsigned> & pointAround) {
    if (tmp.a0 == tmp.a1) { //temporaryDuplicate
        const core::topology::BaseMeshTopology::VerticesAroundVertex & vav = m_container->getVerticesAroundVertex(tmp.a0);
        for (unsigned v=0;v<vav.size();v++) {
            if (isTemporaryDuplicatedPoint(vav[v])) continue;
            if (isTemporaryBreakedEdge(tmp.a0,vav[v])) continue;
            if (isPermanantAddedPoint(vav[v])) continue;

            addSetVector(vav[v],pointAround);
        }
    } else {
        pointAround.push_back(tmp.a0);
        pointAround.push_back(tmp.a1);
        EdgeID eid = m_container->getEdgeIndex(tmp.a0,tmp.a1);
        const core::topology::BaseMeshTopology::TetrahedraAroundEdge & tae = m_container->getTetrahedraAroundEdge(eid);
        for (unsigned t=0;t<tae.size();t++) {
            const Tetra & tetra = m_container->getTetra(tae[t]);

            for (unsigned p=0;p<4;p++) {
                if (contains(tetra[p],pointAround)) continue;
                if (isPermanantAddedPoint(tetra[p])) continue;
                if (isTemporaryDuplicatedPoint(tetra[p])) continue;
                if (isTemporaryBreakedEdge(tmp.a0,tetra[p]) && isTemporaryBreakedEdge(tmp.a1,tetra[p])) continue;

                pointAround.push_back(tetra[p]);
            }
        }
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::splitTemporaryPointAround(unsigned id,helper::vector<bool> & visited,helper::vector<unsigned> & snapTemporaryID,helper::vector<unsigned> & globalBelow,helper::vector<unsigned> & globalAbove) {
    if (visited[id]) return;
    visited[id] = true;

    helper::vector<unsigned> localAbove;
    helper::vector<unsigned> localBelow;
    helper::vector<unsigned> pointAround;
    helper::vector<unsigned> notBelow;

    const TemporaryPoint & tmp = temporaryPoints[id];

    getTemporaryPointAround(tmp,pointAround);

    if (pointAround.empty()) return;

    //collect all the point in pointAround which are connected to pointAround[0]
    recTemporarySerach(pointAround[0],pointAround,localBelow);

    //add all the point which are not below
    for (unsigned p=0;p<pointAround.size();p++) {
        if (!contains(pointAround[p],localBelow)) notBelow.push_back(pointAround[p]);
    }

    if (notBelow.empty()) return;

    //collect all the point in pointAround which are connected to pointAround[0]
    recTemporarySerach(notBelow[0],notBelow,localAbove);


//    std::cout << "Point = [" << tmp.a0 << "," << tmp.a1 << "] PointAround = " << pointAround << " LOCAL BELOW = " << localBelow << " LOCAL ABOVE = " << localAbove << std::endl;
    if (localAbove.size()+localBelow.size() != pointAround.size()) return; // that that we have all the point (i.e the point is separated by only ont cut

    snapTemporaryID.push_back(id);
    mergeSplittedPoints(localBelow,localAbove,globalBelow,globalAbove);

    for (unsigned t=0;t<temporaryPoints.size();t++) {
             if (temporaryPoints[t].a0 == tmp.a0 || temporaryPoints[t].a1 == tmp.a0) splitTemporaryPointAround(t,visited,snapTemporaryID,globalBelow,globalAbove);
        else if (temporaryPoints[t].a0 == tmp.a1 || temporaryPoints[t].a1 == tmp.a1) splitTemporaryPointAround(t,visited,snapTemporaryID,globalBelow,globalAbove);
        else if (contains(temporaryPoints[t].a0,pointAround) || contains(temporaryPoints[t].a1,pointAround)) splitTemporaryPointAround(t,visited,snapTemporaryID,globalBelow,globalAbove);
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::snapTemporaryNodes() {
    toBeAddedPointcoefs.clear();
    toBeAddedPointAncestors.clear();
    toBeAddedTetra.clear();
    toBeAddedTetraIndex.clear();
    toBeRemovedTetraIndex.clear();

    helper::vector<unsigned> snapID;
    helper::vector<unsigned> globalAbove;
    helper::vector<unsigned> globalBelow;
    helper::vector<bool> visited;

    visited.resize(temporaryPoints.size(),false);
    for (unsigned t=0;t<temporaryPoints.size();t++) splitTemporaryPointAround(t,visited,snapID,globalBelow,globalAbove);

    if (globalBelow.empty() || globalAbove.empty()) return;

//    std::cout << "snap id " << snapTemporaryID << std::endl;
//    std::cout << "GLOBAL TEMPORARY BELOW = " << globalBelow << std::endl;
//    std::cout << "GLOBAL TEMPORARY ABOVE = " << globalAbove << std::endl;

    //Create the points twice to be able to separate the cut
    unsigned addedSize = addedPoints.size();
    for (unsigned i=0;i<snapID.size();i++) {
        const TemporaryPoint & tmp = temporaryPoints[snapID[i]];

        //if the point is duplicated from the original mesh, we don't create a point in the topology but set the point as created
        if (tmp.a0 == tmp.a1) createPermanantPoint(tmp.a0,tmp.a0,tmp.a1);
        else createPermanantPoint(createPointOnMesh(tmp.a0,tmp.a1,tmp.alpha),tmp.a0,tmp.a1);
    }

//    std::cout << "DELETE TEMPORARY " << snapID << std::endl;
    deleteTemporaryPoints(snapID);

    breakPermanantEdge(addedSize);//subdivide the tetrahedra    

    propagateModifications();
}

///////////////////////////////////////////////////////////////////
//////////////////SNAP PERMANENT POINTS////////////////////////////
///////////////////////////////////////////////////////////////////

template<class DataTypes>
unsigned TetrahedronCutting<DataTypes>::getPidIndex(unsigned pid,const helper::vector<unsigned> & snapID) {
    unsigned addID;
    if (isPermanantAddedPoint(pid,addID)) {
        if (contains(addID,snapID)) return addedPoints[addID].pid;
    }

    return pid;
}

template<class DataTypes>
unsigned TetrahedronCutting<DataTypes>::getOppIndex(unsigned pid,const helper::vector<unsigned> & snapID) {
    unsigned addID;
    if (isPermanantAddedPoint(pid,addID)) {
        if (contains(addID,snapID)) {
            if (addedPoints[addID].opp==-1) addedPoints[addID].opp = createPointOnMesh(addedPoints[addID].pid,addedPoints[addID].a1,0.0);
            return addedPoints[addID].opp;
        }
    }

    return pid;
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::rebuildTetra(const helper::vector<unsigned> & snapID,const helper::vector<unsigned> & globalSetPoint) {
    //rebuild the tetra such as the don't cross the cut
    for (unsigned s=0;s<snapID.size();s++) {
        const AddedPoint & add = addedPoints[snapID[s]];

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tav = m_container->getTetrahedraAroundVertex(add.pid);
        for (unsigned t=0;t<tav.size();t++) {
            if (contains(tav[t],toBeRemovedTetraIndex)) continue;

            const Tetra & tetra = m_container->getTetra(tav[t]);

            if (contains(tetra[0],globalSetPoint) || contains(tetra[1],globalSetPoint) || contains(tetra[2],globalSetPoint) || contains(tetra[3],globalSetPoint)) {
                toBeRemovedTetraIndex.push_back(tav[t]);
            }
        }
    }

    for (unsigned i=0;i<toBeRemovedTetraIndex.size();i++) {
        Tetra tetra = m_container->getTetra(toBeRemovedTetraIndex[i]);

        tetra[0] = getOppIndex(tetra[0],snapID);
        tetra[1] = getOppIndex(tetra[1],snapID);
        tetra[2] = getOppIndex(tetra[2],snapID);
        tetra[3] = getOppIndex(tetra[3],snapID);

        toBeAddedTetra.push_back(tetra);
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::splitPermanentPointAround(unsigned aid,helper::vector<bool> & visited,helper::vector<unsigned> & snapID,helper::vector<unsigned> & globalBelow,helper::vector<unsigned> & globalAbove) {
    if (visited[aid]) return;
    visited[aid] = true;

    const AddedPoint & add = addedPoints[aid];
    if (add.opp != -1) return;

    helper::vector<unsigned> localAbove;
    helper::vector<unsigned> localBelow;
    helper::vector<unsigned> pointAround;
    helper::vector<unsigned> permanantAround;

    const core::topology::BaseMeshTopology::VerticesAroundVertex & vav = m_container->getVerticesAroundVertex(add.pid);
    for (unsigned v=0;v<vav.size();v++) {
        unsigned addID;
        if (isPermanantAddedPoint(vav[v],addID)) {
            if (addedPoints[addID].opp==-1)
                permanantAround.push_back(vav[v]);
            else pointAround.push_back(vav[v]); //the point is fully duplicate, we treat is as if it was a normal point
        } else pointAround.push_back(vav[v]);

    }

    if (pointAround.empty()) return;

    //collect all the point in pointAround which are connected to pointAround[0]
    rec_serach(pointAround[0],pointAround,localBelow);

    //add all the point which are not below
    helper::vector<unsigned> notBelow;
    for (unsigned p=0;p<pointAround.size();p++) {
        if (!contains(pointAround[p],localBelow)) notBelow.push_back(pointAround[p]);
    }


    if (notBelow.empty()) return; // if pointAround[0] is connected to all the point around the this point cannot be snapped

    //collect all the point in pointAround which are connected to pointAround[0]
    rec_serach(notBelow[0],notBelow,localAbove);

    if (localAbove.size()+localBelow.size() != pointAround.size()) {
//        printf("WARNING A POINT CANNOT BE DUPLICATE, ALL THE TERA ANOUD WILL BE DELETED\n");
        const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tav = m_container->getTetrahedraAroundVertex(add.pid);
        for (unsigned t=0;t<tav.size();t++) addSetVector(tav[t],toBeRemovedTetraIndex);

        return; //check if we have splitted all the points
    }

    if (! mergeSplittedPoints(localBelow,localAbove,globalBelow,globalAbove)) {
        const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tav = m_container->getTetrahedraAroundVertex(add.pid);
        for (unsigned t=0;t<tav.size();t++) addSetVector(tav[t],toBeRemovedTetraIndex);

        return;
    }

    addSetVector(aid,snapID);

    //    std::cout << "Point = " << add.pid << " [" << add.a0 << "," << add.a1 << "] PointAround = " << pointAround << " LOCAL BELOW = " << localBelow << " LOCAL ABOVE = " << localAbove << std::endl;
    for (unsigned p=0;p<permanantAround.size();p++) {
        unsigned addID;
        if (isPermanantAddedPoint(permanantAround[p],addID)) splitPermanentPointAround(addID,visited,snapID,globalBelow,globalAbove);
    }
}

template<class DataTypes>
void TetrahedronCutting<DataTypes>::snapPermanentNodes() {
    toBeAddedPointcoefs.clear();
    toBeAddedPointAncestors.clear();
    toBeAddedTetra.clear();
    toBeAddedTetraIndex.clear();
    toBeRemovedTetraIndex.clear();

    helper::vector<unsigned> snapID;
    helper::vector<unsigned> globalAbove;
    helper::vector<unsigned> globalBelow;
    helper::vector<bool> visited;

    visited.resize(addedPoints.size(),false);
    for (unsigned t=0;t<addedPoints.size();t++) splitPermanentPointAround(t,visited,snapID,globalBelow,globalAbove);

    if (globalBelow.empty() || globalAbove.empty()) return;

//    std::cout << "GLOBAL PERMANENT BELOW = " << globalBelow << std::endl;
//    std::cout << "GLOBAL PERMANENT ABOVE = " << globalAbove << std::endl;

//    rebuildTetra(snapID,globalBelow,globalAbove);
    if (globalBelow.size()<globalAbove.size()) rebuildTetra(snapID,globalBelow);
    else rebuildTetra(snapID,globalAbove);

    propagateModifications();
}

///////////////////////////////////////////////////////
///////////////////////UPDATE STATUS///////////////////
///////////////////////////////////////////////////////

template<class DataTypes>
void TetrahedronCutting<DataTypes>::resetCuts() {
    if (f_resetCut.getValue()==0) return;
    if (nbStep==-1) return; //no new changes

    nbStep++;
    if ((unsigned) nbStep<f_resetCut.getValue()) return;
    nbStep = -1;


    helper::vector<unsigned> searchNodeId;
    for(unsigned p=0;p<addedPoints.size();p++) {
        addSetVector(addedPoints[p].pid,searchNodeId);
        if (addedPoints[p].opp!=-1) addSetVector((unsigned) addedPoints[p].opp,searchNodeId);
    }
    for(unsigned p=0;p<temporaryPoints.size();p++) {
        addSetVector(temporaryPoints[p].a0,searchNodeId);
        addSetVector(temporaryPoints[p].a1,searchNodeId);
    }

    helper::vector<bool> visited;
    visited.resize(searchNodeId.size(),false);

    helper::vector<unsigned> deleteID;
    for(unsigned p=0;p<searchNodeId.size();p++) {
        if (visited[p]) continue;

        helper::vector<unsigned> connected;
        rec_serach(searchNodeId[p],searchNodeId,connected);

        //delete addedPoint indices (only if fullcut)
        helper::vector<unsigned> adddID;
        adddID.resize(connected.size());

        //search if the cut is fully duplicate
        bool fullcut = true;
        for (unsigned i=0;i<connected.size();i++) {
            unsigned connectedID; // search the id of connected[i] in searchNodeId list
            for (connectedID=0;connectedID<searchNodeId.size();connectedID++) {
                if (searchNodeId[connectedID] == connected[i]) break;
            }
            if (connectedID == searchNodeId.size()) {
                serr << "error cannot find the ID in the reset function" << sendl;
                return;
            }

            visited[connectedID] = true;

            if (fullcut) { // if not full cut, we don't nee to compute that because all the point will be keep
                if (!isPermanantAddedPoint(connected[i],adddID[i])) fullcut = false;
                else if (addedPoints[adddID[i]].opp == -1) fullcut = false;
            }
        }

        // add to delete list, all the point which are completely cut
        if (fullcut) {
            for (unsigned i=0;i<adddID.size();i++) addSetVector(adddID[i],deleteID);
        }
    }

    if (deleteID.size()) {
        std::vector<AddedPoint> copy;
        for (unsigned i=0;i<addedPoints.size();i++) copy.push_back(addedPoints[i]);

        addedPoints.clear();
        for (unsigned d=0;d<deleteID.size();d++) {
            if (!contains(d,deleteID)) addedPoints.push_back(copy[d]);
        }
    }
}

/////////////////////////////////////////////////////
/////////////////////DRAW ROUTINE////////////////////
/////////////////////////////////////////////////////

template<class DataTypes>
void TetrahedronCutting<DataTypes>::draw(const core::visual::VisualParams* vparams) {
    if (f_drawActiveEdge.getValue()) {
        const Vec4f c0 = Vec4f(1.0,0.0,1.0,1.0);

        glDisable(GL_LIGHTING);

        vparams->drawTool()->setLightingEnabled(false);
        sofa::core::behavior::MechanicalState<DataTypes> * mstate = dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> *>(this->getContext()->getMechanicalState());
        if (! mstate) return;
        helper::ReadAccessor<Data<VecCoord> > xData = mstate->read(core::ConstVecCoordId::position());


        glPointSize(10);
        glBegin(GL_POINTS);
        for (unsigned j=0;j<xData.size();j++) {
            Coord p0 = xData[j];
            if (canDuplicatePoint(j)) vparams->drawTool()->drawPoint(p0,c0);
        }
        glEnd();
        glPointSize(1);

        glLineWidth(3);
        glBegin(GL_LINES);
        for (int e=0;e<m_container->getNbEdges();e++) {
            const Edge & theEdge = m_container->getEdge(e);

            if (isbreakableEdge(e,theEdge[0],theEdge[1])) {
                Coord p0 = xData[theEdge[0]];
                Coord p1 = xData[theEdge[1]];
                vparams->drawTool()->drawPoint(p0,c0);
                vparams->drawTool()->drawPoint(p1,c0);
            }
        }
        glEnd();
        glLineWidth(1);
    }

    if (f_drawAddedPoints.getValue()) {
        const Vec4f c0 = Vec4f(0.0,0.0,1.0,1.0);
        const Vec4f c1 = Vec4f(0.0,1.0,1.0,1.0);        
        const Vec4f c3 = Vec4f(0.0,1.0,0.0,1.0);
        const Vec4f c4 = Vec4f(1.0,1.0,0.0,1.0);

        vparams->drawTool()->setLightingEnabled(false);
        sofa::core::behavior::MechanicalState<DataTypes> * mstate = dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> *>(this->getContext()->getMechanicalState());
        if (! mstate) return;
        helper::ReadAccessor<Data<VecCoord> > xData = mstate->read(core::ConstVecCoordId::position());

        glPointSize(10);
        glBegin(GL_POINTS);

        for (unsigned i=0;i<addedPoints.size();i++) {
            Coord p0 = xData[addedPoints[i].pid];
            if (addedPoints[i].opp==-1) vparams->drawTool()->drawPoint(p0,c4);
            else {
                Coord p1 = xData[addedPoints[i].opp];
                vparams->drawTool()->drawPoint(p0,c3);
                vparams->drawTool()->drawPoint(p1,c3);
            }
        }

        for (unsigned i=0;i<temporaryPoints.size();i++) {
            Coord p0 = xData[temporaryPoints[i].a0];
            Coord p1 = xData[temporaryPoints[i].a1];
            Coord px = p0 + ((p1-p0) * temporaryPoints[i].alpha);

            if (temporaryPoints[i].alpha==0.0) vparams->drawTool()->drawPoint(px,c0);
            else vparams->drawTool()->drawPoint(px,c1);
        }

        glEnd();
        glPointSize(1);
    }
}


} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENTS_TETEAHEDRONSETTOPOLOGYALGORITHMS_INL
