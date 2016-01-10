#include "path.suite.h"
#include "path.manager.h"
#include "gl.model.h"
#include "gl.view.world.h"
#include "cg.math.acc.record.h"
#include "motion/curve.continuous.regularized.h"
#include "motion/curve.discrete.integrated.h"
#include "motion/layer.curve.discrete.h"
#include "motion/layer.curve.continuous.h"
#include "os.log.h"
#include "os.log.format.h"
#include <cstring> // memcpy

#include "motion/constraint.translation.onSphereLookAtCenter.h"


#define KEY_RAWPATH_GUID        'f' // string
#define KEY_INTPATH_GUID        'g' // string
#define KEY_REGPATH_GUID        'h' // string


using namespace imajuscule;

PathSuite::~PathSuite()
{
    if( m_curveMotion )
        m_curveMotion->deinstantiate();
    m_curveMotion = NULL;
    
    if( m_discreteCurveMotion )
        m_discreteCurveMotion->deinstantiate();
    m_discreteCurveMotion = NULL;

    ClearConstraints();
}

void PathSuite::ClearConstraints()
{
    std::vector<TranslationConstraint*>::iterator it = m_constraints.begin();
    std::vector<TranslationConstraint*>::iterator end = m_constraints.end();
    for (; it != end; ++it)
    {
        delete *it;
    }

    m_constraints.clear();
}

PathSuite::PathSuite(ReferentiableManagerBase * pm, const std::string & name, const std::string & guid, rawPath* rawPath, integratedPath* intPath, regularizedPath* regPath) :
Referentiable(pm, guid,name),
m_rawPath(rawPath),
m_integratedPath(intPath),
m_regularizedPath(regPath),
m_curveMotion(NULL),
m_discreteCurveMotion(NULL)
{
    Initialize();
}

void PathSuite::Initialize()
{
    m_discreteCurveMotion = DiscreteCurveMotion::instantiate(CurveMotion::POSITION_AND_ROTATION, m_integratedPath->lastValueTraversal(), m_integratedPath);
}

PathSuite::PathSuite(ReferentiableManagerBase * pm, const std::string & guid, const std::string & nameHint) :
Referentiable(pm, guid, nameHint),
m_rawPath(NULL),
m_integratedPath(NULL),
m_regularizedPath(NULL),
m_curveMotion(NULL),
m_discreteCurveMotion(NULL)
{
}

/*void PathSuite::AddAccelerationPos(accelerationData & sensordata)
{
    m_integratedPath->addAccelerationPos(sensordata);
}

void PathSuite::AddAccelerationRot(accelerationData & sensordata)
{
    m_integratedPath->addAccelerationRot(sensordata);
}*/

PathError PathSuite::Finalize(unsigned int fps)
{
    PathError ret = PE_SUCCESS;
    if( !m_integratedPath )
    {
        ret = PE_NOT_FOUND;
    }
    else
    {
        m_integratedPath->finalize();
        ret = m_regularizedPath->initializeForFps(fps);
        if (ret == PE_SUCCESS)
        {
            ret = m_rawPath->finalize();
            if (ret == PE_SUCCESS)
            {
                m_curveMotion = ContinuousCurveMotion::instantiate(CurveMotion::POSITION_AND_ROTATION, m_regularizedPath->traversal(), m_regularizedPath);
            }
        }
    }
    
    return ret;
}

/*void PathSuite::AddRot(RotationData &rotdata)
{
    m_integratedPath->addRot(rotdata);
}*/


void PathSuite::AddRotAndAccelerationPosAsQuatArr(double quat[4], double timeRot, double pos[3], double timePos)
{
    RotationData rotData;
    rotData.time = timeRot;
    rotData.rotation = glm::dquat(quat[0], quat[1], quat[2], quat[3]);

    accelerationData accData;
    accData.time = timePos;
    memcpy(accData.acc, pos, sizeof(accData.acc));

    AddRotAndAccelerationPos(rotData, accData);
}


void PathSuite::AddRotAndAccelerationPosAsMatrix(double rot[9], double timeRot, double pos[3], double timePos)
{
    RotationData rotData;
    rotData.time = timeRot;
    rotData.rotation = glm::dquat(glm::dmat3x3(rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]));

    accelerationData accData;
    accData.time = timePos;
    memcpy(accData.acc, pos, sizeof(accData.acc));

    AddRotAndAccelerationPos(rotData, accData);
}

void PathSuite::AddRotAndAccelerationPos(RotationData &rotData, accelerationData & accPosData)
{
    glm::dquat tempRot = glm::normalize( m_rotWorldToDevRefFrame * rotData.rotation * m_rotDeviceToCamera);
    // convert rotation
    rotData.rotation = glm::normalize( glm::inverse(tempRot));
    
    /*
    // convert acceleration
    std::cout << "pathsuite acc : ";
    std::cout << accPosData.acc[0] << " ";
    std::cout << accPosData.acc[1] << " ";
    std::cout << accPosData.acc[2] << " -> ";
    */
    
    // accelerations are in device local coordinates, put them in camera global coordinates
    
    glm::dvec3 accDevLocal(accPosData.acc[0], accPosData.acc[1], accPosData.acc[2]);
    
    glm::dvec3 accCamLocal = accDevLocal * m_rotDeviceToCamera;

    /*accPosData.acc[0] = accCamLocal[0];
    accPosData.acc[1] = accCamLocal[1];
    accPosData.acc[2] = accCamLocal[2];
    
    std::cout << accPosData.acc[0] << " ";
    std::cout << accPosData.acc[1] << " ";
    std::cout << accPosData.acc[2] << " -> ";
    */
    
    glm::dvec3 accGlobal = accCamLocal * rotData.rotation;
    
    accPosData.acc[0] = accGlobal[0];
    accPosData.acc[1] = accGlobal[1];
    accPosData.acc[2] = accGlobal[2];
    /*
    std::cout << accPosData.acc[0] << " ";
    std::cout << accPosData.acc[1] << " ";
    std::cout << accPosData.acc[2] << std::endl;
*/

    if (m_constraints.empty())
        m_integratedPath->addRotAndAccelerationPos(rotData, accPosData);
    else
    {
        double constrainedPos[3];
     
        glm::vec3 pos;

        std::vector<TranslationConstraint*>::iterator it = m_constraints.begin();
        std::vector<TranslationConstraint*>::iterator end = m_constraints.end();
        for (; it != end; ++it)
        {
            (*it)->apply(glm::quat(rotData.rotation), pos);
        }

        constrainedPos[0] = pos[0];
        constrainedPos[1] = pos[1];
        constrainedPos[2] = pos[2];
        m_integratedPath->addRotAndPos(rotData, constrainedPos);
    }
}

PathError PathSuite::ComputeInitialPositionAndRotation()
{
    // the translations are expressed in unitized units
    Transformation const & t = WorldView::gCamera().transformation();
    const glm::vec3 & vpos = t.getTranslation();

    double pos[3] {vpos[0], vpos[1], vpos[2]};
    
    const glm::quat & quat = t.getRotation();
    
    glm::dquat doubleQuat = glm::dquat(quat);
    
    return m_rawPath->SetInitialConditions(pos, doubleQuat);
}

PathError PathSuite::Play()
{
    PathError ret = PE_SUCCESS;
    if (m_curveMotion)
    {
        if( auto wv = WorldView::hasInstance() ) {
            wv->editPlayer().setCurves(m_curveMotion, NULL);
            wv->usePlayer(true);
            WorldView::gCamera().localSpaceMotion()->StopInTime(0.f);
        } else {
            ret = PE_NOT_FOUND;
        }
    }
    else
    {
        ret = PE_NOT_FINALIZED;
    }
    
    return ret;
}

PathError PathSuite::Record(bool bUseTranslationConstraint)
{
    //LG(INFO, "PathSuite::Record(%s)", bUseTranslationConstraint?"true":"false");

    PathError ret = PE_SUCCESS;
    if (m_discreteCurveMotion)
    {
        ret = ComputeInitialPositionAndRotation();
        if( ret == PE_SUCCESS )
        {
            if( auto wv = WorldView::hasInstance() ) {
                wv->editPlayer().setCurves(m_discreteCurveMotion, NULL);
                wv->usePlayer(true);
                WorldView::gCamera().localSpaceMotion()->StopInTime(0.f);
            } else {
                LG(WARN, "PathSuite::Record : Initialize WorldView before call if you want the player to play the path in real time");
            }

            ClearConstraints();

            if (bUseTranslationConstraint)
            {
                glm::vec3 center(0.f, 0.f, 0.f);
                TranslationOnSphere * pCstr = new TranslationOnSphere(center, 100.f);
                m_constraints.push_back(pCstr);
            }
        }
    }
    else
    {
        ret = PE_NOT_FOUND;
    }
    
    //LG(INFO, "PathSuite::Record returns %d", ret);
    return ret;
}

PathError PathSuite::SetTransformsAsMatrix(double rotDeviceToCamera[9], double rotAttitude[9])
{
    glm::dquat qRotDeviceToCamera(glm::dmat3x3(
        rotDeviceToCamera[0], rotDeviceToCamera[1], rotDeviceToCamera[2],
        rotDeviceToCamera[3], rotDeviceToCamera[4], rotDeviceToCamera[5],
        rotDeviceToCamera[6], rotDeviceToCamera[7], rotDeviceToCamera[8]));
    glm::dquat qRotAttitude(glm::dmat3x3(
        rotAttitude[0], rotAttitude[1], rotAttitude[2],
        rotAttitude[3], rotAttitude[4], rotAttitude[5],
        rotAttitude[6], rotAttitude[7], rotAttitude[8]));
    return SetTransforms(qRotDeviceToCamera, qRotAttitude);
}
PathError PathSuite::SetTransforms(double rotDeviceToCamera[4], double rotAttitude[4])
{
    glm::dquat qRotDeviceToCamera(rotDeviceToCamera[0], rotDeviceToCamera[1], rotDeviceToCamera[2], rotDeviceToCamera[3]);
    glm::dquat qRotAttitude(rotAttitude[0], rotAttitude[1], rotAttitude[2], rotAttitude[3]);
    return SetTransforms(qRotDeviceToCamera, qRotAttitude);
}
PathError PathSuite::SetTransforms(const glm::dquat & rotDeviceToCamera, const glm::dquat & rotAttitude)
{
    PathError ret = PE_SUCCESS;
    
    // these quats are needed to put the acceleration in the camera frame
    m_rotDeviceToCamera = rotDeviceToCamera;
    m_rotCameraToDevice = glm::inverse(rotDeviceToCamera);

    double fake[3];
    glm::dquat gCam0;
    if( m_rawPath )
        ret = m_rawPath->GetInitialConditions(fake,gCam0);
    else
        ret = PE_NOT_FOUND;
    
    if( PE_SUCCESS == ret )
    {
        m_rotWorldToDevRefFrame = glm::normalize( glm::inverse(gCam0) * glm::inverse(rotAttitude) );
    }

    return ret;
}

void PathSuite::UnPlay()
{
    if( auto wv = WorldView::hasInstance() ) {
        wv->usePlayer(false);
        wv->editPlayer().setCurves(NULL, NULL);
    } else {
        LG(WARN, "PathSuite::UnPlay : Initialize WorldView before call if you want the player to play the path in real time");
    }
}

PathError PathSuite::FinalizeRecord()
{
    UnPlay();
    
    return Finalize(OGLGetTimeResolution());
}

PathError PathSuite::CountFrames(int & nFrames)
{
    if( !m_regularizedPath )
    {
        return PE_NOT_FOUND;
    }
    
    {
        PositionTraversal * t = m_regularizedPath->traversal();
        if( !t )
        {
            return PE_NOT_FINALIZED;
        }
        
        nFrames = t->countUniqueValues();
    }
    
    return PE_SUCCESS;
}

PathError PathSuite::SaveToFile()
{
    //LG(INFO, "PathSuite::SaveToFile begin");

    if (m_rawPath) {
        auto res = m_rawPath->SaveToFile();
        if( res != PE_SUCCESS ) {
            return PE_NOT_FOUND;
        }
    }

    if (m_integratedPath) {
        auto res = m_integratedPath->SaveToFile();
        if( res != PE_SUCCESS ) {
            return PE_NOT_FOUND;
        }
    }

    if (m_regularizedPath) {
        auto res = m_regularizedPath->SaveToFile();
        if( res != PE_SUCCESS ) {
            return PE_NOT_FOUND;
        }
    }

    PathSuitePersist pss(*this);
    eResult eRes = pss.Save();
    if (eRes != ILE_SUCCESS)
    {
        return PE_NOT_FOUND;
    }

    //LG(INFO, "PathSuite::SaveToFile returns %d", ret);
    return PE_SUCCESS;
}

PathError PathSuite::LoadFromFile(std::string & sRawPathGuid, std::string & sIntPathGuid, std::string & sRegPathGuid)
{
    //LG(INFO, "PathSuite::LoadFromFile begin");
    PathError ret = PE_SUCCESS;

    PathSuiteLoad loader(*this);
    eResult res = loader.Load(sRawPathGuid, sIntPathGuid, sRegPathGuid);
    if (res != ILE_SUCCESS)
    {
        ret = PE_NOT_FOUND;
    }

    //LG(INFO, "PathSuite::LoadFromFile returns %d", ret);
    return ret;
}

PathSuite::PathSuitePersist::PathSuitePersist(PathSuite & pPathSuite) :
ReferentiablePersist( {PATH_SUB_FOLDER,PATHSUITE_PATH}, pPathSuite.guid(), pPathSuite),
m_pPathSuite(pPathSuite)
{
    //LG(INFO, "PathSuitePersist::PathSuitePersist begin");
    
    //todo

    //LG(INFO, "PathSuitePersist::PathSuitePersist end");
}

PathSuite::PathSuitePersist::~PathSuitePersist()
{
    //LG(INFO, "PathSuitePersist::~PathSuitePersist");
}

eResult PathSuite::PathSuitePersist::doSave()
{
    if (m_pPathSuite.m_rawPath)
    {
        std::string sRawPathGuid;
        m_pPathSuite.m_rawPath->getGUID(sRawPathGuid);
        WriteKeyData(KEY_RAWPATH_GUID, sRawPathGuid);
    }
    else
    {
        LG(WARN, "PathSuitePersist::doSave : rawPath is NULL");
    }

    if (m_pPathSuite.m_integratedPath)
    {
        std::string sGuid;
        m_pPathSuite.m_integratedPath->getGUID(sGuid);
        WriteKeyData(KEY_INTPATH_GUID, sGuid);
    }
    else
    {
        LG(WARN, "PathSuitePersist::doSave : intPath is NULL");
    }

    if (m_pPathSuite.m_regularizedPath)
    {
        std::string sGuid;
        m_pPathSuite.m_regularizedPath->getGUID(sGuid);
        WriteKeyData(KEY_REGPATH_GUID, sGuid);
    }
    else
    {
        LG(WARN, "PathSuitePersist::doSave : regPath is NULL");
    }

    return ReferentiablePersist::doSave();
}

void PathSuite::PathSuiteLoad::LoadStringForKey(char key, std::string & sVal)
{
    //LG(INFO, "PathSuiteLoad::LoadStringForKey(%d, %s) begin", key, (sVal.c_str() ? sVal.c_str():"NULL"));

    switch (key)
    {
    case KEY_RAWPATH_GUID:
        m_rawPathGuid = sVal;
        break;

    case KEY_INTPATH_GUID:
        m_intPathGuid = sVal;
        break;

    case KEY_REGPATH_GUID:
        m_regPathGuid = sVal;
        break;

    default:
        ReferentiableLoad::LoadStringForKey(key, sVal);
        break;
    }

    //LG(INFO, "PathSuiteLoad::LoadStringForKey(%d, %s) end", key, (sVal.c_str() ? sVal.c_str() : "NULL"));
}

PathSuite::PathSuiteLoad::PathSuiteLoad(PathSuite&pathSuite) :
ReferentiableLoad({PATH_SUB_FOLDER,PATHSUITE_PATH}, pathSuite.guid(), pathSuite),
m_pPathSuite(pathSuite)
{}

PathSuite::PathSuiteLoad::~PathSuiteLoad()
{
}

eResult PathSuite::PathSuiteLoad::Load(std::string & sRawPathGuid, std::string & sIntPathGuid, std::string & sRegPathGuid)
{
    //LG(INFO, "PathSuiteLoad::Load begin");

    eResult ret = ReadAllKeys();
    if( unlikely(ret != ILE_SUCCESS))
        goto end;

    sRawPathGuid = m_rawPathGuid;
    sIntPathGuid = m_intPathGuid;
    sRegPathGuid = m_regPathGuid;

    if( unlikely(sRawPathGuid.empty()))
    {
        LG(ERR, "PathSuiteLoad::Load : rawPathGuid not found");
        ret = ILE_OBJECT_INVALID;
    }

    if( unlikely(sIntPathGuid.empty()))
    {
        LG(ERR, "PathSuiteLoad::Load : intPathGuid not found");
        ret = ILE_OBJECT_INVALID;
    }

    if( unlikely(sRegPathGuid.empty()))
    {
        LG(ERR, "PathSuiteLoad::Load : regPathGuid not found");
        ret = ILE_OBJECT_INVALID;
    }
end:
    //LG(INFO, "PathSuiteLoad::Load returns %d", ret);
    return ret;
}

void PathSuite::SetPaths(rawPath * rawPath, integratedPath * intPath, regularizedPath * regPath)
{
    m_rawPath = rawPath;
    m_integratedPath = intPath;
    m_regularizedPath = regPath;
}
