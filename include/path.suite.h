#pragma once


#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "motion/curve.discrete.integrated.h"
#include "cg.math.acc.record.h"
#include "motion/curve.continuous.regularized.h"
#include "motion/constraint.translation.h"
#include <string>

#include "referentiable.h"

namespace imajuscule
{
    class CurveMotion;
    class ReferentiableManagerBase;
    class PathSuite : public Referentiable
    {
        protected:
        virtual ~PathSuite();

    public:
        PathSuite(ReferentiableManagerBase* pm, const std::string & guid, const std::string & nameHint);
        PathSuite(ReferentiableManagerBase* pm, const std::string & name, const std::string & guid, rawPath * rawPath, integratedPath* intPath, regularizedPath* regPath);

        PathError Record(bool bUseTranslationConstraint = true);
        PathError SetTransforms(const glm::dquat & rotDeviceToCamera, const glm::dquat & rotAttitude);
        PathError SetTransforms(double rotDeviceToCamera[4], double rotAttitude[4]);
        PathError SetTransformsAsMatrix(double rotDeviceToCamera[9], double rotAttitude[9]);

        /*void AddAccelerationPos(accelerationData & sensordata);
        void AddAccelerationRot(accelerationData & sensordata);
        void AddRot(RotationData & rotdata);
        */
        void AddRotAndAccelerationPos(RotationData &rotdata, accelerationData & accPosData);
        void AddRotAndAccelerationPosAsMatrix(double rot[9], double timeRot, double pos[3], double timePos);
        void AddRotAndAccelerationPosAsQuatArr(double quat[4], double timeRot, double pos[3], double timePos);

        PathError FinalizeRecord();

        PathError CountFrames(int & nFrames);

        PathError Play();
        void UnPlay();

        ///////////////////////////////
        // persistence-related methods
        ///////////////////////////////

        // save
        PathError SaveToFile();
        PathError RemoveFilesRecurse();
        
        // load
        PathError LoadFromFile(std::string & sRawPathGuid, std::string & sIntPathGuid, std::string & sRegPathGuid);
        void SetPaths(rawPath * rawPath, integratedPath * intPath, regularizedPath * regPath);
        void Initialize();

        VISITOR_HEADER_IMPL

    private:
        glm::dquat m_rotWorldToDevRefFrame;
        glm::dquat m_rotDeviceToCamera;
        glm::dquat m_rotCameraToDevice;
        rawPath* m_rawPath;
        integratedPath* m_integratedPath;
        regularizedPath* m_regularizedPath;
        ref_unique_ptr<CurveMotion> m_curveMotion;
        ref_unique_ptr<CurveMotion> m_discreteCurveMotion;

        // unpersisted for now : result of constraint is applied in ipData.
        // in the future it will be persisted, result of constraint will not be applied in ipData (constraint will be applied each time)
        // and the constraint will be made editable
        std::vector<std::unique_ptr<TranslationConstraint>> m_constraints;

        void ClearConstraints();

        PathError ComputeInitialPositionAndRotation();
        PathError Finalize(unsigned int fps);

        class PathSuitePersist : public ReferentiablePersist
        {
        public:
            PathSuitePersist(PathSuite & pPathSuite);

            eResult doSave() override;

        private:
            PathSuite & m_pPathSuite;
        };

        class PathSuiteLoad : public ReferentiableLoad
        {
        public:
            PathSuiteLoad(PathSuite & pPathSuite);

            eResult Load(std::string & sRawPathGuid, std::string & sIntPathGuid, std::string & sRegPathGuid);

        protected:
            void LoadStringForKey(char key, std::string & str) override;
            virtual void LoadBoolForKey(char key, bool bVal) override{
                LG(ERR, "PathSuiteLoad::LoadBoolForKey(%d, %d) should not be called", key, bVal);
            }
            virtual void LoadDoubleArrayForKey(char key, double * pdVal, size_t nElems) override{
                LG(ERR, "PathSuiteLoad::LoadDoubleArrayForKey(%d, ..., %d) should not be called", key, nElems);
            }
            virtual void LoadDoubleForKey(char key, double fVal) override{
                LG(ERR, "PathSuiteLoad::LoadDoubleForKey(%d, %f) should not be called", key, fVal);
            }
            virtual void LoadCharArrayForKey(char key, char * /*pcVal*/, size_t nElems)override {
                LG(ERR, "PathSuiteLoad::LoadCharArrayForKey(%d, ..., %d) should not be called", key, nElems);
            }
            virtual void LoadInt32ArrayForKey(char key, int32_t * /*piVal*/, size_t nElems)override {
                LG(ERR, "PathSuiteLoad::LoadInt32ArrayForKey(%d, ..., %d) should not be called", key, nElems);
            }
            virtual void LoadFloatArrayForKey(char key, float * pfVal, size_t nElems) override{
                LG(ERR, "PathSuiteLoad::LoadFloatArrayForKey(%d, ..., %d) should not be called", key, nElems);
            }
            virtual void LoadCharForKey(char key, char cVal) override{
                LG(ERR, "PathSuiteLoad::LoadCharForKey(%d, %d) should not be called", key, cVal);
            }
            virtual void LoadFloatForKey(char key, float fVal) override{
                LG(ERR, "PathSuiteLoad::LoadFloatForKey(%d, %f) should not be called", key, fVal);
            }

        private:
            PathSuite & m_pPathSuite;
            std::string m_rawPathGuid;
            std::string m_intPathGuid;
            std::string m_regPathGuid;
        };
    };
}
