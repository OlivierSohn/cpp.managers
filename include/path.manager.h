#pragma once

#include <list>
#include <string>
#include <map>

#include "referentiable.manager.h"

#include "path.suite.h"

#include "cg.math.acc.record.h"
#include "motion/curve.discrete.integrated.h"
#include "motion/curve.continuous.regularized.h"

namespace imajuscule
{
    // when first created, if a path is passed to the PathManager it becomes the current directory
    template <>
    class ReferentiableManager<PathSuite> : public ReferentiableManagerBase
    {
    public:
        static ReferentiableManager * getInstance();

        const char * defaultNameHint()override;
        const char * UIName()override;
        
        unsigned int index()override;
    private:
        static ReferentiableManager * g_pRefManager;

        ReferentiableManager();
        virtual ~ReferentiableManager();

        Referentiable* newReferentiableInternal(const std::string & nameHint, const std::vector<std::string> & guids, bool bVisible, bool bFinalize) override;

    public:
        virtual PathSuite* newPath(const std::string & nameHint, 
            const std::vector<std::string> & guids, 
            double freqCutoff = 1.0,
            bool adaptive = false,
            integratedPath::IntegrationMode intmode = integratedPath::TRAPEZOIDAL);
        virtual PathSuite* newPathVariant(PathSuite* finalizedSuite, const std::string & nameHint, double freqCutoff, bool adaptive, integratedPath::IntegrationMode intmode, const std::vector<std::string> & guids);

        virtual PathSuite* newPathByCompression(PathSuite* finalizedSuite, const std::string & nameHint, const std::vector<std::string> & guids);

    private:
        // guid - path
        typedef std::map<std::string, integratedPath*> integratedPaths;
        typedef std::map<std::string, regularizedPath*> regularizedPaths;
        typedef std::map<std::string, rawPath*> rawPaths;
        // guid of elementary path - nRefs
        typedef std::map < std::string, int > refMap;
        
        std::vector<PathSuite*> path_suites;

        int addRef(const std::string & guid);
        int removeRef(const std::string & guid);

        refMap m_refMap;
        integratedPaths m_integratedPaths;
        regularizedPaths m_regularizedPaths;
        rawPaths m_rawPaths;

        void LoadPaths();

        void LoadRawPaths();
        void LoadIntegratedPaths();
        void LoadRegularizedPaths();
        void LoadPathSuites();
    };
}
