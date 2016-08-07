#include <algorithm>

#include "os.storage.h"
#include "os.log.h"
#include "path.manager.h"
#include "cg.math.acc.record.h"
#include "motion/curve.continuous.regularized.h"
#include "motion/curve.discrete.integrated.h"

namespace imajuscule {

ReferentiableManager<PathSuite>::ReferentiableManager():
ReferentiableManagerBase()
{
    // TODO ReferentiableManager : genericize LoadRefs
    LoadPaths();
}


ReferentiableManager<PathSuite>::~ReferentiableManager()
{
    {
        integratedPaths::iterator it = m_integratedPaths.begin();
        for (; it != m_integratedPaths.end(); ++it)
        {
            delete (it->second);
        }
    }

    {
        regularizedPaths::iterator it = m_regularizedPaths.begin();
        for (; it != m_regularizedPaths.end(); ++it)
        {
            delete (it->second);
        }
    }

    {
        rawPaths::iterator it = m_rawPaths.begin();
        for (; it != m_rawPaths.end(); ++it)
        {
            delete (it->second);
        }
    }
}

PathSuite* ReferentiableManager<PathSuite>::newPath(const std::string & nameHint, const std::vector<std::string> & guids, double freqCutoff, bool adaptive, integratedPath::IntegrationMode intmode)
{
    /*LG(INFO, "ReferentiableManager<PathSuite>::newPath(%s, %d guids, %f, %s, %d) begin",
        (nameHint.c_str() ? nameHint.c_str() : "NULL"),
        guids.size(),
        freqCutoff,
        (adaptive ? "true" : "false"),
        intmode);
        */
    PathSuite * curSuite = NULL;
    rawPath * newRawPath = NULL;
    integratedPath * newIntPath = NULL;
    regularizedPath * newRegPath = NULL;

    std::string guid, guid1, guid2, guid3;
    
    int sizeGuids = (int)guids.size();

    if (sizeGuids > 0)
    {
        guid.assign(guids[0]);
    }
    else
    {
        guid = generateGuid();
    }

    if (sizeGuids > 1)
    {
        guid1.assign(guids[1]);
    }
    else
    {
        guid1 = generateGuid();
    }

    if (sizeGuids > 2)
    {
        guid2.assign(guids[2]);
    }
    else
    {
        guid2 = generateGuid();
    }

    if (sizeGuids > 3)
    {
        guid3.assign(guids[3]);
    }
    else
    {
        guid3 = generateGuid();
    }
    
    newRawPath = new rawPath(guid1);
    {
        std::pair<rawPaths::iterator, bool> resInsert = m_rawPaths.insert(rawPaths::value_type(guid1, newRawPath));
        if ( unlikely(!resInsert.second))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::newPath : m_rawPaths.insert failed (uuid: %s)", guid1.c_str());
            goto end;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::newPath : m_rawPaths.insert success (uuid: %s)", guid1.c_str());
        }
    }

    newIntPath = new integratedPath(guid2, newRawPath, freqCutoff, adaptive, intmode);
    {
        std::pair<integratedPaths::iterator, bool> resInsert = m_integratedPaths.insert(integratedPaths::value_type(guid2, newIntPath));
        if ( unlikely(!resInsert.second))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::newPath : m_integratedPaths.insert failed (uuid: %s)", guid2.c_str());
            goto end;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::newPath : m_integratedPaths.insert success (uuid: %s)", guid2.c_str());
        }
    }
    
    newRegPath = new regularizedPath(guid3, newIntPath);
    {
        std::pair<regularizedPaths::iterator, bool> resInsert = m_regularizedPaths.insert(regularizedPaths::value_type(guid3, newRegPath));
        if ( unlikely(!resInsert.second))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::newPath : m_regularizedPaths.insert failed (uuid: %s)", guid3.c_str());
            goto end;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::newPath : m_regularizedPaths.insert success (uuid: %s)", guid3.c_str());
        }
    }

    curSuite = new PathSuite(this, nameHint, guid, newRawPath, newIntPath, newRegPath);
    if ( unlikely(!ComputeSessionName(curSuite, true)))
    {
        LG(ERR, "ReferentiableManager<PathSuite>::newPath : ComputeSessionName failed (uuid: %s)", guid.c_str());
        delete curSuite;
        curSuite = NULL;
        goto end;
    }

end:
    if (!curSuite)
    {
        if (newRawPath)
            delete newRawPath;
        if (newIntPath)
            delete newIntPath;
        if (newRegPath)
            delete newRegPath;
    }

    LG((curSuite ? INFO : ERR), "ReferentiableManager<PathSuite>::newPath(...) returns 0x%x", curSuite);
    return curSuite;
}

PathSuite* ReferentiableManager<PathSuite>::newPathVariant(PathSuite* finalizedSuite, const std::string & nameHint, double freqCutoff, bool adaptive, integratedPath::IntegrationMode intmode, const std::vector<std::string> & guids)
{
    A(0);//todo
    return NULL;
}

PathSuite* ReferentiableManager<PathSuite>::newPathByCompression(PathSuite* finalizedSuite, const std::string & nameHint, const std::vector<std::string> & guids)
{
    A(0);//todo
    return NULL;
}

void ReferentiableManager<PathSuite>::LoadPaths()
{
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadPaths begin");

    LoadRawPaths();
    LoadIntegratedPaths();
    LoadRegularizedPaths();
    LoadPathSuites();

    //LG(INFO, "ReferentiableManager<PathSuite>::LoadPaths end");
}

void ReferentiableManager<PathSuite>::LoadPathSuites()
{
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadPathSuites begin");

    std::string filePath;

    filePath.append("./");
    filePath.append(PATH_SUB_FOLDER);
    filePath.append("/");
    filePath.append(PATHSUITE_PATH);
    filePath.append("/");

    for ( auto const & guid : StorageStuff::listFilenames(filePath) )
    {
        PathSuite * ps = new PathSuite(this, guid, std::string("path"));
        
        std::string Raw, Int, Reg;
        PathError ret = ps->LoadFromFile(Raw, Int, Reg);
        if ( unlikely(ret != PE_SUCCESS))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : ps->LoadFromFile failed (uuid: %s, err: %d)", guid.c_str(), ret);
            delete ps;
            continue;
        }

        rawPaths::iterator rawIt = m_rawPaths.find(rawPaths::key_type(Raw));
        if ( unlikely(rawIt == m_rawPaths.end()))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : raw path not found (uuid : %s)", Raw.c_str());
            delete ps;
            continue;
        }

        integratedPaths::iterator intIt = m_integratedPaths.find(integratedPaths::key_type(Int));
        if ( unlikely(intIt == m_integratedPaths.end()))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : int path not found (uuid : %s)", Int.c_str());
            delete ps;
            continue;
        }

        regularizedPaths::iterator regIt = m_regularizedPaths.find(regularizedPaths::key_type(Reg));
        if ( unlikely(regIt == m_regularizedPaths.end()))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : reg path not found (uuid : %s)", Reg.c_str());
            delete ps;
            continue;
        }

        rawPath * rawP = rawIt->second;
        integratedPath * intP = intIt->second;
        regularizedPath * regP = regIt->second;

        if ( unlikely(!rawP))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : raw path NULL");
            delete ps;
            continue;
        }

        if ( unlikely(!intP))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : int path NULL");
            delete ps;
            continue;
        }

        if ( unlikely(!regP))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : reg path NULL");
            delete ps;
            continue;
        }

        ps->SetPaths(rawP, intP, regP);

        ps->Initialize();

        ret = ps->FinalizeRecord();
        if ( unlikely(ret != PE_SUCCESS))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : ps->FinalizeRecord failed (uuid: %s, err: %d)", guid.c_str(), ret);
            delete ps;
            continue;
        }
        
        if( unlikely(!ComputeSessionName(ps, true)) )
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : ComputeSessionName failed (uuid: %s)", guid.c_str());
            delete ps;
            continue;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadPathSuites : ComputeSessionName success (uuid: %s)", guid.c_str());
        }
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::LoadPathSuites end");
}

int ReferentiableManager<PathSuite>::addRef(const std::string & guid)
{
    //LG(INFO, "ReferentiableManager<PathSuite>::addRef(%s)", (guid.c_str() ? guid.c_str():"NULL"));

    //  try to insert with refCount == 1
    //  if insertion fails
    //      increment existing argument's refCount 

    std::pair<refMap::iterator,bool> res = m_refMap.insert(refMap::value_type(guid, 1));
    if (!res.second)
    {
        //LG(INFO, "ReferentiableManager<PathSuite>::addRef : found existing element");
        res.first->second++;
    }
    else
    {
        //LG(INFO, "ReferentiableManager<PathSuite>::addRef : new element inserted");
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::addRef(%s) returns %d", (guid.c_str() ? guid.c_str() : "NULL"), res.first->second);
    return res.first->second;
}

int ReferentiableManager<PathSuite>::removeRef(const std::string & guid)
{
    //LG(INFO, "ReferentiableManager<PathSuite>::removeRef(%s)", (guid.c_str() ? guid.c_str() : "NULL"));

    int res = 0;

    // if element exists and 
    refMap::iterator it = m_refMap.find(guid);
    if ( likely(it != m_refMap.end()))
    {
        //LG(INFO, "ReferentiableManager<PathSuite>::removeRef : found existing element");
        it->second--;

        res = it->second;

        if (res <= 0)
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::removeRef : no element references this element, remove it from map");
            m_refMap.erase(it);
        }
    }
    else
    {
        LG(ERR, "ReferentiableManager<PathSuite>::removeRef : element not found");
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::removeRef(%s) returns %d", (guid.c_str() ? guid.c_str() : "NULL"), res);
    return res;
}

void ReferentiableManager<PathSuite>::LoadRawPaths()
{
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadRawPaths begin");

    std::string filePath;

    filePath.append("./");
    filePath.append(PATH_SUB_FOLDER);
    filePath.append("/");
    filePath.append(RAW_PATH_SUB_FOLDER);
    filePath.append("/");
    
    for ( auto const & guid : StorageStuff::listFilenames(filePath) )
    {
        rawPath * ps = new rawPath(guid);
       
        PathError err = ps->LoadFromFile();
        if ( unlikely(err != PE_SUCCESS))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadRawPaths : ps->LoadFromFile() failed (%d)", err);
            continue;
        }

        std::pair<rawPaths::iterator, bool> res = m_rawPaths.insert(rawPaths::value_type(guid, ps));
        if ( unlikely(!res.second))
        {
            LG(WARN, "ReferentiableManager<PathSuite>::LoadRawPaths : m_rawPaths.insert failed (uuid: %s)", guid.c_str());
            delete ps;
            continue;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadRawPaths : m_rawPaths.insert success (uuid: %s)", guid.c_str());
        }
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::LoadRawPaths end");
}

void ReferentiableManager<PathSuite>::LoadIntegratedPaths()
{
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadIntegratedPaths begin");

    std::string filePath;

    filePath.append("./");
    filePath.append(PATH_SUB_FOLDER);
    filePath.append("/");
    filePath.append(INT_PATH_SUB_FOLDER);
    filePath.append("/");

    for ( auto const & guid : StorageStuff::listFilenames(filePath) )
    {
        integratedPath * ps = new integratedPath(guid);
        std::string sRawPathGUID;
        
        PathError err = ps->LoadFromFile(sRawPathGUID);
        if ( unlikely(err != PE_SUCCESS))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : ps->LoadFromFile() failed (%d)", err);
            continue;
        }

        rawPaths::iterator it2 = m_rawPaths.find(rawPaths::key_type(sRawPathGUID));
        if ( likely(it2 != m_rawPaths.end()))
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath found (uuid: %s)", sRawPathGUID.c_str());
            if ( likely(it2->second))
                ps->SetRawPath(it2->second);
            else
            {
                LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath is NULL (uuid: %s)", sRawPathGUID.c_str());
                delete ps;
                continue;
            }
        }
        else
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath not found (uuid: %s)", sRawPathGUID.c_str());
            delete ps;
            continue;
        }

        std::pair<integratedPaths::iterator, bool> res = m_integratedPaths.insert(integratedPaths::value_type(guid, ps));
        if (!res.second)
        {
            LG(WARN, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : m_integratedPaths.insert failed (uuid: %s)", guid.c_str());
            delete ps;
            continue;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : m_integratedPaths.insert success (uuid: %s)", guid.c_str());
        }
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::LoadIntegratedPaths end");
}

void ReferentiableManager<PathSuite>::LoadRegularizedPaths()
{
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadRegularizedPaths begin");

    std::string filePath;

    filePath.append("./");
    filePath.append(PATH_SUB_FOLDER);
    filePath.append("/");
    filePath.append(REG_PATH_SUB_FOLDER);
    filePath.append("/");

    for ( auto const & guid : StorageStuff::listFilenames(filePath) )
    {
        regularizedPath * ps = new regularizedPath(guid);
        std::string sIntegratedPathGUID;
        
        PathError err = ps->LoadFromFile(sIntegratedPathGUID);
        if ( unlikely(err != PE_SUCCESS))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadRegularizedPaths : ps->LoadFromFile() failed (%d)", err);
            continue;
        }

        integratedPaths::iterator it2 = m_integratedPaths.find(regularizedPaths::key_type(sIntegratedPathGUID));
        if ( likely(it2 != m_integratedPaths.end()))
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath found (uuid: %s)", sIntegratedPathGUID.c_str());
            if ( likely(it2->second))
                ps->SetIntPath(it2->second);
            else
            {
                LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath is NULL (uuid: %s)", sIntegratedPathGUID.c_str());
                delete ps;
                continue;
            }
        }
        else
        {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath not found (uuid: %s)", sIntegratedPathGUID.c_str());
            delete ps;
            continue;
        }

        std::pair<regularizedPaths::iterator, bool> res = m_regularizedPaths.insert(regularizedPaths::value_type(guid, ps));
        if ( unlikely(!res.second))
        {
            LG(WARN, "ReferentiableManager<PathSuite>::LoadRegularizedPaths : m_regularizedPaths.insert failed (uuid: %s)", guid.c_str());
            delete ps;
            continue;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadRegularizedPaths : m_regularizedPaths.insert success (uuid: %s)", guid.c_str());
        }
    }
    
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadRegularizedPaths end");
}

ReferentiableManager<PathSuite> * ReferentiableManager<PathSuite>::g_pRefManager = NULL;


ReferentiableManager<PathSuite> * ReferentiableManager<PathSuite>::getInstance()
{
    if (!g_pRefManager)
    {
        g_pRefManager = new ReferentiableManager<PathSuite>();
    }

    return g_pRefManager;
}


Referentiable* ReferentiableManager<PathSuite>::newReferentiableInternal(const std::string & nameHint, const std::vector<std::string> & guids, bool bVisible, bool bFinalize)
{
    /*LG(INFO, "ReferentiableManager<PathSuite>::newReferentiable(%s, %d guids) begin",
        (nameHint.c_str() ? nameHint.c_str() : "NULL"),
        guids.size());
        */
    PathSuite* curAnim = NULL;

    std::string guid;

    int sizeGuids = (int)guids.size();

    if (sizeGuids > 0)
    {
        guid.assign(guids[0]);
    }
    else
    {
        guid = generateGuid();
    }

    curAnim = new PathSuite(this, guid, nameHint);
    if (bVisible)
        curAnim->Hide();
    if ( unlikely(!ComputeSessionName(curAnim, bFinalize)))
    {
        LG(ERR, "ReferentiableManager<PathSuite>::newReferentiable : ComputeSessionName failed (uuid: %s)", guid.c_str());
        delete curAnim;
        curAnim = NULL;
        goto end;
    }

end:

    LG((curAnim ? INFO : ERR), "ReferentiableManager<PathSuite>::newReferentiable(...) returns 0x%x", curAnim);
    return curAnim;
}
}
