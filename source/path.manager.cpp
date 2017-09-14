
namespace imajuscule {

ReferentiableManager<PathSuite>::ReferentiableManager():
ReferentiableManagerBase()
{
    // TODO ReferentiableManager : genericize LoadRefs
    LoadPaths();
}

    void ReferentiableManager<PathSuite>::doTearDown() {
        path_suites.clear();
    }
    
ReferentiableManager<PathSuite>::~ReferentiableManager() {
    Assert(g_pRefManager == this);
}

ref_unique_ptr<PathSuite> ReferentiableManager<PathSuite>::newPath(const std::string & nameHint, const std::vector<std::string> & guids, float freqCutoff, integratedPath::IntegrationMode intmode)
{
    /*LG(INFO, "ReferentiableManager<PathSuite>::newPath(%s, %d guids, %f, %s, %d) begin",
        (nameHint.c_str() ? nameHint.c_str() : "nullptr"),
        guids.size(),
        freqCutoff,
        (adaptive ? "true" : "false"),
        intmode);
        */
    std::string guid, guid1, guid2, guid3;
    
    int sizeGuids = (int)guids.size();

    if (sizeGuids > 0) {
        guid.assign(guids[0]);
    }
    else {
        guid = generateGuid();
    }

    if (sizeGuids > 1) {
        guid1.assign(guids[1]);
    }
    else {
        guid1 = generateGuid();
    }

    if (sizeGuids > 2) {
        guid2.assign(guids[2]);
    }
    else {
        guid2 = generateGuid();
    }

    if (sizeGuids > 3) {
        guid3.assign(guids[3]);
    }
    else {
        guid3 = generateGuid();
    }
    
    std::unique_ptr<rawPath> newRawPath(new rawPath(guid1));
    {
        std::pair<rawPaths::iterator, bool> resInsert = m_rawPaths.insert(rawPaths::value_type(guid1, newRawPath.get()));
        if ( unlikely(!resInsert.second))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::newPath : m_rawPaths.insert failed (uuid: %s)", guid1.c_str());
            return {};
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::newPath : m_rawPaths.insert success (uuid: %s)", guid1.c_str());
        }
    }
    
    std::unique_ptr<integratedPath> newIntPath(new integratedPath(guid2, newRawPath.get(), freqCutoff, intmode));
    {
        std::pair<integratedPaths::iterator, bool> resInsert = m_integratedPaths.insert(integratedPaths::value_type(guid2, newIntPath.get()));
        if ( unlikely(!resInsert.second))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::newPath : m_integratedPaths.insert failed (uuid: %s)", guid2.c_str());
            return {};
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::newPath : m_integratedPaths.insert success (uuid: %s)", guid2.c_str());
        }
    }
    
    std::unique_ptr<regularizedPath> newRegPath(new regularizedPath(guid3, newIntPath.get()));
    {
        std::pair<regularizedPaths::iterator, bool> resInsert = m_regularizedPaths.insert(regularizedPaths::value_type(guid3, newRegPath.get()));
        if ( unlikely(!resInsert.second))
        {
            LG(ERR, "ReferentiableManager<PathSuite>::newPath : m_regularizedPaths.insert failed (uuid: %s)", guid3.c_str());
            return {};
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::newPath : m_regularizedPaths.insert success (uuid: %s)", guid3.c_str());
        }
    }

    auto ref = make_unique_ref<PathSuite>(this, nameHint, guid, newRawPath.get(), newIntPath.get(), newRegPath.get());
    if ( unlikely(!ComputeSessionName(ref.get(), true))) {
        LG(ERR, "ReferentiableManager<PathSuite>::newPath : ComputeSessionName failed (uuid: %s)", guid.c_str());
        return {};
    }
    
    newRawPath.release();
    newRegPath.release();
    newIntPath.release();

    LG(INFO, "ReferentiableManager<PathSuite>::newPath(...) returns 0x%x", ref.get());
    return ref;
}

ref_unique_ptr<PathSuite> ReferentiableManager<PathSuite>::newPathVariant(PathSuite * finalizedSuite, const std::string & nameHint, double freqCutoff, bool adaptive, integratedPath::IntegrationMode intmode, const std::vector<std::string> & guids)
{
    Assert(0);//todo
    return {};
}

ref_unique_ptr<PathSuite> ReferentiableManager<PathSuite>::newPathByCompression(PathSuite * finalizedSuite, const std::string & nameHint, const std::vector<std::string> & guids)
{
    Assert(0);//todo
    return {};
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

    using namespace StorageStuff;
    for ( auto guid : listFilenames(directory_pathsuites()) )
    {
        auto ps = make_intrusive<PathSuite>(this, std::move(guid), std::string("path"));
        
        std::string Raw, Int, Reg;
        PathError ret = ps->LoadFromFile(Raw, Int, Reg);
        if ( unlikely(ret != PE_SUCCESS)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : ps->LoadFromFile failed (err: %d)", ret);
            continue;
        }

        rawPaths::iterator rawIt = m_rawPaths.find(rawPaths::key_type(Raw));
        if ( unlikely(rawIt == m_rawPaths.end())) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : raw path not found (uuid : %s)", Raw.c_str());
            continue;
        }

        integratedPaths::iterator intIt = m_integratedPaths.find(integratedPaths::key_type(Int));
        if ( unlikely(intIt == m_integratedPaths.end())) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : int path not found (uuid : %s)", Int.c_str());
            continue;
        }

        regularizedPaths::iterator regIt = m_regularizedPaths.find(regularizedPaths::key_type(Reg));
        if ( unlikely(regIt == m_regularizedPaths.end())) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : reg path not found (uuid : %s)", Reg.c_str());
            continue;
        }

        rawPath * rawP = rawIt->second;
        integratedPath * intP = intIt->second;
        regularizedPath * regP = regIt->second;

        if ( unlikely(!rawP)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : raw path nullptr");
            continue;
        }

        if ( unlikely(!intP)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : int path nullptr");
            continue;
        }

        if ( unlikely(!regP)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : reg path nullptr");
            continue;
        }

        ps->SetPaths(rawP, intP, regP);

        ps->Initialize();

        ret = ps->FinalizeRecord();
        if ( unlikely(ret != PE_SUCCESS)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : ps->FinalizeRecord failed (err: %d)", ret);
            continue;
        }
        
        auto ref = static_pointer_cast<Referentiable>(ps);
        if( unlikely(!ComputeSessionName(ref, true)) ) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadPathSuites : ComputeSessionName failed");
            continue;
        }
        else
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadPathSuites : ComputeSessionName success");
        }
        
        path_suites.emplace_back(std::move(ps));
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::LoadPathSuites end");
}

int ReferentiableManager<PathSuite>::addRef(const std::string & guid)
{
    //LG(INFO, "ReferentiableManager<PathSuite>::addRef(%s)", (guid.c_str() ? guid.c_str():"nullptr"));

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

    //LG(INFO, "ReferentiableManager<PathSuite>::addRef(%s) returns %d", (guid.c_str() ? guid.c_str() : "nullptr"), res.first->second);
    return res.first->second;
}

int ReferentiableManager<PathSuite>::removeRef(const std::string & guid)
{
    //LG(INFO, "ReferentiableManager<PathSuite>::removeRef(%s)", (guid.c_str() ? guid.c_str() : "nullptr"));

    int res = 0;

    // if element exists and 
    refMap::iterator it = m_refMap.find(guid);
    if ( likely(it != m_refMap.end())) {
        //LG(INFO, "ReferentiableManager<PathSuite>::removeRef : found existing element");
        it->second--;

        res = it->second;

        if (res <= 0)
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::removeRef : no element references this element, remove it from map");
            m_refMap.erase(it);
        }
    }
    else {
        LG(ERR, "ReferentiableManager<PathSuite>::removeRef : element not found");
    }

    //LG(INFO, "ReferentiableManager<PathSuite>::removeRef(%s) returns %d", (guid.c_str() ? guid.c_str() : "nullptr"), res);
    return res;
}

void ReferentiableManager<PathSuite>::LoadRawPaths()
{
    //LG(INFO, "ReferentiableManager<PathSuite>::LoadRawPaths begin");

    for ( auto const & guid : StorageStuff::listFilenames(directory_rawpaths()) )
    {
        rawPath * ps = new rawPath(guid);
       
        PathError err = ps->LoadFromFile();
        if ( unlikely(err != PE_SUCCESS)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadRawPaths : ps->LoadFromFile() failed (%d)", err);
            continue;
        }

        std::pair<rawPaths::iterator, bool> res = m_rawPaths.insert(rawPaths::value_type(guid, ps));
        if ( unlikely(!res.second)) {
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

    for ( auto const & guid : StorageStuff::listFilenames(directory_intpaths()) )
    {
        integratedPath * ps = new integratedPath(guid);
        std::string sRawPathGUID;
        
        PathError err = ps->LoadFromFile(sRawPathGUID);
        if ( unlikely(err != PE_SUCCESS)) {
            LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : ps->LoadFromFile() failed (%d)", err);
            continue;
        }

        rawPaths::iterator it2 = m_rawPaths.find(rawPaths::key_type(sRawPathGUID));
        if ( likely(it2 != m_rawPaths.end()))
        {
            //LG(INFO, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath found (uuid: %s)", sRawPathGUID.c_str());
            if ( likely(it2->second)) {
                ps->SetRawPath(it2->second);
            }
            else {
                LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath is nullptr (uuid: %s)", sRawPathGUID.c_str());
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

        auto res = m_integratedPaths.insert(integratedPaths::value_type(guid, ps));
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

    for ( auto const & guid : StorageStuff::listFilenames(directory_regpaths()) )
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
                LG(ERR, "ReferentiableManager<PathSuite>::LoadIntegratedPaths : associated rawPath is nullptr (uuid: %s)", sIntegratedPathGUID.c_str());
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

ReferentiableManager<PathSuite> * ReferentiableManager<PathSuite>::g_pRefManager = nullptr;


ReferentiableManager<PathSuite> * ReferentiableManager<PathSuite>::getInstance()
{
    return Globals::ptr<ReferentiableManager<PathSuite>>(g_pRefManager);
}


ref_unique_ptr<Referentiable> ReferentiableManager<PathSuite>::newReferentiableInternal(const std::string & nameHint, const std::vector<std::string> & guids, bool bFinalize)
{
    /*LG(INFO, "ReferentiableManager<PathSuite>::newReferentiable(%s, %d guids) begin",
        (nameHint.c_str() ? nameHint.c_str() : "nullptr"),
        guids.size());
        */

    std::string guid;

    if (guids.size() > 0) {
        guid.assign(guids[0]);
    }
    else {
        guid = generateGuid();
    }

    auto curAnim = make_unique_ref<PathSuite>(this, guid, nameHint);
    if ( unlikely(!ComputeSessionName(curAnim.get(), bFinalize))) {
        LG(ERR, "ReferentiableManager<PathSuite>::newReferentiable : ComputeSessionName failed (uuid: %s)", guid.c_str());
        return {};
    }

    return {curAnim.release()};
}
}
