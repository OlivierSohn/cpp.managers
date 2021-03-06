
namespace imajuscule
{
    // when first created, if a path is passed to the PathManager it becomes the current directory
    template <>
    class ReferentiableManager<PathSuite> : public ReferentiableManagerBase
    {
        friend class Globals;
    public:
        static ReferentiableManager * getInstance();

        const char * defaultNameHint()override;
        const char * UIName()override;
        
        unsigned int index()override;
    private:
        static ReferentiableManager * g_pRefManager;

        ReferentiableManager();
        virtual ~ReferentiableManager();

        ref_unique_ptr<Referentiable> newReferentiableInternal(const std::string & nameHint, const std::vector<std::string> & guids, bool bFinalize) override;

        void doTearDown() override;
    public:
        ref_unique_ptr<PathSuite> newPath(const std::string & nameHint,
            const std::vector<std::string> & guids, 
            float freqCutoff = 1.f,
            integratedPath::IntegrationMode intmode = integratedPath::TRAPEZOIDAL);
        
        ref_unique_ptr<PathSuite> newPathVariant(PathSuite * finalizedSuite, const std::string & nameHint, double freqCutoff, bool adaptive, integratedPath::IntegrationMode intmode, const std::vector<std::string> & guids);

        ref_unique_ptr<PathSuite> newPathByCompression(PathSuite * finalizedSuite, const std::string & nameHint, const std::vector<std::string> & guids);

    private:
        // guid - path
        typedef std::map<std::string, integratedPath*> integratedPaths;
        typedef std::map<std::string, regularizedPath*> regularizedPaths;
        typedef std::map<std::string, rawPath*> rawPaths;
        // guid of elementary path - nRefs
        typedef std::map < std::string, int > refMap;
        
        std::vector<intrusive_ptr<PathSuite>> path_suites;

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
