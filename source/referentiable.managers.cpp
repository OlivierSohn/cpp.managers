#include <string>

#include "referentiable.manager.hpp"
#include "referentiable.manager.h"
#include "referentiable.root.h"
#include "referentiables.h"

#include "cg.math.animation.param.h"
#include "param.h"
#include "animation.h"

#include "character.h"
#include "constraint.iterative.h"
#include "enum.scandirection.h"
#include "wmoperation.spec.draw.h"
#include "wmoperation.spec.png.h"
#include "wmoperation.spec.text.h"
#include "positionable.body.stai.raster.h"
#include "positionable.body.stai.effect.h"
#include "positionable.body.stai.sim.h"
#include "joint.h"
#include "tdogl/CameraBase.h"
#include "tdogl/Camera.h"
#include "tdogl/WeightCam.h"
#include "tdogl/CamOnCurve.h"
#include "path.suite.h"
#include "path.manager.h"
#include "param.filepath.h"
#include "shot.h"
#include "timeline.h"
#include "motion/layer.mixer.h"
#include "motion/layer.compositor.h"
#include "motion/layer.curve.continuous.h"
#include "motion/layer.curve.discrete.h"
#include "motion/layer.localspace.h"
#include "motion/layer.offset.h"
#include "motion/layer.param.h"

#include "operation.geo.h"
#include "operation.rasterize.h"
#include "operation.render.h"
#include "operation.scale.h"
#include "operation.shrink.h"
#include "operation.stai.raster.h"
#include "operation.stai.effect.h"
#include "operation.stai.sim.h"

#include "paramset.3d.point.h"
#include "paramset.color.h"
#include "paramset.effect.h"
#include "paramset.extrude.h"
#include "paramset.filepath.h"
#include "paramset.geotransform.h"
#include "paramset.rasterize.h"
#include "paramset.rounding.h"
#include "paramset.scale.h"
#include "paramset.shrink.h"

#include "WireModel.h"
#include "WireTextModel.h"

#define MAKE_REF_MANAGER(vIndex, t, uiManagerName, uiRefNameHint) \
template class ReferentiableManager<t>;\
template<> const char * ReferentiableManager<t>::defaultNameHint()  { return uiRefNameHint; }\
template<> const char * ReferentiableManager<t>::UIName()  { return uiManagerName; } \
template<> unsigned int ReferentiableManager<t>::index()  { return vIndex; }


namespace imajuscule {
    
    template class ReferentiableManager<PathSuite>;
    /*template<>*/ const char * ReferentiableManager<PathSuite>::defaultNameHint()       { return "PATH"; }
    /*template<>*/ const char * ReferentiableManager<PathSuite>::UIName()       { return "PATHS"; }
    /*template<>*/ unsigned int ReferentiableManager<PathSuite>::index()  { return 0; }

    MAKE_REF_MANAGER(1,  Animation<float>,      "ANIMATIONS FLOAT", "A");
    MAKE_REF_MANAGER(2,  Animation<int>,        "ANIMATIONS INT",   "AI");
    MAKE_REF_MANAGER(3,  Animation<bool>,       "ANIMATIONS BOOL",  "AB");
    MAKE_REF_MANAGER(4,  Animation<std::string>,"ANIMATIONS STRING","AS");
    MAKE_REF_MANAGER(5,  Param<float>,          "PARAMS FLOAT",     "P");
    MAKE_REF_MANAGER(6,  Param<int>,            "PARAMS INT",       "PI");
    MAKE_REF_MANAGER(7,  Param<bool>,           "PARAMS BOOL",      "PB");
    MAKE_REF_MANAGER(8,  Param<std::string>,    "PARAMS STRING",    "PS");
    MAKE_REF_MANAGER(9,  Shot,                  "SHOTS",            "SHOT");
    MAKE_REF_MANAGER(10, SpecWMDraw,            "SPECWMSDRAW",      "DRAW");
    MAKE_REF_MANAGER(11, RasterSTAIBody,        "BODYS_RASTER",     "BODY");
    MAKE_REF_MANAGER(12, SimSTAIBody,           "BODYS_SIM",        "SIM");
    MAKE_REF_MANAGER(13, Joint,                 "JOINTS",           "J");
    MAKE_REF_MANAGER(14, KinMotion,             "KIN MOTIONS",      "KM");
    MAKE_REF_MANAGER(15, KinChain,              "KIN CHAINS",       "KC");
    MAKE_REF_MANAGER(16, CameraBase,            "CAMERAS",          "CAMB");
    MAKE_REF_MANAGER(17, Camera,                "CAMERAS 2",        "CAM");
    MAKE_REF_MANAGER(18, WeightCam,             "CAMERAS_DYN",      "WCAM");
    MAKE_REF_MANAGER(19, CamOnCurve,            "CAMERAS ON CURVE", "CAMOC");
    MAKE_REF_MANAGER(20, MotionCompositor,      "MOTION COMPOSITORS","MC");
    MAKE_REF_MANAGER(21, ContinuousCurveMotion, "CONT.CURVE MOTIONS","CCM");
    MAKE_REF_MANAGER(22, DiscreteCurveMotion,   "DISC.CURVE MOTIONS","DCM");
    MAKE_REF_MANAGER(23, LocalSpaceMotion,      "LOC.SPACE MOTIONS","LSM");
    MAKE_REF_MANAGER(24, OffsetMotion,          "OFFSET MOTIONS",   "OM");
    MAKE_REF_MANAGER(25, ParamMotion,           "PARAM MOTIONS",    "PM");
    MAKE_REF_MANAGER(26, Position,              "POSITIONS",        "POS");
    MAKE_REF_MANAGER(27, Color,                 "COLORS",           "COLOR");
    MAKE_REF_MANAGER(28, Extrude,               "EXTRUDES",         "EXTRUDE");
    MAKE_REF_MANAGER(29, FilePath,              "FILES",            "FILE");
    MAKE_REF_MANAGER(30, Rasterize,             "RASTERIZES",       "RASTERIZE");
    MAKE_REF_MANAGER(31, Rounding,              "ROUNDINGS",        "ROUNDING");
    MAKE_REF_MANAGER(32, Scale,                 "SCALES",           "SCALE");
    MAKE_REF_MANAGER(33, Shrink,                "SHRINKS",          "SHRINK");
    MAKE_REF_MANAGER(34, WMGeoOp,               "WM GEO OPS",       "WMGEOOP");
    MAKE_REF_MANAGER(35, RasterizeOp,           "RASTERIZE OPS",    "RASTERIZEOP");
    MAKE_REF_MANAGER(36, STAIRenderOp,          "STAI RENDER OPS",  "STAIRENDEROP");
    MAKE_REF_MANAGER(37, ShrinkOp,              "SHRINK OPS",       "SHRINKOP");
    MAKE_REF_MANAGER(38, STAIRasterOp,          "STAI RASTER OPS",  "STAIRASTEROP");
    MAKE_REF_MANAGER(39, STAISimOp,             "STAI SIM OPS",     "STAISIMOP");
    MAKE_REF_MANAGER(40, ScaleOp,               "SCALE OPS",        "SCALE");
    MAKE_REF_MANAGER(41, RoundedWM,             "ROUNDED WMS",      "ROUNDEDWM");
    MAKE_REF_MANAGER(42, WireModel,             "WM",               "WM");
    MAKE_REF_MANAGER(43, WireTextModel,         "WTM",              "WTM");
    MAKE_REF_MANAGER(44, ReferentiableRoot,     "ROOT",             "ROOT");
    MAKE_REF_MANAGER(45, SpecWMText,            "SPECWMSTEXT",      "TEXT");
    MAKE_REF_MANAGER(46, Text,                  "TEXTS",            "TXT");
    MAKE_REF_MANAGER(47, Timeline,              "TIMELINES",        "TIMELINE");
	MAKE_REF_MANAGER(48, GeoTransform,			"GEOTS",            "GEOT");
	MAKE_REF_MANAGER(49, STAIEffectOp,			"EFFECT OPS",       "EFFECT OP");
	MAKE_REF_MANAGER(50, EffectOp,				"2EFFECT OPS",      "2EFFECT OP");
	MAKE_REF_MANAGER(51, EffectSTAIBody,		"BODYS_EFFECTS",    "EFFECT");
    MAKE_REF_MANAGER(52, Effect,				"EFFECTS",          "EFFECT");
    MAKE_REF_MANAGER(53, InterpolationParam, 	"Interpolations",   "interpolation");
    MAKE_REF_MANAGER(54, ScanDirectionParam, 	"Scan_Directions",  "scan_direction");
    MAKE_REF_MANAGER(55, Png,                   "PNG Import Params","PNG Import");
    MAKE_REF_MANAGER(56, SpecWMPng,             "SPECWMSPNG",       "PNG");
    MAKE_REF_MANAGER(57, FilePathParam,         "FILEPATHS",        "FILEPATH");
    MAKE_REF_MANAGER(58, Sphere,                "SPHERES",          "SPHERE");
    MAKE_REF_MANAGER(59, ParallellepipedRect,   "PARRECTS",         "PARRECT");
    MAKE_REF_MANAGER(60, Point3D,               "3dpoints",         "point");
    MAKE_REF_MANAGER(61, Constraint,            "CONSTRAINT",       "CONSTR");
    MAKE_REF_MANAGER(62, IterativeConstraint,   "IT CONSTRAINT",    "ITCONSTR");
    MAKE_REF_MANAGER(63, Human,                 "HUMANS",           "HUMAN");
    MAKE_REF_MANAGER(64, MotionMixer,           "MOTION MIXERS",    "MOTION MIXER");
    

    int InitializeRefManagers()
    {
        static bool bFirst = true;
        if (!bFirst)
            return 0;
        bFirst = false;

        Referentiables::registerManager(*(ReferentiableManager<PathSuite>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Animation<float>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Animation<int>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Animation<bool>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Animation<std::string>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Param<float>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Param<int>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Param<bool>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Param<std::string>>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Shot>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<SpecWMDraw>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<RasterSTAIBody>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<SimSTAIBody>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Joint>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<KinMotion>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<KinChain>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<CameraBase>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Camera>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<WeightCam>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<CamOnCurve>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<MotionCompositor>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ContinuousCurveMotion>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<DiscreteCurveMotion>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<LocalSpaceMotion>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<OffsetMotion>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ParamMotion>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Position>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Color>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Extrude>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<FilePath>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Rasterize>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Rounding>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Scale>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Shrink>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<WMGeoOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<RasterizeOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<STAIRenderOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ShrinkOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<STAIRasterOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<STAISimOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ScaleOp>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<RoundedWM>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<WireModel>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<WireTextModel>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ReferentiableRoot>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<SpecWMText>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Text>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Timeline>::getInstance()));
		Referentiables::registerManager(*(ReferentiableManager<GeoTransform>::getInstance()));
		Referentiables::registerManager(*(ReferentiableManager<STAIEffectOp>::getInstance()));
		Referentiables::registerManager(*(ReferentiableManager<EffectOp>::getInstance()));
		Referentiables::registerManager(*(ReferentiableManager<EffectSTAIBody>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Effect>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<InterpolationParam>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ScanDirectionParam>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Png>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<SpecWMPng>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<FilePathParam>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Sphere>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<ParallellepipedRect>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Point3D>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Constraint>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<IterativeConstraint>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<Human>::getInstance()));
        Referentiables::registerManager(*(ReferentiableManager<MotionMixer>::getInstance()));
        
        ReferentiableRoot::getInstance(); // to make sure there is a root
        return 2;
    }
    
#include "path.manager.cpp"
}

