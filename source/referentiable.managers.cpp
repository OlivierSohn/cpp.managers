#include <string>

#include "referentiable.manager.hpp"
#include "referentiable.manager.h"
#include "referentiable.root.h"
#include "referentiables.h"

#include "cg.math.animation.param.h"
#include "param.h"
#include "animation.h"

#include "layer.rigidbody.h"
#include "character.h"
#include "constraint.iterative.h"
#include "enum.scandirection.h"
#include "enum.postrender.h"
#include "enum.focusmodel.h"
#include "wmoperation.spec.draw.h"
#include "wmoperation.spec.png.h"
#include "wmoperation.spec.text.h"
#include "positionable.body.stai.raster.h"
#include "positionable.body.stai.effect.h"
#include "positionable.body.stai.sim.h"
#include "joint.h"
#include "tdogl/CameraBase.h"
#include "tdogl/Camera.h"
#include "tdogl/CamOnCurve.h"
#include "path.suite.h"
#include "path.manager.h"
#include "param.filepath.h"
#include "shot.h"
#include "script.h"
#include "timeline.h"
#include "motion/layer.generic.h"
#include "motion/layer.generic.framed.h"
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
#include "paramset.postextrude.h"
#include "paramset.rasterize.h"
#include "paramset.render.h"
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
    /*template<>*/ const char * ReferentiableManager<PathSuite>::defaultNameHint()       { return "Path"; }
    /*template<>*/ const char * ReferentiableManager<PathSuite>::UIName()       { return "Paths"; }
    /*template<>*/ unsigned int ReferentiableManager<PathSuite>::index()  { return 0; }

    MAKE_REF_MANAGER(1,  Animation<float>,      "Animations Float", "Anim Flt");
    MAKE_REF_MANAGER(2,  Animation<int>,        "Animations Int",   "Anim Int");
    MAKE_REF_MANAGER(3,  Animation<bool>,       "Animations Bool",  "Anum Bool");
    MAKE_REF_MANAGER(4,  Animation<std::string>,"Animations String","Anim Str");
    MAKE_REF_MANAGER(5,  Param<float>,          "Params Float",     "P");
    MAKE_REF_MANAGER(6,  Param<int>,            "Params Int",       "PI");
    MAKE_REF_MANAGER(7,  Param<bool>,           "Params Bool",      "PB");
    MAKE_REF_MANAGER(8,  Param<std::string>,    "Params String",    "PS");
    MAKE_REF_MANAGER(9,  Shot,                  "Shots",            "Shot");
    MAKE_REF_MANAGER(10, SpecWMDraw,            "SpecWMSDraw",      "Draw");
    MAKE_REF_MANAGER(11, RasterSTAIBody,        "Bodys Raster",     "Body");
    MAKE_REF_MANAGER(12, SimSTAIBody,           "Bodys Sim",        "Sim");
    MAKE_REF_MANAGER(13, Joint,                 "Joints",           "J");
    MAKE_REF_MANAGER(14, KinMotion,             "Kin Motions",      "KM");
    MAKE_REF_MANAGER(15, KinChain,              "Kin Chains",       "KC");
    MAKE_REF_MANAGER(16, CameraBase,            "Cameras",          "CamB");
    MAKE_REF_MANAGER(17, Camera,                "Cameras 2",        "Cam");
    MAKE_REF_MANAGER(18, Script,                "Scripts",          "Script");
    MAKE_REF_MANAGER(19, CamOnCurve,            "Cameras On Curve", "CamOC");
    MAKE_REF_MANAGER(20, MotionCompositor,      "Motion Compositors","MC");
    MAKE_REF_MANAGER(21, ContinuousCurveMotion, "Cont.Curve Motions","CCM");
    MAKE_REF_MANAGER(22, DiscreteCurveMotion,   "Disc.Curve Motions","DCM");
    MAKE_REF_MANAGER(23, LocalSpaceMotion,      "Loc.Space Motions","LSM");
    MAKE_REF_MANAGER(24, OffsetMotion,          "Offset Motions",   "OM");
    MAKE_REF_MANAGER(25, ParamMotion,           "Param Motions",    "PM");
    MAKE_REF_MANAGER(26, Position,              "Positions",        "Position");
    MAKE_REF_MANAGER(27, Color,                 "Colors",           "Color");
    MAKE_REF_MANAGER(28, Extrude,               "Extrudes",         "Extrude");
    MAKE_REF_MANAGER(29, FilePath,              "Files",            "File");
    MAKE_REF_MANAGER(30, Rasterize,             "Rasterizes",       "Rasterize");
    MAKE_REF_MANAGER(31, Rounding,              "Roundings",        "Rounding");
    MAKE_REF_MANAGER(32, Scale,                 "Scalse",           "Scale");
    MAKE_REF_MANAGER(33, Shrink,                "Shrinks",          "Shrink");
    MAKE_REF_MANAGER(34, WMGeoOp,               "WM Goe Ops",       "WMGeoOp");
    MAKE_REF_MANAGER(35, RasterizeOp,           "Rasterize Ops",    "RasterizeOp");
    MAKE_REF_MANAGER(36, STAIRenderOp,          "STAI Render Ops",  "STAIRenderOp");
    MAKE_REF_MANAGER(37, ShrinkOp,              "Shrink Ops",       "ShrinkOp");
    MAKE_REF_MANAGER(38, STAIRasterOp,          "STAI Raster Ops",  "STAIRasterOp");
    MAKE_REF_MANAGER(39, STAISimOp,             "STAI Sim Ops",     "STAISimOp");
    MAKE_REF_MANAGER(40, ScaleOp,               "Scale Ops",        "Scale");
    MAKE_REF_MANAGER(41, RoundedWM,             "Rounded WMS",      "Rounded WM");
    MAKE_REF_MANAGER(42, WireModel,             "WMs",               "WM");
    MAKE_REF_MANAGER(43, WireTextModel,         "WTMs",              "WTM");
    MAKE_REF_MANAGER(44, ReferentiableRoot,     "Roots",             "Root");
    MAKE_REF_MANAGER(45, SpecWMText,            "SpecWmsText",      "SpecText");
    MAKE_REF_MANAGER(46, Text,                  "Texts",            "Text");
    MAKE_REF_MANAGER(47, Timeline,              "Timelines",        "Timeline");
	MAKE_REF_MANAGER(48, GeoTransform,			"GeoTs",            "GeoT");
	MAKE_REF_MANAGER(49, STAIEffectOp,			"Effect Ops",       "Effect Op");
	MAKE_REF_MANAGER(50, EffectOp,				"2Effect Ops",      "2Effect Op");
	MAKE_REF_MANAGER(51, EffectSTAIBody,		"Bodys Effect",    "Effect");
    MAKE_REF_MANAGER(52, Effect,				"Effects",          "Effect");
    MAKE_REF_MANAGER(53, InterpolationParam, 	"Interpolations",   "Interpolation");
    MAKE_REF_MANAGER(54, ScanDirectionParam, 	"Scan_Directions",  "Scan Direction");
    MAKE_REF_MANAGER(55, Png,                   "Png Import Params","Png Import");
    MAKE_REF_MANAGER(56, SpecWMPng,             "SpecWMSPng",       "Png");
    MAKE_REF_MANAGER(57, FilePathParam,         "FilePaths",        "FilePath");
    MAKE_REF_MANAGER(58, Sphere,                "Spheres",          "Sphere");
    MAKE_REF_MANAGER(59, ParallellepipedRect,   "ParRects",         "ParRect");
    MAKE_REF_MANAGER(60, Point3D,               "3dPoints",         "Point");
    MAKE_REF_MANAGER(61, Constraint,            "Constraints",       "Constraint");
    MAKE_REF_MANAGER(62, IterativeConstraint,   "ITER Constraints",    "ITER Constraint");
    MAKE_REF_MANAGER(63, Human,                 "Humans",           "Human");
    MAKE_REF_MANAGER(64, MotionMixer,           "Motion Mixers",    "Motion Mixer");
    MAKE_REF_MANAGER(65, RigidBodyMotion,       "Rigid Body Motions","Rigid Body Motion");
    MAKE_REF_MANAGER(66, PostExtrude,           "PostExtrudes","PostExtrude");
    MAKE_REF_MANAGER(67, Render,           "Renders","Render");
    MAKE_REF_MANAGER(68, postRenderParam,           "Post Render Params","Post Render");
    MAKE_REF_MANAGER(69, GenericMotion,           "Generic Motions","Generic Motion");
    MAKE_REF_MANAGER(70, FramedMotion,           "Framed Motions","Framed Motion");
    MAKE_REF_MANAGER(71, focusModelParam,           "Focus Model Params","Focus Model");

    int InitializeRefManagers(Referentiables & rs )
    {
        // the last referentiable to be deinstantiated on cleanup should be the ref root, so
        // it needs to be the first one created :
        
        // we create first the root manager ...
        auto root_manager = ReferentiableManager<ReferentiableRoot>::getInstance();
        // ... and we create the root
        auto root = ReferentiableRoot::getInstance();

        rs.regManager(ReferentiableManager<PathSuite>::getInstance());
        rs.regManager(ReferentiableManager<Animation<float>>::getInstance());
        rs.regManager(ReferentiableManager<Animation<int>>::getInstance());
        rs.regManager(ReferentiableManager<Animation<bool>>::getInstance());
        rs.regManager(ReferentiableManager<Animation<std::string>>::getInstance());
        rs.regManager(ReferentiableManager<Param<float>>::getInstance());
        rs.regManager(ReferentiableManager<Param<int>>::getInstance());
        rs.regManager(ReferentiableManager<Param<bool>>::getInstance());
        rs.regManager(ReferentiableManager<Param<std::string>>::getInstance());
        rs.regManager(ReferentiableManager<Shot>::getInstance());
        rs.regManager(ReferentiableManager<SpecWMDraw>::getInstance());
        rs.regManager(ReferentiableManager<RasterSTAIBody>::getInstance());
        rs.regManager(ReferentiableManager<SimSTAIBody>::getInstance());
        rs.regManager(ReferentiableManager<Joint>::getInstance());
        rs.regManager(ReferentiableManager<KinMotion>::getInstance());
        rs.regManager(ReferentiableManager<KinChain>::getInstance());
        rs.regManager(ReferentiableManager<CameraBase>::getInstance());
        rs.regManager(ReferentiableManager<Camera>::getInstance());
        rs.regManager(ReferentiableManager<Script>::getInstance());
        rs.regManager(ReferentiableManager<CamOnCurve>::getInstance());
        rs.regManager(ReferentiableManager<MotionCompositor>::getInstance());
        rs.regManager(ReferentiableManager<ContinuousCurveMotion>::getInstance());
        rs.regManager(ReferentiableManager<DiscreteCurveMotion>::getInstance());
        rs.regManager(ReferentiableManager<LocalSpaceMotion>::getInstance());
        rs.regManager(ReferentiableManager<OffsetMotion>::getInstance());
        rs.regManager(ReferentiableManager<ParamMotion>::getInstance());
        rs.regManager(ReferentiableManager<Position>::getInstance());
        rs.regManager(ReferentiableManager<Color>::getInstance());
        rs.regManager(ReferentiableManager<Extrude>::getInstance());
        rs.regManager(ReferentiableManager<FilePath>::getInstance());
        rs.regManager(ReferentiableManager<Rasterize>::getInstance());
        rs.regManager(ReferentiableManager<Rounding>::getInstance());
        rs.regManager(ReferentiableManager<Scale>::getInstance());
        rs.regManager(ReferentiableManager<Shrink>::getInstance());
        rs.regManager(ReferentiableManager<WMGeoOp>::getInstance());
        rs.regManager(ReferentiableManager<RasterizeOp>::getInstance());
        rs.regManager(ReferentiableManager<STAIRenderOp>::getInstance());
        rs.regManager(ReferentiableManager<ShrinkOp>::getInstance());
        rs.regManager(ReferentiableManager<STAIRasterOp>::getInstance());
        rs.regManager(ReferentiableManager<STAISimOp>::getInstance());
        rs.regManager(ReferentiableManager<ScaleOp>::getInstance());
        rs.regManager(ReferentiableManager<RoundedWM>::getInstance());
        rs.regManager(ReferentiableManager<WireModel>::getInstance());
        rs.regManager(ReferentiableManager<WireTextModel>::getInstance());
        rs.regManager(root_manager);
        rs.regManager(ReferentiableManager<SpecWMText>::getInstance());
        rs.regManager(ReferentiableManager<Text>::getInstance());
        rs.regManager(ReferentiableManager<Timeline>::getInstance());
		rs.regManager(ReferentiableManager<GeoTransform>::getInstance());
		rs.regManager(ReferentiableManager<STAIEffectOp>::getInstance());
		rs.regManager(ReferentiableManager<EffectOp>::getInstance());
		rs.regManager(ReferentiableManager<EffectSTAIBody>::getInstance());
        rs.regManager(ReferentiableManager<Effect>::getInstance());
        rs.regManager(ReferentiableManager<InterpolationParam>::getInstance());
        rs.regManager(ReferentiableManager<ScanDirectionParam>::getInstance());
        rs.regManager(ReferentiableManager<Png>::getInstance());
        rs.regManager(ReferentiableManager<SpecWMPng>::getInstance());
        rs.regManager(ReferentiableManager<FilePathParam>::getInstance());
        rs.regManager(ReferentiableManager<Sphere>::getInstance());
        rs.regManager(ReferentiableManager<ParallellepipedRect>::getInstance());
        rs.regManager(ReferentiableManager<Point3D>::getInstance());
        rs.regManager(ReferentiableManager<Constraint>::getInstance());
        rs.regManager(ReferentiableManager<IterativeConstraint>::getInstance());
        rs.regManager(ReferentiableManager<Human>::getInstance());
        rs.regManager(ReferentiableManager<MotionMixer>::getInstance());
        rs.regManager(ReferentiableManager<RigidBodyMotion>::getInstance());
        rs.regManager(ReferentiableManager<PostExtrude>::getInstance());
        rs.regManager(ReferentiableManager<Render>::getInstance());
        rs.regManager(ReferentiableManager<postRenderParam>::getInstance());
        rs.regManager(ReferentiableManager<GenericMotion>::getInstance());
        rs.regManager(ReferentiableManager<FramedMotion>::getInstance());
        rs.regManager(ReferentiableManager<focusModelParam>::getInstance());
        
        // and root must be initialized once all ref managers are registered
        if(root) {
            root->initialize();
        }
        
        return 2;
    }
}

