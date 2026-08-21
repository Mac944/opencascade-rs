#include "rust/cxx.h"
#include <BOPAlgo_GlueEnum.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeShapeOnMesh.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFeat_MakeCylindricalHole.hxx>
#include <BRepFeat_MakeDPrism.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepGProp.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRepLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_JoinType.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
#include <Law_Function.hxx>
#include <Law_Interpol.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <Poly_Connect.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Wireframe.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Type.hxx>
#include <StlAPI_Writer.hxx>
#include <TColgp_Array1OfDir.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

// Generic template constructor
template <typename T, typename... Args> std::unique_ptr<T> construct_unique(Args... args) {
  return std::unique_ptr<T>(new T(args...));
}

// Generic List
template <typename T> std::unique_ptr<std::vector<T>> list_to_vector(const NCollection_List<T> &list) {
  return std::unique_ptr<std::vector<T>>(new std::vector<T>(list.begin(), list.end()));
}

// Handles
typedef opencascade::handle<Standard_Type> HandleStandardType;
typedef opencascade::handle<Geom_Curve> HandleGeomCurve;
typedef opencascade::handle<Geom_BSplineCurve> HandleGeomBSplineCurve;
typedef opencascade::handle<Geom_BezierCurve> HandleGeomBezierCurve;
typedef opencascade::handle<Geom_TrimmedCurve> HandleGeomTrimmedCurve;
typedef opencascade::handle<Geom_Surface> HandleGeomSurface;
typedef opencascade::handle<Geom_BezierSurface> HandleGeomBezierSurface;
typedef opencascade::handle<Geom_Plane> HandleGeomPlane;
typedef opencascade::handle<Geom2d_Curve> HandleGeom2d_Curve;
typedef opencascade::handle<Geom2d_Ellipse> HandleGeom2d_Ellipse;
typedef opencascade::handle<Geom2d_TrimmedCurve> HandleGeom2d_TrimmedCurve;
typedef opencascade::handle<Geom_CylindricalSurface> HandleGeom_CylindricalSurface;
typedef opencascade::handle<Poly_Triangulation> HandlePoly_Triangulation;
typedef opencascade::handle<TopTools_HSequenceOfShape> HandleTopTools_HSequenceOfShape;
typedef opencascade::handle<Law_Function> HandleLawFunction;

typedef opencascade::handle<TColgp_HArray1OfPnt> Handle_TColgpHArray1OfPnt;

inline std::unique_ptr<Handle_TColgpHArray1OfPnt>
new_HandleTColgpHArray1OfPnt_from_TColgpHArray1OfPnt(std::unique_ptr<TColgp_HArray1OfPnt> array) {
  return std::unique_ptr<Handle_TColgpHArray1OfPnt>(new Handle_TColgpHArray1OfPnt(array.release()));
}

// Handle stuff
template <typename T> const T &handle_try_deref(const opencascade::handle<T> &handle) {
  if (handle.IsNull()) {
    throw std::runtime_error("null handle dereference");
  }
  return *handle;
}

inline const HandleStandardType &DynamicType(const HandleGeomSurface &surface) { return surface->DynamicType(); }

inline rust::String type_name(const HandleStandardType &handle) { return std::string(handle->Name()); }

inline std::unique_ptr<gp_Pnt> HandleGeomCurve_Value(const HandleGeomCurve &curve, const Standard_Real U) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(curve->Value(U)));
}

inline std::unique_ptr<gp_Pnt> GCPnts_TangentialDeflection_Value(const GCPnts_TangentialDeflection &approximator,
                                                                 Standard_Integer i) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(approximator.Value(i)));
}

inline std::unique_ptr<HandleGeomPlane> new_HandleGeomPlane_from_HandleGeomSurface(const HandleGeomSurface &surface) {
  HandleGeomPlane plane_handle = opencascade::handle<Geom_Plane>::DownCast(surface);
  return std::unique_ptr<HandleGeomPlane>(new opencascade::handle<Geom_Plane>(plane_handle));
}

// Collections
inline void shape_list_append_face(TopTools_ListOfShape &list, const TopoDS_Face &face) { list.Append(face); }

// Geometry
inline const gp_Pnt &handle_geom_plane_location(const HandleGeomPlane &plane) { return plane->Location(); }

inline std::unique_ptr<HandleGeom_CylindricalSurface> Geom_CylindricalSurface_ctor(const gp_Ax3 &axis, double radius) {
  return std::unique_ptr<HandleGeom_CylindricalSurface>(
      new opencascade::handle<Geom_CylindricalSurface>(new Geom_CylindricalSurface(axis, radius)));
}

inline std::unique_ptr<HandleGeomBSplineCurve> GeomAPI_Interpolate_Curve(const GeomAPI_Interpolate &interpolate) {
  return std::unique_ptr<HandleGeomBSplineCurve>(new opencascade::handle<Geom_BSplineCurve>(interpolate.Curve()));
}

inline std::unique_ptr<HandleGeomBezierCurve>
Geom_BezierCurve_to_handle(std::unique_ptr<Geom_BezierCurve> bezier_curve) {
  return std::unique_ptr<HandleGeomBezierCurve>(new HandleGeomBezierCurve(bezier_curve.release()));
}

inline std::unique_ptr<HandleGeomSurface> cylinder_to_surface(const HandleGeom_CylindricalSurface &cylinder_handle) {
  return std::unique_ptr<HandleGeomSurface>(new opencascade::handle<Geom_Surface>(cylinder_handle));
}

inline std::unique_ptr<HandleGeomBezierSurface> Geom_BezierSurface_ctor(const TColgp_Array2OfPnt &poles) {
  return std::unique_ptr<HandleGeomBezierSurface>(
      new opencascade::handle<Geom_BezierSurface>(new Geom_BezierSurface(poles)));
}

inline std::unique_ptr<HandleGeomSurface> bezier_to_surface(const HandleGeomBezierSurface &bezier_handle) {
  return std::unique_ptr<HandleGeomSurface>(new opencascade::handle<Geom_Surface>(bezier_handle));
}

inline std::unique_ptr<HandleGeom2d_Ellipse> Geom2d_Ellipse_ctor(const gp_Ax2d &axis, double major_radius,
                                                                 double minor_radius) {
  return std::unique_ptr<HandleGeom2d_Ellipse>(
      new opencascade::handle<Geom2d_Ellipse>(new Geom2d_Ellipse(axis, major_radius, minor_radius)));
}

inline std::unique_ptr<HandleGeom2d_Curve> ellipse_to_HandleGeom2d_Curve(const HandleGeom2d_Ellipse &ellipse_handle) {
  return std::unique_ptr<HandleGeom2d_Curve>(new opencascade::handle<Geom2d_Curve>(ellipse_handle));
}

inline std::unique_ptr<HandleGeom2d_TrimmedCurve> Geom2d_TrimmedCurve_ctor(const HandleGeom2d_Curve &curve, double u1,
                                                                           double u2) {
  return std::unique_ptr<HandleGeom2d_TrimmedCurve>(
      new opencascade::handle<Geom2d_TrimmedCurve>(new Geom2d_TrimmedCurve(curve, u1, u2)));
}

inline std::unique_ptr<HandleGeom2d_Curve>
HandleGeom2d_TrimmedCurve_to_curve(const HandleGeom2d_TrimmedCurve &trimmed_curve) {
  return std::unique_ptr<HandleGeom2d_Curve>(new opencascade::handle<Geom2d_Curve>(trimmed_curve));
}

inline std::unique_ptr<gp_Pnt2d> ellipse_value(const HandleGeom2d_Ellipse &ellipse, double u) {
  return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(ellipse->Value(u)));
}

// Segment Stuff
inline std::unique_ptr<HandleGeomTrimmedCurve> GC_MakeSegment_Value(const GC_MakeSegment &segment) {
  return std::unique_ptr<HandleGeomTrimmedCurve>(new opencascade::handle<Geom_TrimmedCurve>(segment.Value()));
}

inline std::unique_ptr<HandleGeom2d_TrimmedCurve> GCE2d_MakeSegment_point_point(const gp_Pnt2d &p1,
                                                                                const gp_Pnt2d &p2) {
  return std::unique_ptr<HandleGeom2d_TrimmedCurve>(
      new opencascade::handle<Geom2d_TrimmedCurve>(GCE2d_MakeSegment(p1, p2)));
}

// Arc stuff
inline std::unique_ptr<HandleGeomTrimmedCurve> GC_MakeArcOfCircle_Value(const GC_MakeArcOfCircle &arc) {
  return std::unique_ptr<HandleGeomTrimmedCurve>(new opencascade::handle<Geom_TrimmedCurve>(arc.Value()));
}

inline std::unique_ptr<gp_Pnt> BRepAdaptor_Curve_value(const BRepAdaptor_Curve &curve, const Standard_Real U) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(curve.Value(U)));
}

// BRepLib
inline bool BRepLibBuildCurves3d(const TopoDS_Shape &shape) { return BRepLib::BuildCurves3d(shape); }

inline void MakeThickSolidByJoin(BRepOffsetAPI_MakeThickSolid &make_thick_solid, const TopoDS_Shape &shape,
                                 const TopTools_ListOfShape &closing_faces, const Standard_Real offset,
                                 const Standard_Real tolerance) {
  make_thick_solid.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance);
}

// Geometric processing
inline const gp_Ax1 &gp_OX() { return gp::OX(); }
inline const gp_Ax1 &gp_OY() { return gp::OY(); }
inline const gp_Ax1 &gp_OZ() { return gp::OZ(); }

inline const gp_Dir &gp_DZ() { return gp::DZ(); }

inline std::unique_ptr<gp_Ax1> gp_Ax1_ctor(const gp_Pnt &origin, const gp_Dir &main_dir) {
  return std::unique_ptr<gp_Ax1>(new gp_Ax1(origin, main_dir));
}

inline std::unique_ptr<gp_Ax2> gp_Ax2_ctor(const gp_Pnt &origin, const gp_Dir &main_dir) {
  return std::unique_ptr<gp_Ax2>(new gp_Ax2(origin, main_dir));
}

inline std::unique_ptr<gp_Ax3> gp_Ax3_from_gp_Ax2(const gp_Ax2 &axis) {
  return std::unique_ptr<gp_Ax3>(new gp_Ax3(axis));
}

inline std::unique_ptr<gp_Dir> gp_Dir_ctor(double x, double y, double z) {
  return std::unique_ptr<gp_Dir>(new gp_Dir(x, y, z));
}

inline std::unique_ptr<gp_Dir2d> gp_Dir2d_ctor(double x, double y) {
  return std::unique_ptr<gp_Dir2d>(new gp_Dir2d(x, y));
}

inline std::unique_ptr<gp_Ax2d> gp_Ax2d_ctor(const gp_Pnt2d &point, const gp_Dir2d &dir) {
  return std::unique_ptr<gp_Ax2d>(new gp_Ax2d(point, dir));
}

// Law_Function stuff
inline std::unique_ptr<HandleLawFunction> Law_Function_to_handle(std::unique_ptr<Law_Function> law_function) {
  return std::unique_ptr<HandleLawFunction>(new HandleLawFunction(law_function.release()));
}

// Law_Interpol stuff
inline std::unique_ptr<Law_Function> Law_Interpol_into_Law_Function(std::unique_ptr<Law_Interpol> law_interpol) {
  return std::unique_ptr<Law_Function>(law_interpol.release());
}

// Shape stuff
inline const TopoDS_Vertex &TopoDS_cast_to_vertex(const TopoDS_Shape &shape) { return TopoDS::Vertex(shape); }
inline const TopoDS_Edge &TopoDS_cast_to_edge(const TopoDS_Shape &shape) { return TopoDS::Edge(shape); }
inline const TopoDS_Wire &TopoDS_cast_to_wire(const TopoDS_Shape &shape) { return TopoDS::Wire(shape); }
inline const TopoDS_Face &TopoDS_cast_to_face(const TopoDS_Shape &shape) { return TopoDS::Face(shape); }
inline const TopoDS_Shell &TopoDS_cast_to_shell(const TopoDS_Shape &shape) { return TopoDS::Shell(shape); }
inline const TopoDS_Solid &TopoDS_cast_to_solid(const TopoDS_Shape &shape) { return TopoDS::Solid(shape); }
inline const TopoDS_Compound &TopoDS_cast_to_compound(const TopoDS_Shape &shape) { return TopoDS::Compound(shape); }

inline const TopoDS_Shape &cast_vertex_to_shape(const TopoDS_Vertex &vertex) { return vertex; }
inline const TopoDS_Shape &cast_edge_to_shape(const TopoDS_Edge &edge) { return edge; }
inline const TopoDS_Shape &cast_wire_to_shape(const TopoDS_Wire &wire) { return wire; }
inline const TopoDS_Shape &cast_face_to_shape(const TopoDS_Face &face) { return face; }
inline const TopoDS_Shape &cast_shell_to_shape(const TopoDS_Shell &shell) { return shell; }
inline const TopoDS_Shape &cast_solid_to_shape(const TopoDS_Solid &solid) { return solid; }
inline const TopoDS_Shape &cast_compound_to_shape(const TopoDS_Compound &compound) { return compound; }

// Compound shapes
inline std::unique_ptr<TopoDS_Shape> TopoDS_Compound_as_shape(std::unique_ptr<TopoDS_Compound> compound) {
  return compound;
}

inline std::unique_ptr<TopoDS_Shape> TopoDS_Shell_as_shape(std::unique_ptr<TopoDS_Shell> shell) { return shell; }

inline const TopoDS_Builder &BRep_Builder_upcast_to_topods_builder(const BRep_Builder &builder) { return builder; }

// Transforms
inline std::unique_ptr<HandleGeomSurface> BRep_Tool_Surface(const TopoDS_Face &face) {
  return std::unique_ptr<HandleGeomSurface>(new opencascade::handle<Geom_Surface>(BRep_Tool::Surface(face)));
}

inline std::unique_ptr<HandleGeomCurve> BRep_Tool_Curve(const TopoDS_Edge &edge, Standard_Real &first,
                                                        Standard_Real &last) {
  return std::unique_ptr<HandleGeomCurve>(new opencascade::handle<Geom_Curve>(BRep_Tool::Curve(edge, first, last)));
}

inline std::unique_ptr<gp_Pnt> BRep_Tool_Pnt(const TopoDS_Vertex &vertex) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(BRep_Tool::Pnt(vertex)));
}

inline std::unique_ptr<gp_Trsf> TopLoc_Location_Transformation(const TopLoc_Location &location) {
  return std::unique_ptr<gp_Trsf>(new gp_Trsf(location.Transformation()));
}

inline std::unique_ptr<HandlePoly_Triangulation>
HandlePoly_Triangulation_ctor(std::unique_ptr<Poly_Triangulation> triangulation) {
  return std::unique_ptr<HandlePoly_Triangulation>(new HandlePoly_Triangulation(triangulation.release()));
}

inline std::unique_ptr<HandlePoly_Triangulation> BRep_Tool_Triangulation(const TopoDS_Face &face,
                                                                         TopLoc_Location &location) {
  return std::unique_ptr<HandlePoly_Triangulation>(
      new opencascade::handle<Poly_Triangulation>(BRep_Tool::Triangulation(face, location)));
}

inline std::unique_ptr<TopoDS_Shape> ExplorerCurrentShape(const TopExp_Explorer &explorer) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(explorer.Current()));
}

inline std::unique_ptr<TopoDS_Vertex> TopExp_FirstVertex(const TopoDS_Edge &edge) {
  return std::unique_ptr<TopoDS_Vertex>(new TopoDS_Vertex(TopExp::FirstVertex(edge)));
}

inline std::unique_ptr<TopoDS_Vertex> TopExp_LastVertex(const TopoDS_Edge &edge) {
  return std::unique_ptr<TopoDS_Vertex>(new TopoDS_Vertex(TopExp::LastVertex(edge)));
}

inline void TopExp_EdgeVertices(const TopoDS_Edge &edge, TopoDS_Vertex &vertex1, TopoDS_Vertex &vertex2) {
  return TopExp::Vertices(edge, vertex1, vertex2);
}

inline void TopExp_WireVertices(const TopoDS_Wire &wire, TopoDS_Vertex &vertex1, TopoDS_Vertex &vertex2) {
  return TopExp::Vertices(wire, vertex1, vertex2);
}

inline bool TopExp_CommonVertex(const TopoDS_Edge &edge1, const TopoDS_Edge &edge2, TopoDS_Vertex &vertex) {
  return TopExp::CommonVertex(edge1, edge2, vertex);
}

inline std::unique_ptr<TopoDS_Face> BRepIntCurveSurface_Inter_face(const BRepIntCurveSurface_Inter &intersector) {
  return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(intersector.Face()));
}

inline std::unique_ptr<gp_Pnt> BRepIntCurveSurface_Inter_point(const BRepIntCurveSurface_Inter &intersector) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(intersector.Pnt()));
}

// BRepFeat
inline std::unique_ptr<BRepFeat_MakeCylindricalHole> BRepFeat_MakeCylindricalHole_ctor() {
  return std::unique_ptr<BRepFeat_MakeCylindricalHole>(new BRepFeat_MakeCylindricalHole());
}

// Data Import
inline IFSelect_ReturnStatus read_step(STEPControl_Reader &reader, rust::String theFileName) {
  return reader.ReadFile(theFileName.c_str());
}

inline IFSelect_ReturnStatus read_iges(IGESControl_Reader &reader, rust::String theFileName) {
  return reader.ReadFile(theFileName.c_str());
}

inline std::unique_ptr<TopoDS_Shape> one_shape_step(const STEPControl_Reader &reader) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(reader.OneShape()));
}

inline std::unique_ptr<TopoDS_Shape> one_shape_iges(const IGESControl_Reader &reader) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(reader.OneShape()));
}

// Data Export
inline IFSelect_ReturnStatus transfer_shape(STEPControl_Writer &writer, const TopoDS_Shape &theShape) {
  return writer.Transfer(theShape, STEPControl_AsIs);
}

inline void compute_model(IGESControl_Writer &writer) { writer.ComputeModel(); }

inline bool add_shape(IGESControl_Writer &writer, const TopoDS_Shape &theShape) { return writer.AddShape(theShape); }

inline IFSelect_ReturnStatus write_step(STEPControl_Writer &writer, rust::String theFileName) {
  return writer.Write(theFileName.c_str());
}

inline bool write_iges(IGESControl_Writer &writer, rust::String theFileName) {
  return writer.Write(theFileName.c_str());
}

inline bool write_stl(StlAPI_Writer &writer, const TopoDS_Shape &theShape, rust::String theFileName) {
  return writer.Write(theShape, theFileName.c_str());
}

inline std::unique_ptr<gp_Dir> Poly_Triangulation_Normal(const Poly_Triangulation &triangulation,
                                                         const Standard_Integer index) {
  return std::unique_ptr<gp_Dir>(new gp_Dir(triangulation.Normal(index)));
}

inline std::unique_ptr<gp_Pnt> Poly_Triangulation_Node(const Poly_Triangulation &triangulation,
                                                       const Standard_Integer index) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(triangulation.Node(index)));
}

inline std::unique_ptr<gp_Pnt2d> Poly_Triangulation_UV(const Poly_Triangulation &triangulation,
                                                       const Standard_Integer index) {
  return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(triangulation.UVNode(index)));
}

inline void compute_normals(const TopoDS_Face &face, const Handle(Poly_Triangulation) & triangulation) {
  BRepLib_ToolTriangulatedShape::ComputeNormals(face, triangulation);
}

// Shape Properties
inline std::unique_ptr<gp_Pnt> GProp_GProps_CentreOfMass(const GProp_GProps &props) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(props.CentreOfMass()));
}

inline void BRepGProp_LinearProperties(const TopoDS_Shape &shape, GProp_GProps &props) {
  BRepGProp::LinearProperties(shape, props);
}

inline void BRepGProp_SurfaceProperties(const TopoDS_Shape &shape, GProp_GProps &props) {
  BRepGProp::SurfaceProperties(shape, props);
}

inline void BRepGProp_VolumeProperties(const TopoDS_Shape &shape, GProp_GProps &props) {
  BRepGProp::VolumeProperties(shape, props);
}

// Fillets
inline std::unique_ptr<TopoDS_Edge> BRepFilletAPI_MakeFillet2d_add_fillet(BRepFilletAPI_MakeFillet2d &make_fillet,
                                                                          const TopoDS_Vertex &vertex,
                                                                          Standard_Real radius) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make_fillet.AddFillet(vertex, radius)));
}

// Chamfers
inline std::unique_ptr<TopoDS_Edge>
BRepFilletAPI_MakeFillet2d_add_chamfer(BRepFilletAPI_MakeFillet2d &make_fillet, const TopoDS_Edge &edge1,
                                       const TopoDS_Edge &edge2, const Standard_Real dist1, const Standard_Real dist2) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make_fillet.AddChamfer(edge1, edge2, dist1, dist2)));
}

inline std::unique_ptr<TopoDS_Edge>
BRepFilletAPI_MakeFillet2d_add_chamfer_angle(BRepFilletAPI_MakeFillet2d &make_fillet, const TopoDS_Edge &edge,
                                             const TopoDS_Vertex &vertex, const Standard_Real dist,
                                             const Standard_Real angle) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make_fillet.AddChamfer(edge, vertex, dist, angle)));
}

// BRepTools
inline std::unique_ptr<TopoDS_Wire> outer_wire(const TopoDS_Face &face) {
  return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(BRepTools::OuterWire(face)));
}

// Collections
inline void map_shapes(const TopoDS_Shape &S, const TopAbs_ShapeEnum T, TopTools_IndexedMapOfShape &M) {
  TopExp::MapShapes(S, T, M);
}

inline void map_shapes_and_ancestors(const TopoDS_Shape &S, const TopAbs_ShapeEnum TS, const TopAbs_ShapeEnum TA,
                                     TopTools_IndexedDataMapOfShapeListOfShape &M) {
  TopExp::MapShapesAndAncestors(S, TS, TA, M);
}

inline void map_shapes_and_unique_ancestors(const TopoDS_Shape &S, const TopAbs_ShapeEnum TS, const TopAbs_ShapeEnum TA,
                                            TopTools_IndexedDataMapOfShapeListOfShape &M) {
  TopExp::MapShapesAndUniqueAncestors(S, TS, TA, M);
}

inline std::unique_ptr<gp_Dir> TColgp_Array1OfDir_Value(const TColgp_Array1OfDir &array, Standard_Integer index) {
  return std::unique_ptr<gp_Dir>(new gp_Dir(array.Value(index)));
}

inline std::unique_ptr<gp_Pnt2d> TColgp_Array1OfPnt2d_Value(const TColgp_Array1OfPnt2d &array, Standard_Integer index) {
  return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(array.Value(index)));
}

inline std::unique_ptr<gp_Pnt> TColgp_HArray1OfPnt_Value(const TColgp_HArray1OfPnt &array, Standard_Integer index) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(array.Value(index)));
}

inline void connect_edges_to_wires(HandleTopTools_HSequenceOfShape &edges, const Standard_Real toler,
                                   const Standard_Boolean shared, HandleTopTools_HSequenceOfShape &wires) {
  ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, toler, shared, wires);
}

inline std::unique_ptr<HandleTopTools_HSequenceOfShape> new_HandleTopTools_HSequenceOfShape() {
  auto sequence = new TopTools_HSequenceOfShape();
  auto handle = new opencascade::handle<TopTools_HSequenceOfShape>(sequence);

  return std::unique_ptr<HandleTopTools_HSequenceOfShape>(handle);
}

inline void TopTools_HSequenceOfShape_append(HandleTopTools_HSequenceOfShape &handle, const TopoDS_Shape &shape) {
  handle->Append(shape);
}

inline Standard_Integer TopTools_HSequenceOfShape_length(const HandleTopTools_HSequenceOfShape &handle) {
  return handle->Length();
}

inline const TopoDS_Shape &TopTools_HSequenceOfShape_value(const HandleTopTools_HSequenceOfShape &handle,
                                                           Standard_Integer index) {
  return handle->Value(index);
}

// BRep Algo API
inline std::unique_ptr<BRepAlgoAPI_BuilderAlgo>
cast_section_to_builderalgo(std::unique_ptr<BRepAlgoAPI_Section> section) {
  return section;
}
// namespace BRepAlgoAPI

// Bnd_Box
inline std::unique_ptr<Bnd_Box> Bnd_Box_ctor() { return std::unique_ptr<Bnd_Box>(new Bnd_Box()); }
inline std::unique_ptr<gp_Pnt> Bnd_Box_CornerMin(const Bnd_Box &box) {
  auto p = box.CornerMin();
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(p));
}
inline std::unique_ptr<gp_Pnt> Bnd_Box_CornerMax(const Bnd_Box &box) {
  auto p = box.CornerMax();
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(p));
}

// BRepBndLib
inline void BRepBndLib_Add(const TopoDS_Shape &shape, Bnd_Box &box, const Standard_Boolean useTriangulation) {
  BRepBndLib::Add(shape, box, useTriangulation);
}

// BRepCheck — Shape validation
inline bool BRepCheck_IsValid(const TopoDS_Shape &shape) {
  BRepCheck_Analyzer analyzer(shape, Standard_True);
  return analyzer.IsValid();
}

// Boolean fuzzy tolerance wrappers
inline void Fuse_SetFuzzyValue(BRepAlgoAPI_Fuse &op, Standard_Real fuzz) { op.SetFuzzyValue(fuzz); }
inline void Cut_SetFuzzyValue(BRepAlgoAPI_Cut &op, Standard_Real fuzz) { op.SetFuzzyValue(fuzz); }
inline void Common_SetFuzzyValue(BRepAlgoAPI_Common &op, Standard_Real fuzz) { op.SetFuzzyValue(fuzz); }

// Boolean error reporting wrappers
inline bool Fuse_HasErrors(const BRepAlgoAPI_Fuse &op) { return op.HasErrors(); }
inline bool Cut_HasErrors(const BRepAlgoAPI_Cut &op) { return op.HasErrors(); }
inline bool Common_HasErrors(const BRepAlgoAPI_Common &op) { return op.HasErrors(); }
inline bool Fuse_HasWarnings(const BRepAlgoAPI_Fuse &op) { return op.HasWarnings(); }
inline bool Cut_HasWarnings(const BRepAlgoAPI_Cut &op) { return op.HasWarnings(); }
inline bool Common_HasWarnings(const BRepAlgoAPI_Common &op) { return op.HasWarnings(); }

// Safe wrappers that catch C++ exceptions before they cross the FFI boundary.
// OCCT can throw Standard_Failure (or subclasses) from shell/fillet/chamfer operations
// on certain geometry combinations. Without these wrappers, the exception propagates
// through cxx → Rust → panic_cannot_unwind → process abort (STATUS_STACK_BUFFER_OVERRUN on Windows).

static bool _do_thick_solid_inner(
    BRepOffsetAPI_MakeThickSolid &make_thick_solid,
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    Standard_Real offset,
    Standard_Real tolerance) {
  try {
    make_thick_solid.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance);
    return make_thick_solid.IsDone();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
#include <windows.h>
#include <excpt.h>
inline bool Safe_MakeThickSolidByJoin(
    BRepOffsetAPI_MakeThickSolid &make_thick_solid,
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    Standard_Real offset,
    Standard_Real tolerance) {
  __try {
    return _do_thick_solid_inner(make_thick_solid, shape, closing_faces, offset, tolerance);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_MakeThickSolidByJoin(
    BRepOffsetAPI_MakeThickSolid &make_thick_solid,
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    Standard_Real offset,
    Standard_Real tolerance) {
  try {
    make_thick_solid.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance);
    return make_thick_solid.IsDone();
  } catch (...) {
    return false;
  }
}
#endif

// GeomAbs_Intersection join type — sharp edges at concave junctions instead of
// fillet-like arcs. Preferred for cylindrical holes and similar geometry where
// the user expects crisp inner edges from the shell operation.
static bool _do_thick_solid_intersection_inner(
    BRepOffsetAPI_MakeThickSolid &make_thick_solid,
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    Standard_Real offset,
    Standard_Real tolerance) {
  try {
    make_thick_solid.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance,
        BRepOffset_Skin, Standard_False, Standard_False, GeomAbs_Intersection);
    return make_thick_solid.IsDone();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_MakeThickSolidByJoinIntersection(
    BRepOffsetAPI_MakeThickSolid &make_thick_solid,
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    Standard_Real offset,
    Standard_Real tolerance) {
  __try {
    return _do_thick_solid_intersection_inner(make_thick_solid, shape, closing_faces, offset, tolerance);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_MakeThickSolidByJoinIntersection(
    BRepOffsetAPI_MakeThickSolid &make_thick_solid,
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    Standard_Real offset,
    Standard_Real tolerance) {
  try {
    make_thick_solid.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance,
        BRepOffset_Skin, Standard_False, Standard_False, GeomAbs_Intersection);
    return make_thick_solid.IsDone();
  } catch (...) {
    return false;
  }
}
#endif

static bool _do_fillet_build_inner(BRepFilletAPI_MakeFillet &fillet) {
  try {
    Message_ProgressRange progress;
    fillet.Build(progress);
    return fillet.IsDone();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_Fillet_Build(BRepFilletAPI_MakeFillet &fillet) {
  __try {
    return _do_fillet_build_inner(fillet);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_Fillet_Build(BRepFilletAPI_MakeFillet &fillet) {
  try {
    Message_ProgressRange progress;
    fillet.Build(progress);
    return fillet.IsDone();
  } catch (...) {
    return false;
  }
}
#endif

// Safe wrapper for adding an edge to a fillet builder.
// OCCT BRepFilletAPI_MakeFillet::Add() can throw on degenerate edges from complex booleans.
static bool _do_fillet_add_edge_inner(BRepFilletAPI_MakeFillet &fillet, Standard_Real radius, const TopoDS_Edge &edge) {
  try {
    fillet.Add(radius, edge);
    return true;
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_Fillet_AddEdge(BRepFilletAPI_MakeFillet &fillet, Standard_Real radius, const TopoDS_Edge &edge) {
  __try {
    return _do_fillet_add_edge_inner(fillet, radius, edge);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_Fillet_AddEdge(BRepFilletAPI_MakeFillet &fillet, Standard_Real radius, const TopoDS_Edge &edge) {
  try {
    fillet.Add(radius, edge);
    return true;
  } catch (...) {
    return false;
  }
}
#endif

static bool _do_chamfer_build_inner(BRepFilletAPI_MakeChamfer &chamfer) {
  try {
    Message_ProgressRange progress;
    chamfer.Build(progress);
    return chamfer.IsDone();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_Chamfer_Build(BRepFilletAPI_MakeChamfer &chamfer) {
  __try {
    return _do_chamfer_build_inner(chamfer);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_Chamfer_Build(BRepFilletAPI_MakeChamfer &chamfer) {
  try {
    Message_ProgressRange progress;
    chamfer.Build(progress);
    return chamfer.IsDone();
  } catch (...) {
    return false;
  }
}
#endif

// Safe wrapper for adding an edge to a chamfer builder.
static bool _do_chamfer_add_edge_inner(BRepFilletAPI_MakeChamfer &chamfer, Standard_Real dist, const TopoDS_Edge &edge) {
  try {
    chamfer.Add(dist, edge);
    return true;
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_Chamfer_AddEdge(BRepFilletAPI_MakeChamfer &chamfer, Standard_Real dist, const TopoDS_Edge &edge) {
  __try {
    return _do_chamfer_add_edge_inner(chamfer, dist, edge);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_Chamfer_AddEdge(BRepFilletAPI_MakeChamfer &chamfer, Standard_Real dist, const TopoDS_Edge &edge) {
  try {
    chamfer.Add(dist, edge);
    return true;
  } catch (...) {
    return false;
  }
}
#endif

// Safe wrapper for BRepCheck_Analyzer (used by is_shape_valid())
// Can throw on severely degenerate geometry from fillet/chamfer/boolean results.
static bool _do_brep_check_inner(const TopoDS_Shape &shape) {
  try {
    BRepCheck_Analyzer analyzer(shape, Standard_True);
    return analyzer.IsValid();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_BRepCheck_IsValid(const TopoDS_Shape &shape) {
  __try {
    return _do_brep_check_inner(shape);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_BRepCheck_IsValid(const TopoDS_Shape &shape) {
  try {
    BRepCheck_Analyzer analyzer(shape, Standard_True);
    return analyzer.IsValid();
  } catch (...) {
    return false;
  }
}
#endif

// Safe wrapper for ShapeUpgrade_UnifySameDomain::Build() (used by shape.clean())
// Can throw on degenerate fillet/chamfer output geometry.
// Returns nullptr on failure instead of crashing the process.
static std::unique_ptr<TopoDS_Shape> _do_clean_shape_inner(const TopoDS_Shape &shape) {
  try {
    ShapeUpgrade_UnifySameDomain upgrader(shape, Standard_True, Standard_True, Standard_True);
    upgrader.AllowInternalEdges(Standard_False);
    upgrader.Build();
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(upgrader.Shape()));
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<TopoDS_Shape> Safe_Clean_Shape(const TopoDS_Shape &shape) {
  __try {
    return _do_clean_shape_inner(shape);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<TopoDS_Shape> Safe_Clean_Shape(const TopoDS_Shape &shape) {
  try {
    ShapeUpgrade_UnifySameDomain upgrader(shape, Standard_True, Standard_True, Standard_True);
    upgrader.AllowInternalEdges(Standard_False);
    upgrader.Build();
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(upgrader.Shape()));
  } catch (...) {
    return nullptr;
  }
}
#endif

// Edges-only clean: unify edges but NOT faces. Avoids creating complex
// face boundary wires that BRepMesh cannot tessellate (sphere+fillet+chamfer).
static std::unique_ptr<TopoDS_Shape> _do_clean_shape_edges_only_inner(const TopoDS_Shape &shape) {
  try {
    ShapeUpgrade_UnifySameDomain upgrader(shape, Standard_True, Standard_False, Standard_True);
    upgrader.AllowInternalEdges(Standard_False);
    upgrader.Build();
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(upgrader.Shape()));
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<TopoDS_Shape> Safe_Clean_Shape_EdgesOnly(const TopoDS_Shape &shape) {
  __try {
    return _do_clean_shape_edges_only_inner(shape);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<TopoDS_Shape> Safe_Clean_Shape_EdgesOnly(const TopoDS_Shape &shape) {
  try {
    ShapeUpgrade_UnifySameDomain upgrader(shape, Standard_True, Standard_False, Standard_True);
    upgrader.AllowInternalEdges(Standard_False);
    upgrader.Build();
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(upgrader.Shape()));
  } catch (...) {
    return nullptr;
  }
}
#endif

// Shape copy via BRepBuilderAPI_Copy (deep copy including geometry)
inline std::unique_ptr<TopoDS_Shape> Copy_Shape(const TopoDS_Shape &shape) {
  BRepBuilderAPI_Copy copier(shape, Standard_True, Standard_False);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(copier.Shape()));
}

// Translate a shape copy by (dx, dy, dz)
inline std::unique_ptr<TopoDS_Shape> Transform_Translate(
    const TopoDS_Shape &shape,
    double dx, double dy, double dz) {
  gp_Trsf trsf;
  trsf.SetTranslation(gp_Vec(dx, dy, dz));
  BRepBuilderAPI_Transform builder(shape, trsf, Standard_True);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(builder.Shape()));
}

// Rotate a shape copy around an axis (origin + direction) by angle_radians
inline std::unique_ptr<TopoDS_Shape> Transform_Rotate(
    const TopoDS_Shape &shape,
    double ax_x, double ax_y, double ax_z,
    double dir_x, double dir_y, double dir_z,
    double angle_radians) {
  gp_Ax1 axis(gp_Pnt(ax_x, ax_y, ax_z), gp_Dir(dir_x, dir_y, dir_z));
  gp_Trsf trsf;
  trsf.SetRotation(axis, angle_radians);
  BRepBuilderAPI_Transform builder(shape, trsf, Standard_True);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(builder.Shape()));
}

// Mirror a shape copy across a plane (origin + normal)
inline std::unique_ptr<TopoDS_Shape> Transform_Mirror(
    const TopoDS_Shape &shape,
    double pl_x, double pl_y, double pl_z,
    double pn_x, double pn_y, double pn_z) {
  gp_Ax2 plane(gp_Pnt(pl_x, pl_y, pl_z), gp_Dir(pn_x, pn_y, pn_z));
  gp_Trsf trsf;
  trsf.SetMirror(plane);
  BRepBuilderAPI_Transform builder(shape, trsf, Standard_True);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(builder.Shape()));
}

// Safe wrapper for BRepOffsetAPI_ThruSections::Build() (used by loft)
// Can throw on incompatible profiles or degenerate geometry.
static bool _do_thru_sections_build_inner(BRepOffsetAPI_ThruSections &loft) {
  try {
    Message_ProgressRange progress;
    loft.Build(progress);
    return loft.IsDone();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_ThruSections_Build(BRepOffsetAPI_ThruSections &loft) {
  __try {
    return _do_thru_sections_build_inner(loft);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_ThruSections_Build(BRepOffsetAPI_ThruSections &loft) {
  try {
    Message_ProgressRange progress;
    loft.Build(progress);
    return loft.IsDone();
  } catch (...) {
    return false;
  }
}
#endif

// Safe wrapper for BRepOffsetAPI_MakePipe::Build() (used by sweep)
// Can throw on self-intersecting paths or incompatible profile/path.
static bool _do_make_pipe_build_inner(BRepOffsetAPI_MakePipe &pipe) {
  try {
    Message_ProgressRange progress;
    pipe.Build(progress);
    return pipe.IsDone();
  } catch (...) {
    return false;
  }
}

#ifdef _WIN32
inline bool Safe_MakePipe_Build(BRepOffsetAPI_MakePipe &pipe) {
  __try {
    return _do_make_pipe_build_inner(pipe);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
#else
inline bool Safe_MakePipe_Build(BRepOffsetAPI_MakePipe &pipe) {
  try {
    Message_ProgressRange progress;
    pipe.Build(progress);
    return pipe.IsDone();
  } catch (...) {
    return false;
  }
}
#endif

// =============================================================================
// ShapeHealing — topology repair for downstream fillet/chamfer success
// =============================================================================
// Removes micro-edges, fixes wire gaps, and performs general topology repair.
// This improves fillet/chamfer success on shapes produced by complex boolean ops.

static std::unique_ptr<TopoDS_Shape> _do_shape_heal_inner(const TopoDS_Shape &shape) {
  try {
    // Step 1: Fix wireframe — remove small edges, fix wire gaps
    Handle(ShapeFix_Wireframe) wirefix = new ShapeFix_Wireframe(shape);
    wirefix->SetPrecision(1e-4);
    wirefix->FixSmallEdges();
    wirefix->FixWireGaps();
    TopoDS_Shape intermediate = wirefix->Shape();

    // Step 2: General shape healing
    Handle(ShapeFix_Shape) shapefix = new ShapeFix_Shape(intermediate);
    shapefix->Perform();

    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(shapefix->Shape()));
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<TopoDS_Shape> Safe_ShapeHeal(const TopoDS_Shape &shape) {
  __try {
    return _do_shape_heal_inner(shape);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<TopoDS_Shape> Safe_ShapeHeal(const TopoDS_Shape &shape) {
  try {
    Handle(ShapeFix_Wireframe) wirefix = new ShapeFix_Wireframe(shape);
    wirefix->SetPrecision(1e-4);
    wirefix->FixSmallEdges();
    wirefix->FixWireGaps();
    TopoDS_Shape intermediate = wirefix->Shape();
    Handle(ShapeFix_Shape) shapefix = new ShapeFix_Shape(intermediate);
    shapefix->Perform();
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(shapefix->Shape()));
  } catch (...) {
    return nullptr;
  }
}
#endif

// =============================================================================
// Parallel boolean operations — set options BEFORE Build
// =============================================================================
// The 2-argument BRepAlgoAPI_Fuse/Cut/Common constructors call Build() internally,
// so SetFuzzyValue/SetRunParallel called after construction have no effect.
// These wrappers use the empty constructor + SetArguments/SetTools + options + Build.

static std::unique_ptr<BRepAlgoAPI_Fuse> _do_fuse_with_options_inner(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  try {
    auto fuse = std::make_unique<BRepAlgoAPI_Fuse>();
    TopTools_ListOfShape args, tools;
    args.Append(shape1);
    tools.Append(shape2);
    fuse->SetArguments(args);
    fuse->SetTools(tools);
    fuse->SetFuzzyValue(fuzzyValue);
    fuse->SetRunParallel(isParallel);
    Message_ProgressRange progress;
    fuse->Build(progress);
    return fuse;
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<BRepAlgoAPI_Fuse> Fuse_WithOptions(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  __try {
    return _do_fuse_with_options_inner(shape1, shape2, fuzzyValue, isParallel);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<BRepAlgoAPI_Fuse> Fuse_WithOptions(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  return _do_fuse_with_options_inner(shape1, shape2, fuzzyValue, isParallel);
}
#endif

static std::unique_ptr<BRepAlgoAPI_Cut> _do_cut_with_options_inner(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  try {
    auto cut = std::make_unique<BRepAlgoAPI_Cut>();
    TopTools_ListOfShape args, tools;
    args.Append(shape1);
    tools.Append(shape2);
    cut->SetArguments(args);
    cut->SetTools(tools);
    cut->SetFuzzyValue(fuzzyValue);
    cut->SetRunParallel(isParallel);
    Message_ProgressRange progress;
    cut->Build(progress);
    return cut;
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<BRepAlgoAPI_Cut> Cut_WithOptions(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  __try {
    return _do_cut_with_options_inner(shape1, shape2, fuzzyValue, isParallel);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<BRepAlgoAPI_Cut> Cut_WithOptions(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  return _do_cut_with_options_inner(shape1, shape2, fuzzyValue, isParallel);
}
#endif

static std::unique_ptr<BRepAlgoAPI_Common> _do_common_with_options_inner(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  try {
    auto common = std::make_unique<BRepAlgoAPI_Common>();
    TopTools_ListOfShape args, tools;
    args.Append(shape1);
    tools.Append(shape2);
    common->SetArguments(args);
    common->SetTools(tools);
    common->SetFuzzyValue(fuzzyValue);
    common->SetRunParallel(isParallel);
    Message_ProgressRange progress;
    common->Build(progress);
    return common;
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<BRepAlgoAPI_Common> Common_WithOptions(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  __try {
    return _do_common_with_options_inner(shape1, shape2, fuzzyValue, isParallel);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<BRepAlgoAPI_Common> Common_WithOptions(
    const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
    Standard_Real fuzzyValue, Standard_Boolean isParallel) {
  return _do_common_with_options_inner(shape1, shape2, fuzzyValue, isParallel);
}
#endif

// =============================================================================
// Parallel tessellation — BRepMesh_IncrementalMesh with isInParallel flag
// =============================================================================

// =============================================================================
// Draft angle — BRepOffsetAPI_DraftAngle with safe wrappers
// =============================================================================
// Applies a draft angle to specified faces. The pull direction and neutral plane
// define how the taper is applied (e.g. Z-up pull with XY neutral plane for mold draft).
// Returns nullptr on failure.

static std::unique_ptr<TopoDS_Shape> _do_draft_angle_inner(
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &faces,
    double dir_x, double dir_y, double dir_z,
    double angle_radians,
    double plane_px, double plane_py, double plane_pz,
    double plane_nx, double plane_ny, double plane_nz) {
  try {
    gp_Dir direction(dir_x, dir_y, dir_z);
    gp_Pln neutral_plane(gp_Pnt(plane_px, plane_py, plane_pz),
                          gp_Dir(plane_nx, plane_ny, plane_nz));

    BRepOffsetAPI_DraftAngle drafter(shape);

    TopTools_ListIteratorOfListOfShape iter(faces);
    for (; iter.More(); iter.Next()) {
      const TopoDS_Face &face = TopoDS::Face(iter.Value());
      drafter.Add(face, direction, angle_radians, neutral_plane);
    }

    Message_ProgressRange progress;
    drafter.Build(progress);
    if (!drafter.IsDone()) {
      return nullptr;
    }
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(drafter.Shape()));
  } catch (...) {
    return nullptr;
  }
}

#ifdef _WIN32
inline std::unique_ptr<TopoDS_Shape> Safe_DraftAngle(
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &faces,
    double dir_x, double dir_y, double dir_z,
    double angle_radians,
    double plane_px, double plane_py, double plane_pz,
    double plane_nx, double plane_ny, double plane_nz) {
  __try {
    return _do_draft_angle_inner(shape, faces,
        dir_x, dir_y, dir_z, angle_radians,
        plane_px, plane_py, plane_pz, plane_nx, plane_ny, plane_nz);
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}
#else
inline std::unique_ptr<TopoDS_Shape> Safe_DraftAngle(
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &faces,
    double dir_x, double dir_y, double dir_z,
    double angle_radians,
    double plane_px, double plane_py, double plane_pz,
    double plane_nx, double plane_ny, double plane_nz) {
  return _do_draft_angle_inner(shape, faces,
      dir_x, dir_y, dir_z, angle_radians,
      plane_px, plane_py, plane_pz, plane_nx, plane_ny, plane_nz);
}
#endif

inline std::unique_ptr<BRepMesh_IncrementalMesh> BRepMesh_IncrementalMesh_ctor_parallel(
    const TopoDS_Shape &shape, Standard_Real deflection) {
  return std::unique_ptr<BRepMesh_IncrementalMesh>(
      new BRepMesh_IncrementalMesh(shape, deflection, Standard_False, 0.5, Standard_True));
}
