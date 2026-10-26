// ParaForge's guarded shape constructors.
//
// The binding builds wires, faces, extrusions, revolutions and pipes inside OCCT
// constructors and accessors that can throw (StdFail_NotDone from Shape() / Wire(),
// Standard_ConstructionError, Standard_DomainError, ...). A throw that reaches a cxx
// shim ends the process. Each PF_ function below builds the same thing as the binding
// call it names, with the same arguments, but returns a null pointer instead: on any
// throw, when the builder is not done, or when the result is null or holds no face. No
// result is cast to a solid (the binding's cast throws on anything else).
//
// OCCT's libraries are built here with BUILD_RELEASE_DISABLE_EXCEPTIONS, which compiles
// out the input checks inside them (`*_Raise_if`): bad input carries on instead of
// throwing. Checks in OCCT's inline headers (gp_Dir, the TopoDS:: casts) are compiled into
// this file and do still throw. Each function repeats the library checks it relies on, at
// OCCT's own threshold, and refuses numbers that are not finite.
//
// C++11 only: build.rs compiles with -std=c++11, and MSVC cannot tell (it has no C++11
// mode), so a C++14 slip shows up on macOS only.

#ifndef PARAFORGE_KERNEL_HXX
#define PARAFORGE_KERNEL_HXX

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rust/cxx.h"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IMeshData_Status.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <Message_ProgressScope.hxx>
#include <Precision.hxx>
#include <STEPControl_Reader.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeAnalysis_ShapeTolerance.hxx>
#include <ShapeFix_Solid.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_MapIteratorOfMapOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#ifdef _MSC_VER
#include <windows.h>
#include <excpt.h>
#include <malloc.h>
#endif

static const Standard_Real PF_TWO_PI = 6.28318530717958647692528676655900577;

static bool _pf_finite3(Standard_Real x, Standard_Real y, Standard_Real z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// Whether (x, y, z) is finite and longer than `minimum`. Huge finite components overflow
// the length to infinity, which a plain `>` would let through.
static bool _pf_length_above(Standard_Real x, Standard_Real y, Standard_Real z,
                             Standard_Real minimum) {
  if (!_pf_finite3(x, y, z)) {
    return false;
  }
  const Standard_Real length = std::sqrt(x * x + y * y + z * z);
  return std::isfinite(length) && length > minimum;
}

// A result that is null or holds no face (an empty compound) is not a shape.
static std::unique_ptr<TopoDS_Shape> _pf_shape_with_faces(const TopoDS_Shape &shape) {
  if (shape.IsNull() || !TopExp_Explorer(shape, TopAbs_FACE).More()) {
    return nullptr;
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(shape));
}

// Wire::from_unordered_edges(edges, EdgeConnection::Fuzzy { tolerance }):
// ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, tolerance, shared = false), then
// BRepBuilderAPI_MakeWire with every wire that returned. Edges that make more than one
// piece leave MakeWire not done, where the binding's Wire() throws.
static std::unique_ptr<TopoDS_Wire> _pf_make_wire_inner(
    const Handle(TopTools_HSequenceOfShape) &edges, Standard_Real tolerance) {
  try {
    // With no edges ConnectEdgesToWires returns without filling `wires`.
    if (edges.IsNull() || edges->Length() < 1 || !std::isfinite(tolerance) || tolerance < 0.0) {
      return nullptr;
    }
    // ConnectEdgesToWires reverses edges in the sequence it is given: work on a copy. It
    // adds each item to a wire with the compatibility check compiled out, so anything that
    // is not an edge is refused here.
    Handle(TopTools_HSequenceOfShape) work = new TopTools_HSequenceOfShape();
    for (Standard_Integer i = 1; i <= edges->Length(); i++) {
      const TopoDS_Shape &edge = edges->Value(i);
      if (edge.IsNull() || edge.ShapeType() != TopAbs_EDGE) {
        return nullptr;
      }
      work->Append(edge);
    }
    Handle(TopTools_HSequenceOfShape) wires = new TopTools_HSequenceOfShape();
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(work, tolerance, Standard_False, wires);
    if (wires.IsNull() || wires->Length() < 1) {
      return nullptr;
    }
    BRepBuilderAPI_MakeWire make_wire;
    for (Standard_Integer i = 1; i <= wires->Length(); i++) {
      make_wire.Add(TopoDS::Wire(wires->Value(i)));
    }
    if (!make_wire.IsDone()) {
      return nullptr;
    }
    const TopoDS_Wire &wire = make_wire.Wire();
    if (wire.IsNull()) {
      return nullptr;
    }
    return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(wire));
  } catch (...) {
    return nullptr;
  }
}

// Wire::to_face(): BRepBuilderAPI_MakeFace(wire, OnlyPlane = false). Its Face() does not
// check IsDone() and hands back a null face, so it is checked here. An open wire is
// refused: OCCT would make a face from it, and an open face is never a sketch profile.
static std::unique_ptr<TopoDS_Face> _pf_make_face_inner(const TopoDS_Wire &wire) {
  try {
    if (wire.IsNull() || !BRep_Tool::IsClosed(wire)) {
      return nullptr;
    }
    BRepBuilderAPI_MakeFace make_face(wire, Standard_False);
    if (!make_face.IsDone()) {
      return nullptr;
    }
    const TopoDS_Face &face = make_face.Face();
    if (face.IsNull()) {
      return nullptr;
    }
    return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(face));
  } catch (...) {
    return nullptr;
  }
}

// Face::extrude / CompoundFace::extrude: BRepPrimAPI_MakePrism(shape, vector, Copy = false,
// Canonize = true), which builds in its constructor. Its Build() always reports done, so
// only the result tells.
static std::unique_ptr<TopoDS_Shape> _pf_make_prism_inner(
    const TopoDS_Shape &shape, Standard_Real dx, Standard_Real dy, Standard_Real dz) {
  try {
    // BRepSweep_Prism's own check: longer than Precision::Confusion().
    if (shape.IsNull() || !_pf_length_above(dx, dy, dz, Precision::Confusion())) {
      return nullptr;
    }
    BRepPrimAPI_MakePrism make_prism(shape, gp_Vec(dx, dy, dz), Standard_False, Standard_True);
    if (!make_prism.IsDone()) {
      return nullptr;
    }
    return _pf_shape_with_faces(make_prism.Shape());
  } catch (...) {
    return nullptr;
  }
}

// Face::revolve / CompoundFace::revolve: BRepPrimAPI_MakeRevol(shape, axis, angle,
// Copy = false), which builds in its constructor. A profile the kernel finds invalid for
// the axis (it crosses the axis) leaves the builder not done with a null shape, and the
// binding's Shape() throws.
static std::unique_ptr<TopoDS_Shape> _pf_make_revol_inner(
    const TopoDS_Shape &shape,
    Standard_Real ox, Standard_Real oy, Standard_Real oz,
    Standard_Real dx, Standard_Real dy, Standard_Real dz,
    Standard_Real angle_radians) {
  try {
    // gp_Dir's own check (longer than gp::Resolution()), and BRepSweep_Revol's: an angle
    // above Precision::Angular(). OCCT brings a larger angle down to one turn by taking
    // 2 pi off in a loop, which runs for minutes, or for ever, on a huge one: one turn is
    // the most accepted.
    if (shape.IsNull() || !_pf_finite3(ox, oy, oz) ||
        !_pf_length_above(dx, dy, dz, gp::Resolution()) || !std::isfinite(angle_radians) ||
        !(std::fabs(angle_radians) > Precision::Angular()) ||
        std::fabs(angle_radians) > PF_TWO_PI + Precision::Angular()) {
      return nullptr;
    }
    gp_Ax1 axis(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz));
    BRepPrimAPI_MakeRevol make_revol(shape, axis, angle_radians, Standard_False);
    if (!make_revol.IsDone()) {
      return nullptr;
    }
    return _pf_shape_with_faces(make_revol.Shape());
  } catch (...) {
    return nullptr;
  }
}

// Face::sweep_along: BRepOffsetAPI_MakePipe(spine, profile), which builds in its
// constructor (and throws Standard_DomainError on a solid profile, among others).
static std::unique_ptr<TopoDS_Shape> _pf_make_pipe_inner(
    const TopoDS_Wire &spine, const TopoDS_Shape &profile) {
  try {
    if (spine.IsNull() || profile.IsNull()) {
      return nullptr;
    }
    BRepOffsetAPI_MakePipe make_pipe(spine, profile);
    if (!make_pipe.IsDone()) {
      return nullptr;
    }
    return _pf_shape_with_faces(make_pipe.Shape());
  } catch (...) {
    return nullptr;
  }
}

// The export's mesh (ParaForge queue row PE). The binding's BRepMesh_IncrementalMesh_ctor_parallel
// fixes the angle at 0.5 rad and meshes the shape in place, where OCCT keeps any existing
// triangulation finer than 1.1 times the request and never compares the angle
// (BRepMesh_ModelPreProcessor.cxx), so an export at another quality needs its own copy:
// BRepBuilderAPI_Copy(shape, copyGeom = true, copyMesh = false), meshed with the given deviation
// and angle on faces and edges alike (the 5-argument constructor sets AngleInterior, which
// B-spline faces use, to twice the angle). Edges still get half of both (BRepMesh_CurveTessellator).
// One hang at a small angle is reported upstream (tracker 33082), so the run stops at a time
// limit; OCCT checks it between faces and deflection passes, not while it splits an edge.
//
// `status` says why the result is null: 0 meshed, 1 input refused, 2 OCCT threw or the copy
// failed, 3 stopped at the time limit, 4 some face could not be meshed (IMeshData_Failure),
// 5 the kernel crashed (set by PF_MeshCopy before the call; the inner function never returns).
static const int PF_MESH_OK = 0;
static const int PF_MESH_REFUSED = 1;
static const int PF_MESH_THREW = 2;
static const int PF_MESH_TIME_LIMIT = 3;
static const int PF_MESH_FACE_FAILED = 4;
static const int PF_MESH_CRASHED = 5;

class _PF_MeshDeadline : public Message_ProgressIndicator {
public:
  explicit _PF_MeshDeadline(Standard_Real seconds)
      : deadline_(std::chrono::steady_clock::now() +
                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(seconds))) {}
  DEFINE_STANDARD_RTTI_INLINE(_PF_MeshDeadline, Message_ProgressIndicator)

protected:
  // Called from the mesher's threads: reads the clock and a constant only.
  virtual Standard_Boolean UserBreak() override {
    return std::chrono::steady_clock::now() >= deadline_;
  }
  virtual void Show(const Message_ProgressScope &, const Standard_Boolean) override {}

private:
  const std::chrono::steady_clock::time_point deadline_;
};

static std::unique_ptr<TopoDS_Shape> _pf_mesh_copy_inner(
    const TopoDS_Shape &shape, Standard_Real linear, Standard_Real angle_radians,
    Standard_Real time_limit_s, int &status) {
  try {
    // BRepMesh_IncrementalMesh's own floors (it throws below them) and finite numbers: a NaN
    // deviation passes OCCT's checks and can hang it (OCCT #1598). An angle past half a turn
    // means nothing; 10 days caps the time limit so the clock arithmetic cannot overflow.
    if (shape.IsNull() || !std::isfinite(linear) || !std::isfinite(angle_radians) ||
        !std::isfinite(time_limit_s) || linear < Precision::Confusion() ||
        angle_radians < Precision::Angular() || angle_radians > PF_TWO_PI / 2.0 ||
        time_limit_s <= 0.0 || time_limit_s > 864000.0) {
      status = PF_MESH_REFUSED;
      return nullptr;
    }
    BRepBuilderAPI_Copy copier(shape, Standard_True, Standard_False);
    if (!copier.IsDone() || copier.Shape().IsNull()) {
      status = PF_MESH_THREW;
      return nullptr;
    }
    TopoDS_Shape copy = copier.Shape();
    IMeshTools_Parameters parameters;
    parameters.Deflection = linear;
    parameters.DeflectionInterior = linear;
    parameters.Angle = angle_radians;
    parameters.AngleInterior = angle_radians;
    parameters.Relative = Standard_False;
    parameters.InParallel = Standard_True;
    Handle(_PF_MeshDeadline) deadline = new _PF_MeshDeadline(time_limit_s);
    BRepMesh_IncrementalMesh mesher(copy, parameters, deadline->Start());
    const Standard_Integer flags = mesher.GetStatusFlags();
    if ((flags & IMeshData_UserBreak) != 0) {
      status = PF_MESH_TIME_LIMIT;
      return nullptr;
    }
    if (!mesher.IsDone()) {
      status = PF_MESH_THREW;
      return nullptr;
    }
    if ((flags & IMeshData_Failure) != 0) {
      status = PF_MESH_FACE_FAILED;
      return nullptr;
    }
    std::unique_ptr<TopoDS_Shape> result = _pf_shape_with_faces(copy);
    status = result ? PF_MESH_OK : PF_MESH_THREW;
    return result;
  } catch (...) {
    status = PF_MESH_THREW;
    return nullptr;
  }
}

// A STEP import's read, check and repair (ParaForge queue row SI). `status` says why a result is
// null: 0 done, 1 input refused, 2 OCCT threw or built nothing, 3 the kernel crashed (set by the
// PF_ function before the call; the inner function never returns), 4 PF_ReadStep only: the file
// could not be read as STEP.
static const int PF_IMPORT_OK = 0;
static const int PF_IMPORT_REFUSED = 1;
static const int PF_IMPORT_THREW = 2;
static const int PF_IMPORT_CRASHED = 3;
static const int PF_IMPORT_UNREADABLE = 4;

// Shape::read_step: STEPControl_Reader::ReadFile, TransferRoots, OneShape, with OCCT's own
// settings (CSF_STEPDefaults unset: one ShapeFix_Shape pass inside the transfer, no sewing). A
// file with nothing to transfer gives a null or empty shape: returned as it is, for the caller to
// count.
static std::unique_ptr<TopoDS_Shape> _pf_read_step_inner(rust::Str path, int &status) {
  try {
    const std::string file(path.data(), path.size());
    if (file.empty()) {
      status = PF_IMPORT_REFUSED;
      return nullptr;
    }
    STEPControl_Reader reader;
    if (reader.ReadFile(file.c_str()) != IFSelect_RetDone) {
      status = PF_IMPORT_UNREADABLE;
      return nullptr;
    }
    reader.TransferRoots(Message_ProgressRange());
    const TopoDS_Shape shape = reader.OneShape();
    status = PF_IMPORT_OK;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(shape));
  } catch (...) {
    status = PF_IMPORT_THREW;
    return nullptr;
  }
}

// BRepBuilderAPI_Sewing(tolerance) with its default options (sewing, analysis, cutting; no
// non-manifold), as ParaForge-Geometry's prototype measured it (spike/importheal.py). The sewn
// shape is a shell, or a compound of shells and faces.
static std::unique_ptr<TopoDS_Shape> _pf_sew_inner(
    const TopoDS_Shape &shape, Standard_Real tolerance, int &status) {
  try {
    if (shape.IsNull() || !std::isfinite(tolerance) || !(tolerance > 0.0) || tolerance > 10.0) {
      status = PF_IMPORT_REFUSED;
      return nullptr;
    }
    BRepBuilderAPI_Sewing sewing(tolerance);
    sewing.Add(shape);
    sewing.Perform();
    std::unique_ptr<TopoDS_Shape> result = _pf_shape_with_faces(sewing.SewedShape());
    status = result ? PF_IMPORT_OK : PF_IMPORT_THREW;
    return result;
  } catch (...) {
    status = PF_IMPORT_THREW;
    return nullptr;
  }
}

// Every closed shell that is not already inside a solid becomes a solid
// (ShapeFix_Solid::SolidFromShell, which also turns it outward); every solid, open shell, face
// outside a shell and edge outside a face is kept as it is. One solid and nothing else comes back
// as that solid, not a compound.
static std::unique_ptr<TopoDS_Shape> _pf_solids_from_shells_inner(const TopoDS_Shape &shape, int &status) {
  try {
    if (shape.IsNull()) {
      status = PF_IMPORT_REFUSED;
      return nullptr;
    }
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    TopTools_MapOfShape seen;
    Standard_Integer pieces = 0;
    TopoDS_Shape last;
    for (TopExp_Explorer it(shape, TopAbs_SOLID); it.More(); it.Next()) {
      if (seen.Add(it.Current())) {
        builder.Add(compound, it.Current());
        last = it.Current();
        pieces++;
      }
    }
    for (TopExp_Explorer it(shape, TopAbs_SHELL, TopAbs_SOLID); it.More(); it.Next()) {
      if (!seen.Add(it.Current())) {
        continue;
      }
      const TopoDS_Shell &shell = TopoDS::Shell(it.Current());
      if (BRep_Tool::IsClosed(shell)) {
        ShapeFix_Solid fix;
        const TopoDS_Solid solid = fix.SolidFromShell(shell);
        if (solid.IsNull()) {
          status = PF_IMPORT_THREW;
          return nullptr;
        }
        builder.Add(compound, solid);
        last = solid;
      } else {
        builder.Add(compound, shell);
        last = shell;
      }
      pieces++;
    }
    for (TopExp_Explorer it(shape, TopAbs_FACE, TopAbs_SHELL); it.More(); it.Next()) {
      if (seen.Add(it.Current())) {
        builder.Add(compound, it.Current());
        last = it.Current();
        pieces++;
      }
    }
    for (TopExp_Explorer it(shape, TopAbs_EDGE, TopAbs_FACE); it.More(); it.Next()) {
      if (seen.Add(it.Current())) {
        builder.Add(compound, it.Current());
        last = it.Current();
        pieces++;
      }
    }
    if (pieces == 1 && last.ShapeType() == TopAbs_SOLID) {
      status = PF_IMPORT_OK;
      return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(last));
    }
    std::unique_ptr<TopoDS_Shape> result = _pf_shape_with_faces(compound);
    status = result ? PF_IMPORT_OK : PF_IMPORT_THREW;
    return result;
  } catch (...) {
    status = PF_IMPORT_THREW;
    return nullptr;
  }
}

// What a shape holds, for an import's report. Each sub-shape is counted once: faces; solids;
// shells outside any solid, closed and open; faces outside any shell; faces with no surface (a
// mesh-only STEP: triangles and nothing else); open edges (on exactly one face, not a seam, not
// degenerated); the largest edge tolerance and the smallest vertex tolerance (0 when there are
// none).
static int _pf_shape_counts_inner(
    const TopoDS_Shape &shape, int &faces, int &solids, int &closed_shells, int &open_shells,
    int &loose_faces, int &faces_without_surface, int &open_edges,
    Standard_Real &max_edge_tolerance, Standard_Real &min_vertex_tolerance) {
  try {
    faces = solids = closed_shells = open_shells = loose_faces = faces_without_surface = open_edges = 0;
    max_edge_tolerance = min_vertex_tolerance = 0.0;
    if (shape.IsNull()) {
      return PF_IMPORT_OK;
    }
    TopTools_IndexedMapOfShape all_faces;
    TopExp::MapShapes(shape, TopAbs_FACE, all_faces);
    faces = all_faces.Extent();
    for (Standard_Integer i = 1; i <= all_faces.Extent(); i++) {
      TopLoc_Location location;
      if (BRep_Tool::Surface(TopoDS::Face(all_faces(i)), location).IsNull()) {
        faces_without_surface++;
      }
    }
    TopTools_IndexedMapOfShape all_solids;
    TopExp::MapShapes(shape, TopAbs_SOLID, all_solids);
    solids = all_solids.Extent();
    TopTools_MapOfShape seen;
    for (TopExp_Explorer it(shape, TopAbs_SHELL, TopAbs_SOLID); it.More(); it.Next()) {
      if (seen.Add(it.Current())) {
        if (BRep_Tool::IsClosed(it.Current())) {
          closed_shells++;
        } else {
          open_shells++;
        }
      }
    }
    for (TopExp_Explorer it(shape, TopAbs_FACE, TopAbs_SHELL); it.More(); it.Next()) {
      if (seen.Add(it.Current())) {
        loose_faces++;
      }
    }
    TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
    for (Standard_Integer i = 1; i <= edge_faces.Extent(); i++) {
      const TopoDS_Edge &edge = TopoDS::Edge(edge_faces.FindKey(i));
      if (BRep_Tool::Degenerated(edge)) {
        continue;
      }
      TopTools_MapOfShape on;
      for (TopTools_ListIteratorOfListOfShape f(edge_faces.FindFromIndex(i)); f.More(); f.Next()) {
        on.Add(f.Value());
      }
      if (on.Extent() == 1) {
        TopTools_MapIteratorOfMapOfShape only(on);
        if (!BRep_Tool::IsClosed(edge, TopoDS::Face(only.Key()))) {
          open_edges++;
        }
      }
    }
    ShapeAnalysis_ShapeTolerance tolerance;
    if (TopExp_Explorer(shape, TopAbs_EDGE).More()) {
      max_edge_tolerance = tolerance.Tolerance(shape, 1, TopAbs_EDGE);
    }
    if (TopExp_Explorer(shape, TopAbs_VERTEX).More()) {
      min_vertex_tolerance = tolerance.Tolerance(shape, -1, TopAbs_VERTEX);
    }
    return PF_IMPORT_OK;
  } catch (...) {
    return PF_IMPORT_THREW;
  }
}

// With MSVC an access violation inside the kernel is a structured exception, which only
// __except catches; elsewhere (macOS) it cannot be caught at all, and the input checks
// above are the only protection. MSVC refuses __try and C++ try in one function, so each
// PF_ function wraps its _inner one. After a stack overflow the stack's guard page is
// gone until _resetstkoflw() puts it back.
#ifdef _MSC_VER
#define PF_GUARDED(call)                                \
  __try {                                               \
    return call;                                        \
  } __except (EXCEPTION_EXECUTE_HANDLER) {              \
    if (GetExceptionCode() == EXCEPTION_STACK_OVERFLOW) { \
      _resetstkoflw();                                  \
    }                                                   \
    return nullptr;                                     \
  }
#else
#define PF_GUARDED(call) return call;
#endif

inline std::unique_ptr<TopoDS_Wire> PF_MakeWire(
    const Handle(TopTools_HSequenceOfShape) &edges, Standard_Real tolerance) {
  PF_GUARDED(_pf_make_wire_inner(edges, tolerance))
}

inline std::unique_ptr<TopoDS_Face> PF_MakeFace(const TopoDS_Wire &wire) {
  PF_GUARDED(_pf_make_face_inner(wire))
}

inline std::unique_ptr<TopoDS_Shape> PF_MakePrism(
    const TopoDS_Shape &shape, Standard_Real dx, Standard_Real dy, Standard_Real dz) {
  PF_GUARDED(_pf_make_prism_inner(shape, dx, dy, dz))
}

inline std::unique_ptr<TopoDS_Shape> PF_MakeRevol(
    const TopoDS_Shape &shape,
    Standard_Real ox, Standard_Real oy, Standard_Real oz,
    Standard_Real dx, Standard_Real dy, Standard_Real dz,
    Standard_Real angle_radians) {
  PF_GUARDED(_pf_make_revol_inner(shape, ox, oy, oz, dx, dy, dz, angle_radians))
}

inline std::unique_ptr<TopoDS_Shape> PF_MakePipe(
    const TopoDS_Wire &spine, const TopoDS_Shape &profile) {
  PF_GUARDED(_pf_make_pipe_inner(spine, profile))
}

// A copy of `shape` meshed at `linear` (mm, absolute) and `angle_radians`, or null with `status`
// saying why (see _pf_mesh_copy_inner). The shape itself and its own triangulation are untouched.
inline std::unique_ptr<TopoDS_Shape> PF_MeshCopy(
    const TopoDS_Shape &shape, Standard_Real linear, Standard_Real angle_radians,
    Standard_Real time_limit_s, int &status) {
  status = PF_MESH_CRASHED;
  PF_GUARDED(_pf_mesh_copy_inner(shape, linear, angle_radians, time_limit_s, status))
}

// The shape a STEP file holds (null or empty when it holds nothing), or null with `status` saying
// why (see PF_IMPORT_OK and the rest above).
inline std::unique_ptr<TopoDS_Shape> PF_ReadStep(rust::Str path, int &status) {
  status = PF_IMPORT_CRASHED;
  PF_GUARDED(_pf_read_step_inner(path, status))
}

// The shape's faces sewn at `tolerance` (mm), or null with `status`.
inline std::unique_ptr<TopoDS_Shape> PF_Sew(const TopoDS_Shape &shape, Standard_Real tolerance, int &status) {
  status = PF_IMPORT_CRASHED;
  PF_GUARDED(_pf_sew_inner(shape, tolerance, status))
}

// The shape with each closed shell made a solid, or null with `status`.
inline std::unique_ptr<TopoDS_Shape> PF_SolidsFromShells(const TopoDS_Shape &shape, int &status) {
  status = PF_IMPORT_CRASHED;
  PF_GUARDED(_pf_solids_from_shells_inner(shape, status))
}

// Fills the counts (see _pf_shape_counts_inner) and returns PF_IMPORT_OK, or another status with the
// counts left at 0.
inline int PF_ShapeCounts(
    const TopoDS_Shape &shape, int &faces, int &solids, int &closed_shells, int &open_shells,
    int &loose_faces, int &faces_without_surface, int &open_edges,
    Standard_Real &max_edge_tolerance, Standard_Real &min_vertex_tolerance) {
#ifdef _MSC_VER
  __try {
    return _pf_shape_counts_inner(shape, faces, solids, closed_shells, open_shells, loose_faces,
                                  faces_without_surface, open_edges, max_edge_tolerance,
                                  min_vertex_tolerance);
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    if (GetExceptionCode() == EXCEPTION_STACK_OVERFLOW) {
      _resetstkoflw();
    }
    faces = solids = closed_shells = open_shells = loose_faces = faces_without_surface = open_edges = 0;
    max_edge_tolerance = min_vertex_tolerance = 0.0;
    return PF_IMPORT_CRASHED;
  }
#else
  return _pf_shape_counts_inner(shape, faces, solids, closed_shells, open_shells, loose_faces,
                                faces_without_surface, open_edges, max_edge_tolerance,
                                min_vertex_tolerance);
#endif
}

#undef PF_GUARDED

#endif // PARAFORGE_KERNEL_HXX
