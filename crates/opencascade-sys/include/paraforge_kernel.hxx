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

#include <cmath>
#include <memory>

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Tool.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
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

#undef PF_GUARDED

#endif // PARAFORGE_KERNEL_HXX
