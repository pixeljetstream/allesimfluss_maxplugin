/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_STRIP__H
#define __SKETCHMODELER_BRUSH_STRIP__H

#include "brush.h"
#include "strokeprocessor.h"

#ifdef SM_SUPPORT_STRIP

//////////////////////////////////////////////////////////////////////////
// SMBrushStrip



enum {
  SM_STRIP_DEFAULT,
  SM_STRIP_REPLAY,
  SM_STRIPS,
};

class SMBrushStrip : public SMBrush
{
private:
  enum Mode{
    TYPE_FREE,
    TYPE_EXTEND,
    TYPE_EXTRUDE,
    TYPE_INSET,
    TYPE_CLONE,
    TYPE_GRABPATH,
    TYPE_REPLAYFREE,
    TYPE_REPLAYEXTEND,
    TYPE_REPLAYEXTRUDE,
    TYPE_IGNORE,
  };

public:
  SMBrushStrip() 
    : m_type(TYPE_FREE)
    , m_stroking(FALSE)
    , m_replayFlip(FALSE)
    , m_replayScale(FALSE)
    , m_tchange(NULL)
    , m_drawhighlights(FALSE)
    , m_delayconstr(FALSE)
    , m_modeangle(45.0f)
    , m_drawpoints(FALSE)
    , m_drawpointsforce(FALSE)
    , m_vertrestore(NULL)
    , m_drawconnected(FALSE)
    , m_refine(FALSE)
    , m_selection(StrokeProcessor::SEL_ADD)
    , m_allowConnection(TRUE)
    , SMBrush(SM_BRUSH_STRIP,SM_REQUIRE_SOBASETYPES | SM_REQUIRE_MODIFYCOMMAND | SM_REQUIRE_DRAWPLANE | SM_REQUIRE_DRAWPLANEALT, COORDS_LOCAL) {}
  ~SMBrushStrip(){}

  // inherited
  BOOL DrawCircleBrush(BOOL meshit) {
    return !m_stroking;
  }
  void Init(SketchModeler* oSM);
  void SetConstrained(BOOL state);
  void SetAlternative(BOOL state);

  void Display(TimeValue t, ViewExp *vpt, int flags);
  void GetViewportRect( TimeValue t, ViewExp *vpt, Rect *rect );

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit);

  void EndStroke(const SMHit &hit);
  void AbortStroke();

  BOOL BeginRefine();
  void Refine();

  void  SetRemoveThreshold(float state) { m_strokeproc.SetRemoveThreshold(state);}
  float GetRemoveThreshold() const { return m_strokeproc.GetRemoveThreshold();}
  void  SetRailMode(int state) { m_strokeproc.SetRailMode((StrokeProcessor::RailMode)state);}
  int   GetRailMode() const { return m_strokeproc.GetRailMode();}
  void  SetWeldFactor(float state) { m_strokeproc.SetWeldFactor(state);}
  float GetWeldFactor() const { return m_strokeproc.GetWeldFactor();}
  void  SetWeldMode(int state) { m_strokeproc.SetWeldMode((StrokeProcessor::WeldMode)state);}
  int   GetWeldMode() const { return m_strokeproc.GetWeldMode();}
  void  SetStrideMulti(float mul) { m_strokeproc.SetStrideMulti(mul);}
  float GetStrideMulti() const {return m_strokeproc.GetStrideMulti();}
  void  SetStrideMode(int  mul) { m_strokeproc.SetStrideMode((StrokeProcessor::DeltaStrideMode)mul);}
  int   GetStrideMode() const {return m_strokeproc.GetStrideMode();}
  void  SetRelaxFactor(float mul) {m_strokeproc.SetRelaxFactor(mul);}
  float GetRelaxFactor() const {return m_strokeproc.GetRelaxFactor();}
  void  SetRelaxRuns(int mul) {m_strokeproc.SetRelaxRuns(mul);}
  int   GetRelaxRuns() const {return m_strokeproc.GetRelaxRuns();}
  void  SetRotateFactor(float mul) {m_strokeproc.SetRotateFactor(mul);}
  float GetRotateFactor() const { return m_strokeproc.GetRotateFactor();}
  void  SetRotateFree(float mul) {m_strokeproc.SetRotateFree(mul);}
  float GetRotateFree() const { return m_strokeproc.GetRotateFree();}
  void  SetDeltaInplane(BOOL state) { m_strokeproc.SetEdgeCorrectInplane(state);}
  BOOL  GetDeltaInplane() const { return m_strokeproc.GetEdgeCorrectInplane(); }
  void  SetDeltaThreshold(float state) { m_strokeproc.SetEdgeCorrectThreshold(state);}
  float GetDeltaThreshold() const { return m_strokeproc.GetEdgeCorrectThreshold();}
  void  SetSmoothing(BOOL state) { m_strokeproc.SetSmoothing(state);}
  BOOL  GetSmoothing() const { return m_strokeproc.GetSmoothing(); }
  void  SetSmoothThreshold(float state) { m_strokeproc.SetSmoothThreshold(state);}
  float GetSmoothThreshold() const { return m_strokeproc.GetSmoothThreshold();}
  void  SetFreeDeltas(const Tab<Point3> &deltas){ m_strokeproc.SetFreeDeltas(deltas);}
  void  SetExplicitStride(BOOL state) { m_strokeproc.SetExplicitStride(state);}
  BOOL  GetExplicitStride() const { return m_strokeproc.GetExplicitStride();}
  void  SetSelectionMode(int mode) { m_selection = (StrokeProcessor::SelectionMode) mode;}
  int   GetSelectionMode() const { return m_selection; }
  void  SetConnectInterpolation(const SMInterpolation& interpolate) { m_strokeproc.SetConnectInterpolation(interpolate);}
  void  GetConnectInterpolation(SMInterpolation& interpolate) const { m_strokeproc.GetConnectInterpolation(interpolate);}
  void  SetOpenInterpolation(BOOL state) {m_strokeproc.SetOpenInterpolation( state ); }
  BOOL  GetOpenInterpolation() {return m_strokeproc.GetOpenInterpolation(); }
  void  SetOpenOffset(float state) {m_strokeproc.SetOpenOffset( state ); }
  float  GetOpenOffset() {return m_strokeproc.GetOpenOffset(); }
  

  BOOL SaveReplay( Tab<float>&replay ) const {
    return m_strokeproc.SaveReplay(replay);
  }
  BOOL LoadReplay( const Tab<float>&replay ) {
    return m_strokeproc.LoadReplay(replay);
  }

  float m_modeangle;
  BOOL  m_drawpoints;
  BOOL  m_allowConnection;
  BOOL  m_replayFlip;
  BOOL  m_replayScale;

private:
  int     GetEndEdge(const SMHit &hit, bool discardfirst=true);
  Mode    GetFaceType(MNMesh* mesh, int face, const Matrix3 &objmatinv, Matrix3 &facematrix);

  void    ProcInset(const SMHit &hit, Point3 curpos, MNMesh * mesh );
  void    ProcClone( const SMHit &hit, Point3 curposworld, float constrscale, ViewExp * vpt );
  void    StartClone( const SMHit &hit, MNMesh * mesh );
  BOOL    StartInset(const SMHit &hit, MNMesh * mesh );

  BOOL            m_stroking;
  BOOL            m_delaystart; // for EXTEND & FREEE
  BOOL            m_delayconstr;
  BOOL            m_drawconnected;
  BOOL            m_replayed;
  BOOL            m_refine;
  BOOL            m_lastReplayFlip;
  BOOL            m_lastReplayScale;
  SMHit           m_refinehit;
  int             m_subobj;

  BOOL            m_drawpointsforce;
  Mode            m_type;
  StrokeProcessor m_strokeproc;
  StrokeProcessor m_strokeprocTemp;
  StrokeProcessor::SelectionMode   m_selection;

  int             m_startface;
  int             m_startedge;

  int             m_lastendedge;
  int             m_lastendpoint;
  SMInterpolation m_lastinterpolate;

  Point2          m_startmouse;
  float           m_startdist;
  float           m_startsize;
  Point3          m_startpos;
  Point3          m_startnormal;
  Matrix3         m_startfacematrix;

  BOOL            m_drawhighlights;
  Tab<int>        m_highlightedges;
  Point3          m_highlightanchor;

  MNMesh          m_pmesh;    // preview/change mesh
  MNMesh          m_pmeshTemp;

  Point3              m_center;   // for cloning
  MeshVertRestore     *m_vertrestore;
  TopoChangeRestore   *m_tchange;
  MNTempData          m_meshtemp;

  BOOL              m_isflipped;
  BOOL              m_canflip;
};

#endif

#endif

