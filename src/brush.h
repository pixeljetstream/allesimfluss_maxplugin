/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/


#ifndef __SKETCHMODELER_BRUSH__H
#define __SKETCHMODELER_BRUSH__H

#include "maxincludes.h"
#include "helpers.h"

typedef enum{
  SM_BRUSH_DUMMY,

  SM_BRUSH_SELECT,
  SM_BRUSH_STRIP,
  SM_BRUSH_TWEAK,             // subobj level transform

  SM_BRUSH_FILL,
  SM_BRUSH_RELAX,

  SM_BRUSH_SMOOTH,
  SM_BRUSH_SCULPT,
  SM_BRUSH_FLATTEN,

  SM_BRUSH_NODESELECT,
  SM_BRUSH_NODETRANSFORM,     // obj level transform
  SM_BRUSHES,
}SMBrushType;

class SMBrushSelect;
class SMBrushTweak;
class SMBrushSmooth;
class SMBrushSculpt;
class SMBrushFlatten;
class SMBrushStrip;
class SMBrushNodeSelect;
class SMBrushNodeTransform;
class SMBrushReflow;
class SMBrushFill;
class SMBrushRelax;

enum SMBrushRequires{
  SM_REQUIRE_NOTHING    = 0,
  SM_REQUIRE_MESHCONVERT  = 1<<0,
  SM_REQUIRE_SOBASETYPES  = 1<<1,
  SM_REQUIRE_OBJLEVEL   = 1<<2, // otherwise need solevel
  SM_REQUIRE_TRANSFORM  = 1<<3, // "startmode" will be overridden to transformtype
  SM_REQUIRE_DRAWHITS   = 1<<4,
  SM_REQUIRE_MODIFYCOMMAND = 1<<5,
  SM_REQUIRE_NOCUSTOMRCS  = 1<<6,
  SM_REQUIRE_HITUPDWHENPUSHED = 1<<7,
  SM_REQUIRE_DRAWPLANE  = 1<<8,
  SM_REQUIRE_DRAWPLANEALT = 1<<9,   // allow alternative drawplane
  SM_REQUIRE_DRAWPLANEKEEP = 1<<10, // dont change the plane draw state
  SM_REQUIRE_DRAWHITSALT = 1<<11,
};


class SketchModeler;

//////////////////////////////////////////////////////////////////////////
// SMBrush
class SMBrush
{
private:
  DWORD     m_require;
  int       m_refcoordsys;
  SMBrushType   m_smbrushtype;

public:
  SketchModeler* m_SM;



  SMBrush(SMBrushType type) {m_require = SM_REQUIRE_NOCUSTOMRCS; m_refcoordsys = -1; m_smbrushtype = type;}
  SMBrush(SMBrushType type,DWORD require) {m_require = require; m_refcoordsys = -1; m_smbrushtype = type;}
  SMBrush(SMBrushType type,DWORD require, int refcoord) {m_require = require; m_refcoordsys = refcoord; m_smbrushtype = type;}

  virtual ~SMBrush() {}

  BOOL  Requires(UINT flag) {return (m_require & flag)!=0;}
  DWORD Requires() { return m_require;}

  int   GetRefCoordSys() { 
    return m_refcoordsys >= 0 ? m_refcoordsys : GetCOREInterface()->GetRefCoordSys(); 
  }
  void  StoreRefCoordSys() {
    m_refcoordsys = GetCOREInterface()->GetRefCoordSys();
  }

  SMBrushType GetType() {return m_smbrushtype;}

  // overloaded
  virtual void Init(SketchModeler* oSM) { m_SM = oSM;}

  virtual BOOL DrawCircleBrush(BOOL meshit) {return FALSE;}
  virtual void SetConstrained(BOOL state) {}
  virtual void SetAlternative(BOOL state) {}

  virtual void Display(TimeValue t, ViewExp *vpt, int flags) {}
  virtual void GetViewportRect( TimeValue t, ViewExp *vpt, Rect *rect ) {}

  virtual void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm) {}

  virtual void StartStroke(int mode, BOOL continued, int subobj, const SMHit &hit) {}
  virtual void EndStroke(const SMHit &hit) {}
  virtual void AbortStroke() {}
};

//////////////////////////////////////////////////////////////////////////
// SMStroke
// some brushes might want to have multiple stroke types

class SMStroke
{
public:
  SketchModeler* m_SM;

  virtual void Init(SketchModeler* SM, SMBrush *brush) {}

  virtual void Display(TimeValue t, ViewExp *vpt, int flags) {}
  virtual void GetViewportRect( TimeValue t, ViewExp *vpt, Rect *rect ) {}

  virtual void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm) {}

  virtual void Start(BOOL cont, int subobj, const SMHit &hit) {}
  virtual void End(const SMHit &hit) {}
  virtual void Abort() {}
  virtual MCHAR* GetUndoMessage() {return _M("AIF Stroke");}
};

#endif
