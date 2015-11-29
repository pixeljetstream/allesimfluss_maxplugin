/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_SELECT__H
#define __SKETCHMODELER_BRUSH_SELECT__H

#include "brush.h"

#ifdef SM_SUPPORT_SELECT

//////////////////////////////////////////////////////////////////////////
// SMBrushSelect

enum{
  SM_SELECT_PAINT,
  SM_SELECT_LOOPPART,
  SM_SELECT_LOOPFULL,
  SM_SELECT_SHAPE,
  SM_SELECT_PAINTWIDE,
  SM_SELECTS,
};

class SMStrokeSelectPacket;

class SMBrushSelect : public SMBrush
{
public:
  SMBrushSelect() : m_additive(true),m_stroke(NULL),m_strokepacket(NULL),
    SMBrush(SM_BRUSH_SELECT,SM_REQUIRE_DRAWHITS | SM_REQUIRE_NOCUSTOMRCS | SM_REQUIRE_HITUPDWHENPUSHED | SM_REQUIRE_DRAWPLANEKEEP) {}
  ~SMBrushSelect();

  // overloaded
  void Init(SketchModeler* oSM);

  void Display(TimeValue t, ViewExp *vpt, int flags);
  void GetViewportRect( TimeValue t, ViewExp *vpt, Rect *rect );

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit);
  void EndStroke(const SMHit &hit);
  void AbortStroke();

  BOOL    m_additive;

private:
  SMStroke  *m_stroke;
  SMStroke  *m_strokes[SM_SELECTS];
  SMStrokeSelectPacket  *m_strokepacket;

};

#else

class SMBrushSelect : public SMBrush
{
public:
  SMBrushSelect() :
    SMBrush(SM_BRUSH_SELECT,SM_REQUIRE_DRAWHITS | SM_REQUIRE_NOCUSTOMRCS | SM_REQUIRE_HITUPDWHENPUSHED | SM_REQUIRE_DRAWPLANEKEEP) {}
  ~SMBrushSelect() {}

};


#endif

#endif

