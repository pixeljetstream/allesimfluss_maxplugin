/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_SMOOTH__H
#define __SKETCHMODELER_BRUSH_SMOOTH__H

#include "brush.h"

#ifdef SM_SUPPORT_SMOOTH

//////////////////////////////////////////////////////////////////////////
// SMBrushSmooth

class SMBrushSmooth : public SMBrush
{

public:
  SMBrushSmooth() : m_pinch(FALSE),m_stroking(FALSE),m_speed(0.5f),m_startselected(FALSE),m_actrestore(NULL),
    SMBrush(SM_BRUSH_SMOOTH,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES, COORDS_SCREEN) {}
  ~SMBrushSmooth(){}

  // inherited
  BOOL DrawCircleBrush(BOOL meshit) { return meshit; }

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit);
  void EndStroke(const SMHit &hit);
  void AbortStroke();

  float       m_speed;
  BOOL        m_pinch;

private:
  BOOL              m_stroking;
  MeshVertRestore*  m_actrestore;
  int               m_subobj;
  BOOL              m_startselected;
};

#else

class SMBrushSmooth : public SMBrush
{

public:
  SMBrushSmooth() :
    SMBrush(SM_BRUSH_SMOOTH,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES, COORDS_SCREEN) {}
  ~SMBrushSmooth(){}

};

#endif

#endif

