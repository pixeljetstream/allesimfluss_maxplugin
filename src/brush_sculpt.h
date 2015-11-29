/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_SCULPT__H
#define __SKETCHMODELER_BRUSH_SCULPT__H

#include "brush.h"

#ifdef SM_SUPPORT_SCULPT

//////////////////////////////////////////////////////////////////////////
// SMBrushSculpt

class SMBrushSculpt : public SMBrush
{

public:
  SMBrushSculpt() : m_stroking(FALSE),m_speed(0.2f),m_actrestore(NULL),
    SMBrush(SM_BRUSH_SCULPT,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES | SM_REQUIRE_DRAWPLANE, COORDS_SCREEN) {}
  ~SMBrushSculpt(){}

  // overloaded
  BOOL DrawCircleBrush(BOOL meshit) { return meshit; }

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit);
  void EndStroke(const SMHit &hit);
  void AbortStroke();

  float       m_dirsign;
  float       m_speed;

private:
  BOOL              m_stroking;
  MeshVertRestore*  m_actrestore;
  int               m_subobj;
};

#else

class SMBrushSculpt : public SMBrush
{

public:
  SMBrushSculpt() :
    SMBrush(SM_BRUSH_SCULPT,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES | SM_REQUIRE_DRAWPLANE, COORDS_SCREEN) {}
  ~SMBrushSculpt(){}

};

#endif

#endif

