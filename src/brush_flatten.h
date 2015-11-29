/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_FLATTEN__H
#define __SKETCHMODELER_BRUSH_FLATTEN__H

#include "brush.h"

#ifdef SM_SUPPORT_FLATTEN

//////////////////////////////////////////////////////////////////////////
// SMBrushFlatten

class SMBrushFlatten : public SMBrush
{

public:
  SMBrushFlatten() : m_stroking(FALSE),m_speed(0.2f),m_actrestore(NULL),
    SMBrush(SM_BRUSH_FLATTEN,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES | SM_REQUIRE_DRAWPLANE) {}
  ~SMBrushFlatten(){}

  // overloaded
  BOOL DrawCircleBrush(BOOL meshit) { return meshit; }

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit);
  void EndStroke(const SMHit &hit);
  void AbortStroke();

  BOOL        m_fixdist;
  float       m_speed;

private:
  BOOL              m_stroking;
  MeshVertRestore*  m_actrestore;
  int               m_subobj;
  BOOL              m_first;

  Point3            m_planenormal;
  float             m_planedot;
};

#else

class SMBrushFlatten : public SMBrush
{

public:
  SMBrushFlatten() : 
    SMBrush(SM_BRUSH_FLATTEN,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES | SM_REQUIRE_DRAWPLANE) {}
  ~SMBrushFlatten(){}

};

#endif

#endif

