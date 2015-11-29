/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_TWEAK__H
#define __SKETCHMODELER_BRUSH_TWEAK__H

#include "brush.h"

#ifdef SM_SUPPORT_TWEAK

//////////////////////////////////////////////////////////////////////////
// SMBrushTweak

class SMStrokeTweakMove;
class SMStrokeTweakEPolyTransform;

class SMBrushTweak : public SMBrush
{
friend class SMStrokeTweakMove;
friend class SMStrokeTweakEPolyTransform;

public:

  SMBrushTweak() : m_startid(-1),m_actrestore(NULL),m_soft(FALSE),m_stroke(NULL),m_topochange(NULL),m_masked(FALSE),
    SMBrush(SM_BRUSH_TWEAK,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES | SM_REQUIRE_TRANSFORM | SM_REQUIRE_DRAWPLANE, COORDS_LOCAL) {}
  ~SMBrushTweak();

  // inherited
  BOOL DrawCircleBrush(BOOL meshit);
  void Init(SketchModeler* oSM);
  void SetConstrained(BOOL state);
  void SetAlternative(BOOL state);

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit);
  void EndStroke(const SMHit &hit);
  void AbortStroke();

  BOOL        m_wasactive;
  BOOL        m_soft;
  BOOL        m_masked;


private:
  void RevertInterimChanges();

  SMStroke*     m_stroke;
  int           m_subobj;
  float         m_strokesize;

  BitArray      m_oldsel;
  int           m_oldlocksoft;
  int           m_oldusesel;


  BOOL              m_alternative;
  SMTransformType   m_type;

  int           m_startid;
  Point3        m_startpos;
  Point3        m_startwpos;
  Point2        m_startmouse;
  Point3        m_center;

  MeshVertRestore   *m_actrestore;
  TopoChangeRestore *m_topochange;

  SMStrokeTweakMove*            m_strokemove;
  SMStrokeTweakEPolyTransform*  m_strokepoly;
};

#else

class SMBrushTweak : public SMBrush
{
public:

  SMBrushTweak() :
    SMBrush(SM_BRUSH_TWEAK,SM_REQUIRE_MESHCONVERT | SM_REQUIRE_SOBASETYPES | SM_REQUIRE_TRANSFORM, COORDS_SCREEN | SM_REQUIRE_DRAWPLANE | SM_REQUIRE_DRAWPLANEALT ) {}
    ~SMBrushTweak() {}
};

#endif

#endif

