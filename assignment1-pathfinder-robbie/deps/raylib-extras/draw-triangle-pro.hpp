#ifndef _DRAW_TRIANGLE_PRO_
#define _DRAW_TRIANGLE_PRO_

#include "raylib.h" // Vector2, Color
#include "rlgl.h"   // rlCheckRenderBatchLimit, rlPushMatrix

namespace ai {

// Based on DrawTriangle in ./src/rshapes.c
// with translation/rotation a la. DrawPoly
// Vector2 is a "C" struct, in the global scope. raylib-cpp.hpp also makes
// it available in the raylib namespace, but that's not required here.

// NOTE: Vertex must be provided in counter-clockwise order
inline void DrawTrianglePro(Vector2 center, Vector2 v1, Vector2 v2, Vector2 v3,
                            float rotation, Color color)
{
  rlCheckRenderBatchLimit(4);

  rlPushMatrix();
    rlTranslatef(center.x, center.y, 0.0f);
    rlRotatef(rotation, 0.0f, 0.0f, 1.0f);



#if defined(SUPPORT_QUADS_DRAW_MODE)
    rlSetTexture(texShapes.id);

    rlBegin(RL_QUADS);
        rlColor4ub(color.r, color.g, color.b, color.a);

        rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(v1.x, v1.y);

        rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(v2.x, v2.y);

        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(v2.x, v2.y);

        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(v3.x, v3.y);
    rlEnd();

    rlSetTexture(0);
#else
    rlBegin(RL_TRIANGLES);
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlVertex2f(v1.x, v1.y);
        rlVertex2f(v2.x, v2.y);
        rlVertex2f(v3.x, v3.y);
    rlEnd();
#endif



  rlPopMatrix();
}

// Draw circle outline (Vector version)
inline void DrawCircleLinesV(Vector2 position, float radius, Color color)
{
  DrawCircleLines(static_cast<int>(position.x), static_cast<int>(position.y),
                  radius, color);
}

} // namespace ai

#endif // _DRAW_TRIANGLE_PRO_
