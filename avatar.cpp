// avatar.cpp - Sims-style ragdoll avatar template (C++ + raylib 5.x)
//
// The avatar is a Verlet-physics skeleton (joints = particles, bones = distance
// constraints). Every body part is drawn from basic shapes: spheres + capsules.
//
// Controls
//   SPACE        toggle standing pose <-> ragdoll
//   R            reset to standing pose
//   Left mouse   grab / drag / throw a joint (auto-enables ragdoll)
//   Right mouse  orbit camera      Wheel: zoom
//   UP / DOWN    height            LEFT / RIGHT: build
//   K / H / C / P  cycle skin / hair / shirt / pants colours
 
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <algorithm>
 
// ----------------------------------------------------------------------------
// Template data
// ----------------------------------------------------------------------------
enum Joint {
    HEAD, NECK,
    L_SHO, R_SHO, L_ELB, R_ELB, L_HAND, R_HAND,
    PELVIS, L_HIP, R_HIP, L_KNEE, R_KNEE, L_FOOT, R_FOOT,
    CHEST_F,            // invisible point in front of the chest; keeps the torso 3D-rigid
    JOINT_COUNT
};
 
struct Appearance {
    float height = 1.75f;   // metres
    float build  = 1.0f;    // width multiplier (0.7 slim .. 1.5 broad)
    Color skin   {241, 194, 125, 255};
    Color hair   { 60,  40,  30, 255};
    Color shirt  { 70, 130, 200, 255};
    Color pants  { 50,  60,  90, 255};
    Color shoes  {235, 235, 235, 255};
};
 
static const Color SKINS[]  = {{255,224,196,255},{241,194,125,255},{198,134,66,255},{141,85,36,255},{92,58,30,255}};
static const Color HAIRS[]  = {{40,30,25,255},{110,70,30,255},{200,160,80,255},{170,60,40,255},{200,200,200,255}};
static const Color SHIRTS[] = {{70,130,200,255},{200,70,70,255},{80,170,110,255},{230,200,70,255},{150,100,200,255}};
static const Color PANTS[]  = {{50,60,90,255},{60,60,60,255},{110,90,60,255},{40,80,60,255}};
template <typename T, size_t N> constexpr int Count(const T (&)[N]) { return (int)N; }
 
// ----------------------------------------------------------------------------
// Ragdoll
// ----------------------------------------------------------------------------
static const int   STEPS = 2;           // physics sub-steps per frame
static const float DT    = 1.0f / 120.0f;
static const int   ITERS = 14;          // constraint iterations per step
 
struct Stick { int a, b; float lo, hi; };   // allowed length range [lo, hi]
 
struct Ragdoll {
    Appearance look;
    Vector3 rest[JOINT_COUNT];
    Vector3 pos[JOINT_COUNT];
    Vector3 prev[JOINT_COUNT];
    std::vector<Stick> sticks;
    bool limp = false;
    int  grabbed = -1;
 
    bool  Pinned(int i) const { return !limp || i == grabbed; }
    float Radius(int j) const { return (j == HEAD ? 0.065f : 0.03f) * look.height; }
 
    // Add a distance constraint. loFrac/hiFrac are fractions of the rest length.
    void Link(int a, int b, float loFrac = 1.0f, float hiFrac = 1.0f) {
        float d = Vector3Distance(rest[a], rest[b]);
        sticks.push_back({a, b, d * loFrac, d * hiFrac});
    }
 
    void Build(const Appearance& a) {
        look = a;
        const float H = a.height, b = a.build;
        auto P = [&](Joint j, float x, float y, float z) { rest[j] = {x * H, y * H, z * H}; };
 
        // Character faces +Z, its left side is +X.
        P(HEAD,    0.00f,        0.935f, 0.010f);
        P(NECK,    0.00f,        0.855f, 0.000f);
        P(L_SHO,  +0.11f * b,    0.820f, 0.000f);
        P(R_SHO,  -0.11f * b,    0.820f, 0.000f);
        P(L_ELB,  +0.125f * b,   0.650f, 0.000f);
        P(R_ELB,  -0.125f * b,   0.650f, 0.000f);
        P(L_HAND, +0.137f * b,   0.500f, 0.020f);
        P(R_HAND, -0.137f * b,   0.500f, 0.020f);
        P(PELVIS,  0.00f,        0.515f, 0.000f);
        P(L_HIP,  +0.055f * b,   0.530f, 0.000f);
        P(R_HIP,  -0.055f * b,   0.530f, 0.000f);
        P(L_KNEE, +0.060f * b,   0.285f, 0.010f);
        P(R_KNEE, -0.060f * b,   0.285f, 0.010f);
        P(L_FOOT, +0.060f * b,   0.040f, 0.000f);
        P(R_FOOT, -0.060f * b,   0.040f, 0.000f);
        P(CHEST_F, 0.00f,        0.700f, 0.090f * b);
 
        sticks.clear();
        // Head: loose, so it can flop a little
        Link(HEAD, NECK);
        Link(HEAD, L_SHO, 0.92f, 1.10f);
        Link(HEAD, R_SHO, 0.92f, 1.10f);
        // Shoulder girdle
        Link(NECK, L_SHO); Link(NECK, R_SHO); Link(L_SHO, R_SHO);
        // Arms (min limit stops the arm folding through itself)
        Link(L_SHO, L_ELB); Link(L_ELB, L_HAND); Link(L_SHO, L_HAND, 0.20f, 10.0f);
        Link(R_SHO, R_ELB); Link(R_ELB, R_HAND); Link(R_SHO, R_HAND, 0.20f, 10.0f);
        // Rigid torso: triangulated + braced by the chest-front point
        Link(NECK, PELVIS);  Link(L_SHO, PELVIS); Link(R_SHO, PELVIS);
        Link(L_SHO, L_HIP);  Link(R_SHO, R_HIP);  Link(L_SHO, R_HIP); Link(R_SHO, L_HIP);
        Link(NECK, L_HIP);   Link(NECK, R_HIP);
        Link(PELVIS, L_HIP); Link(PELVIS, R_HIP); Link(L_HIP, R_HIP);
        for (int j : {NECK, L_SHO, R_SHO, PELVIS, L_HIP, R_HIP}) Link(CHEST_F, j);
        // Legs
        Link(L_HIP, L_KNEE); Link(L_KNEE, L_FOOT); Link(L_HIP, L_FOOT, 0.25f, 10.0f);
        Link(R_HIP, R_KNEE); Link(R_KNEE, R_FOOT); Link(R_HIP, R_FOOT, 0.25f, 10.0f);
 
        Stand();
    }
 
    void Stand() {
        limp = false; grabbed = -1;
        for (int i = 0; i < JOINT_COUNT; i++) pos[i] = prev[i] = rest[i];
    }
 
    void GoLimp() {
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
        limp = true;
    }
 
    void SolveSticks() {
        for (const Stick& s : sticks) {
            Vector3 d = Vector3Subtract(pos[s.b], pos[s.a]);
            float len = Vector3Length(d);
            if (len < 1e-6f) continue;
            float target = len < s.lo ? s.lo : (len > s.hi ? s.hi : len);
            if (target == len) continue;
            float wa = Pinned(s.a) ? 0.0f : 1.0f;
            float wb = Pinned(s.b) ? 0.0f : 1.0f;
            float w = wa + wb;
            if (w == 0.0f) continue;
            Vector3 corr = Vector3Scale(d, (len - target) / len / w);
            pos[s.a] = Vector3Add(pos[s.a], Vector3Scale(corr, wa));
            pos[s.b] = Vector3Subtract(pos[s.b], Vector3Scale(corr, wb));
        }
    }
 
    void Step(float dt) {
        if (!limp) return;
        const Vector3 g = {0.0f, -9.81f, 0.0f};
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (Pinned(i)) continue;
            Vector3 vel = Vector3Scale(Vector3Subtract(pos[i], prev[i]), 0.996f);
            prev[i] = pos[i];
            pos[i] = Vector3Add(Vector3Add(pos[i], vel), Vector3Scale(g, dt * dt));
        }
        for (int it = 0; it < ITERS; it++) {
            SolveSticks();
            for (int i = 0; i < JOINT_COUNT; i++) {          // floor
                if (Pinned(i)) continue;
                pos[i].y = std::max(pos[i].y, Radius(i));
            }
        }
        for (int i = 0; i < JOINT_COUNT; i++) {              // friction (once per step)
            if (Pinned(i) || pos[i].y > Radius(i) + 1e-4f) continue;
            prev[i].x += (pos[i].x - prev[i].x) * 0.30f;
            prev[i].z += (pos[i].z - prev[i].z) * 0.30f;
            prev[i].y = pos[i].y;
        }
    }
 
    void Release() {
        if (grabbed < 0) return;
        Vector3 v = Vector3Scale(Vector3Subtract(pos[grabbed], prev[grabbed]), 1.0f / STEPS);
        float l = Vector3Length(v);
        if (l > 0.25f) v = Vector3Scale(v, 0.25f / l);
        prev[grabbed] = Vector3Subtract(pos[grabbed], v);
        grabbed = -1;
    }
 
    // ------------------------------------------------------------------------
    void Draw() const {
        const float H = look.height, b = look.build, s = std::sqrt(b);
        auto Cap = [](Vector3 a, Vector3 c, float r, Color col) { DrawCapsule(a, c, r, 12, 6, col); };
        auto Ball = [](Vector3 c, float r, Color col) { DrawSphereEx(c, r, 14, 14, col); };
 
        // Body frame from the torso
        Vector3 shoMid = Vector3Lerp(pos[L_SHO], pos[R_SHO], 0.5f);
        Vector3 up  = Vector3Normalize(Vector3Subtract(pos[NECK], pos[PELVIS]));
        Vector3 fwd = Vector3Subtract(pos[CHEST_F], Vector3Lerp(pos[NECK], pos[PELVIS], 0.4f));
        fwd = Vector3Normalize(Vector3Subtract(fwd, Vector3Scale(up, Vector3DotProduct(fwd, up))));
        Vector3 right = Vector3CrossProduct(fwd, up);
 
        // Torso + pants waist
        Cap(Vector3Lerp(shoMid, pos[PELVIS], 0.12f), Vector3Lerp(shoMid, pos[PELVIS], 0.72f), 0.072f * H * b, look.shirt);
        Cap(Vector3Lerp(shoMid, pos[PELVIS], 0.72f), pos[PELVIS], 0.066f * H * b, look.pants);
        Cap(pos[L_HIP], pos[R_HIP], 0.055f * H * s, look.pants);
        Cap(pos[L_SHO], pos[R_SHO], 0.040f * H * s, look.shirt);
 
        // Neck + head
        Cap(pos[NECK], pos[HEAD], 0.022f * H, look.skin);
        const float R = 0.065f * H;
        Vector3 hc = pos[HEAD];
        Ball(hc, R, look.skin);
        Ball(Vector3Add(hc, Vector3Add(Vector3Scale(up, 0.35f * R), Vector3Scale(fwd, -0.30f * R))), 1.04f * R, look.hair);
        for (float side : {-1.0f, 1.0f}) {
            Vector3 e = Vector3Add(hc, Vector3Scale(fwd, 0.88f * R));
            e = Vector3Add(e, Vector3Add(Vector3Scale(right, side * 0.35f * R), Vector3Scale(up, 0.08f * R)));
            Ball(e, 0.13f * R, BLACK);
        }
 
        // Arms: short sleeves
        const int sho[2] = {L_SHO, R_SHO}, elb[2] = {L_ELB, R_ELB}, han[2] = {L_HAND, R_HAND};
        for (int k = 0; k < 2; k++) {
            Ball(pos[sho[k]], 0.036f * H * s, look.shirt);
            Cap(pos[sho[k]], pos[elb[k]], 0.028f * H * s, look.shirt);
            Ball(pos[elb[k]], 0.026f * H * s, look.skin);
            Cap(pos[elb[k]], pos[han[k]], 0.024f * H * s, look.skin);
            Ball(pos[han[k]], 0.030f * H, look.skin);
        }
 
        // Legs + shoes
        const int hip[2] = {L_HIP, R_HIP}, kne[2] = {L_KNEE, R_KNEE}, foo[2] = {L_FOOT, R_FOOT};
        for (int k = 0; k < 2; k++) {
            Cap(pos[hip[k]], pos[kne[k]], 0.045f * H * s, look.pants);
            Ball(pos[kne[k]], 0.040f * H * s, look.pants);
            Cap(pos[kne[k]], pos[foo[k]], 0.035f * H * s, look.pants);
            Vector3 start = Vector3Subtract(pos[foo[k]], Vector3Add(Vector3Scale(up, 0.025f * H), Vector3Scale(fwd, 0.015f * H)));
            Cap(start, Vector3Add(start, Vector3Scale(fwd, 0.11f * H)), 0.032f * H, look.shoes);
        }
    }
};
 
// ----------------------------------------------------------------------------
int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1100, 720, "Ragdoll Avatar Template");
    SetTargetFPS(60);
 
    int skinI = 1, hairI = 0, shirtI = 0, pantsI = 0;
    Appearance look;
    auto ApplyColours = [&](Appearance& a) {
        a.skin = SKINS[skinI]; a.hair = HAIRS[hairI]; a.shirt = SHIRTS[shirtI]; a.pants = PANTS[pantsI];
    };
    ApplyColours(look);
 
    Ragdoll avatar;
    avatar.Build(look);
 
    float yaw = 0.6f, pitch = 0.25f, dist = 4.0f;
    float grabDist = 0.0f;
 
    while (!WindowShouldClose()) {
        // ---- camera ----
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            yaw   -= d.x * 0.005f;
            pitch += d.y * 0.005f;
            pitch = Clamp(pitch, -0.1f, 1.45f);
        }
        dist = Clamp(dist - GetMouseWheelMove() * 0.3f, 1.5f, 10.0f);
 
        Camera3D cam = {};
        Vector3 target = {0.0f, 0.55f * avatar.look.height, 0.0f};
        cam.target = target;
        cam.position = {target.x + dist * std::cos(pitch) * std::sin(yaw),
                        target.y + dist * std::sin(pitch),
                        target.z + dist * std::cos(pitch) * std::cos(yaw)};
        cam.up = {0.0f, 1.0f, 0.0f};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;
 
        // ---- template editing (rebuilds the skeleton) ----
        bool rebuild = false;
        if (IsKeyPressed(KEY_UP))    { look.height = Clamp(look.height + 0.05f, 1.20f, 2.10f); rebuild = true; }
        if (IsKeyPressed(KEY_DOWN))  { look.height = Clamp(look.height - 0.05f, 1.20f, 2.10f); rebuild = true; }
        if (IsKeyPressed(KEY_RIGHT)) { look.build  = Clamp(look.build  + 0.10f, 0.70f, 1.50f); rebuild = true; }
        if (IsKeyPressed(KEY_LEFT))  { look.build  = Clamp(look.build  - 0.10f, 0.70f, 1.50f); rebuild = true; }
        if (IsKeyPressed(KEY_K)) skinI  = (skinI  + 1) % Count(SKINS);
        if (IsKeyPressed(KEY_H)) hairI  = (hairI  + 1) % Count(HAIRS);
        if (IsKeyPressed(KEY_C)) shirtI = (shirtI + 1) % Count(SHIRTS);
        if (IsKeyPressed(KEY_P)) pantsI = (pantsI + 1) % Count(PANTS);
        ApplyColours(look);
        avatar.look.skin = look.skin; avatar.look.hair = look.hair;
        avatar.look.shirt = look.shirt; avatar.look.pants = look.pants;
        if (rebuild) avatar.Build(look);
 
        // ---- pose / physics control ----
        if (IsKeyPressed(KEY_R)) avatar.Stand();
        if (IsKeyPressed(KEY_SPACE)) { if (avatar.limp) avatar.Stand(); else avatar.GoLimp(); }
 
        // ---- mouse grab ----
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetMouseRay(GetMousePosition(), cam);
            float best = 1e9f; int hit = -1;
            for (int i = 0; i < JOINT_COUNT; i++) {
                RayCollision c = GetRayCollisionSphere(ray, avatar.pos[i], 0.07f * avatar.look.height);
                if (c.hit && c.distance < best) { best = c.distance; hit = i; }
            }
            if (hit >= 0) {
                if (!avatar.limp) avatar.GoLimp();
                avatar.grabbed = hit;
                grabDist = best;
            }
        }
        if (avatar.grabbed >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetMouseRay(GetMousePosition(), cam);
            Vector3 t = Vector3Add(ray.position, Vector3Scale(ray.direction, grabDist));
            t.y = std::max(t.y, 0.03f);
            avatar.prev[avatar.grabbed] = avatar.pos[avatar.grabbed];
            avatar.pos[avatar.grabbed]  = t;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) avatar.Release();
 
        for (int i = 0; i < STEPS; i++) avatar.Step(DT);
 
        // ---- draw ----
        BeginDrawing();
        ClearBackground({205, 220, 235, 255});
        BeginMode3D(cam);
            DrawPlane({0.0f, 0.0f, 0.0f}, {12.0f, 12.0f}, {170, 180, 165, 255});
            DrawGrid(24, 0.5f);
            avatar.Draw();
        EndMode3D();
 
        DrawText("SPACE ragdoll/stand   R reset   LMB drag joint   RMB orbit   Wheel zoom", 12, 10, 18, DARKGRAY);
        DrawText("UP/DOWN height   LEFT/RIGHT build   K skin  H hair  C shirt  P pants", 12, 34, 18, DARKGRAY);
        DrawText(TextFormat("height %.2f m   build %.1f   %s", look.height, look.build,
                            avatar.limp ? "RAGDOLL" : "STANDING"), 12, 58, 18, DARKGRAY);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}