// avatar.cpp - Sims-style ragdoll avatar template (C++ + raylib 5.x)
//
// The avatar is a Verlet-physics skeleton (joints = particles, bones = distance
// constraints). Every body part is drawn from basic shapes: spheres + capsules.
//
// Fall / get-up cycle:
//   STANDING --(knocked down)--> RAGDOLL --(on the ground for 0.5 s)--> GETUP --> STANDING
// During GETUP the body is driven through keyframes (crouch -> rise -> stand) with
// 2-bone IK for the arms and legs, while the constraint solver keeps the bones
// intact. It stands up wherever it fell, facing the direction it ends up facing.
//
// Controls
//   SPACE        knock the avatar over
//   R            hard reset to standing at the origin
//   Left mouse   grab / drag / throw a joint (auto-enables ragdoll)
//   Drag on empty space (or hold right mouse) to turn the camera      Wheel: zoom
//   Screen on the -Z wall at his eye level, as wide as he is tall. Click it to switch it on / off. It plays screen.gif
//   (next to the program, or pass a path as the first argument) at a fixed 10 frames/s; until then a test pattern.
//   Click the wall panel just BELOW the screen: he walks to a spot in front of it, sits on the floor and watches.
//   Entertainment drains slowly; while he sits watching with the screen ON it refills slowly; with the screen OFF it
//   does not. He gets up when entertained (100), sleepy, when there is food on the floor, or after 8 s of a dead screen.
//   Brick pile (5 wide x 8 long x 4 high) in the corner opposite the bed: pick up, drag and throw them with the mouse.
//   Lift one out and whatever rested on it falls; they stack and slide. Bricks collide with the avatar: a hard enough hit
//   costs mood and he puts a hand to the spot and rubs it (higher spot = stands tall, lower = hunches over). A ragdoll
//   avatar is pushed by the brick; a standing one is not (the brick bounces off).
//   Disks can be picked up, dragged and thrown with the mouse, like the avatar. They bounce off the walls and can land on the bed.
//   While the mouse holds the avatar, his free hands go for a target (the nearest disk, or the bed if he is sleepy):
//     held in the air          -> the free hands reach out for it
//     dragged along the ground by a leg -> he crawls, hands alternating
//   Sleepiness at 0 for 10 s: he drops (even from the mouse) and sleeps where he is; at 100 he wakes and gets up.
//   X            test shortcut: set sleepiness to 0
//   [Drop food] button (top of the screen): a pink disk drops onto a random spot of the grass. A standing avatar
//                walks to it, crouches, picks it up, holds it to his mouth for 1 s, and it disappears.
//   Height keys (UP/DOWN) are switched off for now (HEIGHT_KEYS).
//   Bed (corner): carry the avatar onto the bed and let go. Lying still for 1 s = asleep.
//                sleepiness refills while asleep; at 100 it wakes and stands up beside the bed.
//   Mood > 60: he dances for 2.5 s at random intervals.
//   Mood < 10: he cries continuously (hands over his face).
//   Mood 10-25: he cries for 1 s, once at a random moment in every 7 s.
//   B / M / N    test shortcuts: set mood to 5 (miserable) / 15 (sad) / 100 (happy)
//   Z            make him sleepy right now (sets sleepiness to the go-to-bed level)
//   Below 25 sleepiness a standing avatar walks to the bed, sits on the edge, lies down and sleeps.
//   LEFT / RIGHT build
//   K / H / C / P  cycle skin / hair / shirt / pants colours
 
#include "raylib.h"
#include "rlgl.h"          // for drawing the textured screen
 
// raylib 5.5 renamed GetMouseRay to GetScreenToWorldRay (and the old name was dropped in 6.x), so pick the right one.
#if defined(RAYLIB_VERSION_MAJOR) && (RAYLIB_VERSION_MAJOR > 5 || (RAYLIB_VERSION_MAJOR == 5 && RAYLIB_VERSION_MINOR >= 5))
  #define RAYLIB_HAS_SCREEN_TO_WORLD_RAY 1
#endif
#include "raymath.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <cstring>
 
#ifdef RAYLIB_HAS_SCREEN_TO_WORLD_RAY
static inline Ray ScreenRay(Vector2 p, Camera3D c) { return GetScreenToWorldRay(p, c); }
#else
static inline Ray ScreenRay(Vector2 p, Camera3D c) { return GetMouseRay(p, c); }
#endif
 
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
 
enum Mode { MODE_STANDING, MODE_RAGDOLL, MODE_GETUP, MODE_WALK, MODE_BEDANIM, MODE_FETCH, MODE_EAT, MODE_TV };
enum TvStage { TV_WALK, TV_SIT, TV_WATCH, TV_RISE };   // within MODE_TV
 
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
// Keyframes for the stand-up animation (all lengths in units of body height)
// ----------------------------------------------------------------------------
struct PoseKey {
    float pelvisY, pelvisZ;     // pelvis position (local frame, +Z = forward)
    float lean;                 // torso pitch forward, degrees
    float footX, footY, footZ;  // foot target (X mirrored, scaled by build)
    float handX, handY, handZ;  // hand target (X mirrored, scaled by build)
};
 
static const PoseKey KEY_CROUCH = {0.20f, -0.08f, 65.0f, 0.06f, 0.04f, 0.05f, 0.13f, 0.03f, 0.30f};
static const PoseKey KEY_RISE   = {0.38f, -0.02f, 25.0f, 0.06f, 0.04f, 0.03f, 0.12f, 0.45f, 0.18f};
static const PoseKey KEY_STAND  = {0.515f, 0.00f,  0.0f, 0.06f, 0.04f, 0.00f, 0.145f, 0.52f, 0.03f};
// Eating: deep crouch to pick a disk off the floor (right hand is steered separately), then upright with a slight bow
static const PoseKey KEY_PICK   = {0.170f, -0.10f, 66.0f, 0.07f, 0.04f, 0.02f, 0.145f, 0.12f, 0.15f};
static const PoseKey KEY_MOUTH  = {0.515f, 0.00f,  6.0f, 0.06f, 0.04f, 0.00f, 0.145f, 0.52f, 0.03f};
// Sitting on the floor to watch TV: knees up, feet flat in front, hands resting on the knees, leaning back a little.
// (The pelvis ends up behind where he stood, so the feet stay roughly where they were.)
static const PoseKey KEY_SIT_MID = {0.240f, -0.12f, 22.0f, 0.07f, 0.04f, 0.04f, 0.12f, 0.36f, 0.00f};
static const PoseKey KEY_SIT     = {0.070f, -0.22f, -6.0f, 0.10f, 0.03f, 0.06f, 0.10f, 0.31f, -0.05f};
 
static PoseKey KeyLerp(const PoseKey& a, const PoseKey& b, float t) {
    PoseKey r;
    float* ra = &r.pelvisY; const float* pa = &a.pelvisY; const float* pb = &b.pelvisY;
    for (int i = 0; i < 9; i++) ra[i] = pa[i] + (pb[i] - pa[i]) * t;
    return r;
}
static float Smooth(float t) { t = Clamp(t, 0.0f, 1.0f); return t * t * (3.0f - 2.0f * t); }
 
// Extra adjustments layered on a PoseKey (lengths in body heights, local frame): used by emotes.
struct PoseTweak {
    float   pelvisX   = 0.0f;                    // sideways hip sway
    Vector3 lHand = {0, 0, 0}, rHand = {0, 0, 0};   // added to the hand targets
    Vector3 lFoot = {0, 0, 0}, rFoot = {0, 0, 0};   // added to the foot targets
    float   coverFace = 0.0f;                    // 0..1: bring both hands up over the eyes
    Vector3 rHandAt = {0, 0, 0};                 // absolute local target (body heights) for the right hand...
    float   rHandAtW  = 0.0f;                    // ...and how strongly it is pulled there (0..1)
    float   rToMouth  = 0.0f;                    // 0..1: then pulled on up to the mouth
    float   rBendDown = 0.0f;                    // 0..1: right elbow points out/down instead of back
    // Pain: one hand goes to the spot on his body that was hit and rubs it
    int     painA = -1, painB = -1;              // the bone the spot is on...
    float   painSeg = 0.0f;                      // ...and how far along it (0..1)
    Vector3 painDir = {0, 0, 0};                 // world direction from the bone to the contact point
    float   painRadius = 0.0f;                   // how far out (the body surface)
    int     painHand = 0;                        // 0 = left hand, 1 = right hand
    float   painBlend = 0.0f;                    // 0..1
    float   painClock = 0.0f;                    // drives the rubbing
};
 
// Timing (seconds)
static const float FALL_WAIT      = 0.5f;   // time on the ground before getting up
static const float GETUP_CROUCH   = 0.8f;   // tuck + hands down
static const float GETUP_RISE_END = 1.4f;   // crouch -> rise
static const float GETUP_STAND_END= 2.0f;   // rise -> stand
static const float GETUP_TOTAL    = 2.2f;   // incl. settling time
static const float GETUP_GAIN     = 0.15f;  // how strongly joints are pulled to the animation per step
 
static PoseKey GetUpKey(float t) {
    if (t < GETUP_CROUCH)    return KEY_CROUCH;
    if (t < GETUP_RISE_END)  return KeyLerp(KEY_CROUCH, KEY_RISE,  Smooth((t - GETUP_CROUCH)   / (GETUP_RISE_END  - GETUP_CROUCH)));
    if (t < GETUP_STAND_END) return KeyLerp(KEY_RISE,   KEY_STAND, Smooth((t - GETUP_RISE_END) / (GETUP_STAND_END - GETUP_RISE_END)));
    return KEY_STAND;
}
 
// Two-bone IK: returns the middle joint (knee/elbow) for a chain a -> c.
// If c is out of reach it is pulled in (passed by reference).
static Vector3 TwoBone(Vector3 a, Vector3& c, float l1, float l2, Vector3 bend) {
    Vector3 d = Vector3Subtract(c, a);
    float dist = Vector3Length(d);
    Vector3 dir;
    if (dist < 1e-5f) { dir = {0.0f, -1.0f, 0.0f}; dist = 0.02f; c = Vector3Add(a, Vector3Scale(dir, dist)); }
    else dir = Vector3Scale(d, 1.0f / dist);
    float maxR = (l1 + l2) * 0.9999f;
    if (dist > maxR) { dist = maxR; c = Vector3Add(a, Vector3Scale(dir, dist)); }
    float x = (l1 * l1 - l2 * l2 + dist * dist) / (2.0f * dist);
    float h = std::sqrt(std::max(l1 * l1 - x * x, 0.0f));
    Vector3 p = Vector3Subtract(bend, Vector3Scale(dir, Vector3DotProduct(bend, dir)));
    float pl = Vector3Length(p);
    if (pl > 1e-5f) p = Vector3Scale(p, 1.0f / pl); else p = {1.0f, 0.0f, 0.0f};
    return Vector3Add(a, Vector3Add(Vector3Scale(dir, x), Vector3Scale(p, h)));
}
 
// ----------------------------------------------------------------------------
// Ragdoll
// ----------------------------------------------------------------------------
static const int   STEPS = 2;           // physics sub-steps per frame
static const float DT    = 1.0f / 120.0f;
static const int   ITERS = 14;          // constraint iterations per step
 
// Arena: a square of grass enclosed by walls (inner faces at +/-ARENA metres)
static const float ARENA  = 12.0f;
static const float WALL_H = 1.0f;
static const float WALL_T = 0.3f;
 
// Solid objects are axis-aligned boxes that joints cannot enter.
struct Box { Vector3 min, max; };
 
// The bed sits in the +X/+Z corner with its head against the +Z wall.
static const float BED_W = 1.1f, BED_L = 2.1f, BED_H = 0.5f;
static const Box   BED   = {{ARENA - BED_W, 0.0f, ARENA - BED_L}, {ARENA, BED_H, ARENA}};
 
// Sleep / mood tuning
static const float SLEEP_DROP     = 1.0f;   // sleepiness points lost per tick...
static const float SLEEP_INTERVAL = 3.0f;   // ...every this many seconds
static const float SLEEP_REGEN    = 2.0f;   // points per second gained while asleep in bed
static const float HELD_MOOD_DRAIN= 2.0f;   // mood points per second lost while held by the mouse
static const float MOOD_REGEN_INTERVAL = 2.0f;   // mood recovers 1 point every this many seconds
 
// Emotes (standing only)
static const float MOOD_DANCE_ABOVE = 60.0f;    // above this he dances now and then
static const float MOOD_CRY_BELOW   = 25.0f;    // below this he starts crying (a brief gesture now and then)...
static const float MOOD_CRY_CONT    = 10.0f;    // ...and below this he cries continuously
static const float CRY_BURST_TIME   = 1.0f;     // length of the occasional cry gesture
static const float CRY_WINDOW       = 7.0f;     // one gesture at a random moment in every window of this length
static const float DANCE_TIME       = 2.5f;     // seconds per dance
 
// Going to bed: below this sleepiness the avatar walks to the bed by itself
static const float SLEEPY_AT   = 25.0f;
static const float WALK_SPEED  = 1.1f;      // m/s
static const float TURN_RATE   = 4.0f;      // rad/s
static const float BED_LIE_Z   = BED.max.z - 0.95f;   // where along the bed the pelvis ends up lying
static const float BED_SIT_T   = 1.2f;      // seconds: turn around + sit on the edge
static const float BED_LIE_T   = 1.5f;      // seconds: lie back and swing the legs up
 
static float WrapPi(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}
 
// Does the straight walk a -> b (ground plane) cut through the bed (plus a margin)?
static bool SegmentHitsBed(Vector3 a, Vector3 b, float margin) {
    float t0 = 0.0f, t1 = 1.0f;
    const float dx = b.x - a.x, dz = b.z - a.z;
    auto slab = [&](float p0, float d, float lo, float hi) {
        if (std::fabs(d) < 1e-6f) return p0 >= lo && p0 <= hi;
        float ta = (lo - p0) / d, tb = (hi - p0) / d;
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta); t1 = std::min(t1, tb);
        return t0 <= t1;
    };
    return slab(a.x, dx, BED.min.x - margin, BED.max.x + margin) &&
           slab(a.z, dz, BED.min.z - margin, BED.max.z + margin);
}
 
// Food: a small pink disk dropped from above. Drawn the size of his hand.
enum FoodState { FOOD_FALLING, FOOD_LANDED, FOOD_TARGETED, FOOD_DRAGGED };
struct Food { Vector3 pos; Vector3 vel; int state; };   // pos = centre of the disk
 
// Entertainment: drains slowly; refills only while he sits watching the TV with the screen switched on
static const float ENT_DROP        = 1.0f;      // points lost per tick...
static const float ENT_INTERVAL    = 2.0f;      // ...every this many seconds
static const float ENT_REGEN       = 2.0f;      // points per second gained while watching (screen on)
static const float TV_DISTANCE     = 2.8f;      // how far from the wall he stands before sitting down (m)
static const float TV_SIT_T        = 1.8f;      // seconds to sit down (and to get back up)
static const float TV_OFF_PATIENCE = 8.0f;      // seconds he keeps staring at a switched-off screen before giving up
static const float PASS_OUT_AFTER = 10.0f;  // seconds at zero sleepiness before he collapses and sleeps where he stands
static const float FETCH_STOP = 0.34f;      // how far from the food he stops before crouching (m)
static const bool  HEIGHT_KEYS = false;     // the height keys are switched off for now
 
static bool OverBed(Vector3 p) { return p.x > BED.min.x && p.x < BED.max.x && p.z > BED.min.z && p.z < BED.max.z; }
 
// Push a sphere (centre p, radius r) out of a box along the axis of least penetration.
// The floor is handled elsewhere, so only the sides and the top matter.
static void PushOutOfBox(Vector3& p, float r, const Box& b) {
    float dx0 = p.x - (b.min.x - r), dx1 = (b.max.x + r) - p.x;
    float dz0 = p.z - (b.min.z - r), dz1 = (b.max.z + r) - p.z;
    float dy1 = (b.max.y + r) - p.y;
    if (dx0 <= 0.0f || dx1 <= 0.0f || dz0 <= 0.0f || dz1 <= 0.0f || dy1 <= 0.0f) return;   // outside
    float m = std::min(std::min(std::min(dx0, dx1), std::min(dz0, dz1)), dy1);
    if      (m == dy1) p.y = b.max.y + r;
    else if (m == dx0) p.x = b.min.x - r;
    else if (m == dx1) p.x = b.max.x + r;
    else if (m == dz0) p.z = b.min.z - r;
    else               p.z = b.max.z + r;
}
 
// A sphere on his body that bricks can hit: the point a + t*(b - a) of a bone, with a radius.
struct Probe { int a, b; float t; float r; int armSide; };    // armSide: 0 = left arm, 1 = right arm, -1 = elsewhere
 
static const float PAIN_TIME         = 2.0f;    // seconds he rubs the sore spot
static const float BRICK_HURT_SPEED  = 1.5f;    // closing speed (m/s) below which a brick doesn't hurt
 
struct Stick { int a, b; float lo, hi; };   // allowed length range [lo, hi]
 
struct Ragdoll {
    Appearance look;
    Vector3 rest[JOINT_COUNT];
    Vector3 pos[JOINT_COUNT];
    Vector3 prev[JOINT_COUNT];
    std::vector<Stick> sticks;
 
    Mode  mode = MODE_STANDING;
    int   grabbed = -1;
    float groundTimer = 0.0f;   // how long the avatar has been down
    float getupT = 0.0f;        // animation clock
    Vector3 origin = {0.0f, 0.0f, 0.0f};   // where the avatar stands (XZ)
    float yaw = 0.0f;                      // which way it faces when standing
 
    // Collision tracking (feeds the mood bar). The game loop reads and clears pendingMoodHit.
    float pendingMoodHit = 0.0f;           // mood points lost to collisions since last read
    float impactCooldown = 0.0f;           // merges the many contacts of one fall into one collision
 
    // Pain from being hit by a brick
    float painTimer = 0.0f;                // seconds left of the pain reaction
    float painBlend = 0.0f;                // 0..1 ease in/out of the hand-to-spot pose (while on his feet)
    int   painA = 0, painB = 0;            // bone the sore spot is on
    float painSeg = 0.0f;                  // position along that bone
    Vector3 painDir = {0, 0, 0};           // world direction from the bone to where the brick touched
    float painRadius = 0.0f;
    int   painHand = 0;                    // which hand rubs it (0 = left)
    float brickHitCooldown = 0.0f;         // one hit at a time
    std::function<void(Vector3&)> adjustOrigin;   // lets the game loop move his get-up spot to somewhere free
 
    // Bed interaction
    float bedTimer = 0.0f;                 // time spent lying still on the mattress
    int   walkStage = 0;                   // 0 = pick route, 1 = via the corner waypoint, 2 = to the bedside
    float gaitPhase = 0.0f;                // walk cycle position (radians)
    float gaitAmp   = 0.0f;                // 0 = standing, 1 = full stride (ramps in/out)
    float bedT      = 0.0f;                // clock of the bed animation (runs backwards when getting out)
    bool  bedReverse = false;              // true while getting OUT of bed
    float wakeBlend = 0.0f;                // 0..1: eases from wherever he was lying into the animation
    Vector3 wakeStart[JOINT_COUNT];
 
    // Held by the mouse: the free hands react to a target (food, or the bed when sleepy)
    bool    hasTarget = false;
    Vector3 targetPos = {0, 0, 0};
    float   activeClock = 0.0f;
    bool    passedOut = false;                 // collapsed from exhaustion: sleeps wherever he is
 
    // Food fetching / eating
    Vector3 foodGoal = {0, 0, 0};              // world position of the food he is walking to
    int     fetchStage = 0;                    // 0 = pick route, 1 = via the bed's corner, 2 = to the food
    float   eatT = 0.0f;                       // clock of the crouch / pick up / eat animation
    Vector3 eatFoodLocal = {0, 0, 0};          // food position in his own frame (body heights)
    bool    foodHeld = false;                  // true while the disk is in his hand
    Vector3 foodPos = {0, 0, 0};               // where the held disk is (read by the game loop)
    bool    foodEaten = false;                 // set when the disk has been eaten (game loop clears it)
 
    // Watching TV
    Vector3 tvSpot = {0, 0, 0};                // where he stands (facing the screen) before sitting down
    int     tvStage = TV_WALK;
    float   tvT = 0.0f;                        // sit-down / get-up progress, or time spent watching
    bool    tvRiseRequested = false;           // the game loop asks him to get up (he finishes sitting down first)
 
    // Emotes (set moodLevel from the game loop each frame)
    float moodLevel   = 100.0f;
    float danceT      = 0.0f;                  // seconds of the current dance left (0 = not dancing)
    float danceClock  = 0.0f;                  // drives the dance moves
    float danceBlend  = 0.0f;                  // 0..1 ease in/out of the dance pose
    float cryBlend    = 0.0f;                  // 0..1 ease in/out of hands-on-face
    float emoteClock  = 0.0f;
    float nextDanceIn = 5.0f;                  // random countdown to the next dance
    float cryWindowT  = 0.0f;                  // position inside the current 7 s window (mood 10-25)
    float cryStartAt  = 2.0f;                  // when in the window the gesture starts
    float cryBurstT   = 0.0f;                  // seconds of the current cry gesture left
    bool  cryBurstDone = false;                // already cried this window
    bool  asleep   = false;
 
    bool  Pinned(int i) const { return mode == MODE_STANDING || mode == MODE_WALK || mode == MODE_BEDANIM || mode == MODE_FETCH || mode == MODE_EAT || mode == MODE_TV || i == grabbed; }
    float Radius(int j) const { return (j == HEAD ? 0.065f : 0.03f) * look.height; }
    float Bone(int a, int b) const { return Vector3Distance(rest[a], rest[b]); }
 
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
 
    // Generate a full-body pose from a keyframe (IK for the limbs), placed at
    // `org` (XZ) and rotated by `yw` about the Y axis.
    void ComputePose(const PoseKey& k, Vector3 org, float yw, Vector3 out[JOINT_COUNT],
                     float gait = 0.0f, float gaitStrength = 0.0f, const PoseTweak* tw = nullptr) const {
        const float H = look.height, b = look.build;
        const float cl = std::cos(k.lean * DEG2RAD), sl = std::sin(k.lean * DEG2RAD);
        const float cy = std::cos(yw), sy = std::sin(yw);
        Vector3 loc[JOINT_COUNT];
 
        // Torso + hips move as one rigid block, pitched about the pelvis
        Vector3 pel = {(tw ? tw->pelvisX : 0.0f) * H, k.pelvisY * H, k.pelvisZ * H};
        loc[PELVIS] = pel;
        for (int j : {HEAD, NECK, L_SHO, R_SHO, CHEST_F, L_HIP, R_HIP}) {
            Vector3 o = Vector3Subtract(rest[j], rest[PELVIS]);
            loc[j] = {pel.x + o.x,
                      pel.y + o.y * cl - o.z * sl,
                      pel.z + o.z * cl + o.y * sl};
        }
 
        // Feet and hands are targets; knees and elbows come from IK
        loc[L_FOOT] = {+k.footX * b * H, k.footY * H, k.footZ * H};
        loc[R_FOOT] = {-k.footX * b * H, k.footY * H, k.footZ * H};
        loc[L_HAND] = {+k.handX * b * H, k.handY * H, k.handZ * H};
        loc[R_HAND] = {-k.handX * b * H, k.handY * H, k.handZ * H};
 
        if (gaitStrength > 0.0f) {                       // walk cycle: feet swing + lift, arms swing the other way
            const float sL = std::sin(gait), cL = std::cos(gait);
            loc[L_FOOT].z += sL * 0.12f * H * gaitStrength;  loc[L_FOOT].y += std::max(0.0f,  cL) * 0.05f * H * gaitStrength;
            loc[R_FOOT].z -= sL * 0.12f * H * gaitStrength;  loc[R_FOOT].y += std::max(0.0f, -cL) * 0.05f * H * gaitStrength;
            loc[L_HAND].z -= sL * 0.09f * H * gaitStrength;
            loc[R_HAND].z += sL * 0.09f * H * gaitStrength;
        }
 
        const Vector3 back = {0.0f, 0.0f, -1.0f};
        Vector3 bendL = back, bendR = back;               // which way the elbows point
        if (tw) {
            loc[L_HAND] = Vector3Add(loc[L_HAND], Vector3Scale(tw->lHand, H));
            loc[R_HAND] = Vector3Add(loc[R_HAND], Vector3Scale(tw->rHand, H));
            loc[L_FOOT] = Vector3Add(loc[L_FOOT], Vector3Scale(tw->lFoot, H));
            loc[R_FOOT] = Vector3Add(loc[R_FOOT], Vector3Scale(tw->rFoot, H));
            if (tw->coverFace > 0.0f) {                   // hands over the eyes, elbows out and down
                const float c = tw->coverFace;
                const Vector3 faceDir = {0.0f, -sl, cl};  // where the face points once the torso leans
                Vector3 centre = Vector3Add(loc[HEAD], Vector3Scale(faceDir, 0.095f * H));
                centre.y += 0.005f * H;
                const Vector3 faceL = {centre.x + 0.035f * H, centre.y, centre.z};
                const Vector3 faceR = {centre.x - 0.035f * H, centre.y, centre.z};
                loc[L_HAND] = Vector3Lerp(loc[L_HAND], faceL, c);
                loc[R_HAND] = Vector3Lerp(loc[R_HAND], faceR, c);
                bendL = Vector3Lerp(back, {+0.4f, -1.0f, 0.1f}, c);
                bendR = Vector3Lerp(back, {-0.4f, -1.0f, 0.1f}, c);
            }
            if (tw->rHandAtW > 0.0f)                      // reach for something (the food)
                loc[R_HAND] = Vector3Lerp(loc[R_HAND], Vector3Scale(tw->rHandAt, H), tw->rHandAtW);
            if (tw->rToMouth > 0.0f) {                    // bring it up to the mouth
                const Vector3 faceDir = {0.0f, -sl, cl};
                Vector3 mouth = Vector3Add(loc[HEAD], Vector3Scale(faceDir, 0.093f * H));
                mouth.y -= 0.03f * H;
                loc[R_HAND] = Vector3Lerp(loc[R_HAND], mouth, tw->rToMouth);
            }
            if (tw->rBendDown > 0.0f) bendR = Vector3Lerp(bendR, {-0.4f, -1.0f, 0.1f}, tw->rBendDown);
        }
        // Knees point forward when the leg is vertical and UP when it is horizontal (sitting, lying), slightly outward.
        const Vector3 kneeL = {+0.12f, 1.0f, 1.0f}, kneeR = {-0.12f, 1.0f, 1.0f};
        loc[L_KNEE] = TwoBone(loc[L_HIP], loc[L_FOOT], Bone(L_HIP, L_KNEE), Bone(L_KNEE, L_FOOT), kneeL);
        loc[R_KNEE] = TwoBone(loc[R_HIP], loc[R_FOOT], Bone(R_HIP, R_KNEE), Bone(R_KNEE, R_FOOT), kneeR);
        loc[L_ELB]  = TwoBone(loc[L_SHO], loc[L_HAND], Bone(L_SHO, L_ELB),  Bone(L_ELB, L_HAND),  bendL);
        loc[R_ELB]  = TwoBone(loc[R_SHO], loc[R_HAND], Bone(R_SHO, R_ELB),  Bone(R_ELB, R_HAND),  bendR);
 
        if (tw && tw->painBlend > 0.0f && tw->painA >= 0) {      // ouch: a hand to the sore spot, rubbing it
            Vector3 spot = Vector3Lerp(loc[tw->painA], loc[tw->painB], tw->painSeg);
            const Vector3 dl = {tw->painDir.x * cy - tw->painDir.z * sy, tw->painDir.y,       // contact direction, world -> his frame
                                tw->painDir.x * sy + tw->painDir.z * cy};
            spot = Vector3Add(spot, Vector3Scale(dl, tw->painRadius));
            const float rub = tw->painClock * 2.0f * PI * 3.0f;
            spot.x += 0.012f * H * std::cos(rub);
            spot.y += 0.012f * H * std::sin(rub);
            const bool left = (tw->painHand == 0);
            const int hnd = left ? L_HAND : R_HAND, sho = left ? L_SHO : R_SHO, elb = left ? L_ELB : R_ELB;
            Vector3 target = Vector3Lerp(loc[hnd], spot, tw->painBlend);
            const Vector3 bend = Vector3Lerp(back, {left ? 0.4f : -0.4f, -1.0f, 0.1f}, tw->painBlend);
            loc[elb] = TwoBone(loc[sho], target, Bone(sho, elb), Bone(elb, hnd), bend);
            loc[hnd] = target;
        }
 
        for (int j = 0; j < JOINT_COUNT; j++) {
            out[j] = {org.x + loc[j].x * cy + loc[j].z * sy,
                      loc[j].y,
                      org.z - loc[j].x * sy + loc[j].z * cy};
        }
    }
 
    // Snap to the standing pose at the current origin/yaw
    void Stand() {
        mode = MODE_STANDING; grabbed = -1; groundTimer = 0.0f; getupT = 0.0f;
        bedTimer = 0.0f; asleep = false; passedOut = false; walkStage = 0; gaitPhase = 0.0f; gaitAmp = 0.0f; bedT = 0.0f; bedReverse = false;
        foodHeld = false; tvRiseRequested = false; danceT = 0.0f; danceBlend = 0.0f; cryBlend = 0.0f; cryWindowT = 0.0f; cryBurstT = 0.0f; cryBurstDone = false; nextDanceIn = (float)GetRandomValue(30, 100) / 10.0f;
        ComputePose(KEY_STAND, origin, yaw, pos);
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
    }
 
    void HardReset() { origin = {0.0f, 0.0f, 0.0f}; yaw = 0.0f; painTimer = 0.0f; painBlend = 0.0f; Stand(); }
 
    // Switch to physics. `push` gives the upper body a shove so it topples.
    void GoLimp(bool push) {
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
        mode = MODE_RAGDOLL; groundTimer = 0.0f; getupT = 0.0f; bedReverse = false; tvRiseRequested = false;
        if (push) {
            float a = (float)GetRandomValue(0, 628) / 100.0f;
            Vector3 dir = {std::sin(a), 0.0f, std::cos(a)};
            for (int j : {HEAD, NECK, L_SHO, R_SHO, CHEST_F})
                prev[j] = Vector3Subtract(pos[j], Vector3Scale(dir, 0.012f));
        }
    }
 
    void BodyFrame(Vector3& up, Vector3& fwd) const {
        up  = Vector3Normalize(Vector3Subtract(pos[NECK], pos[PELVIS]));
        fwd = Vector3Subtract(pos[CHEST_F], Vector3Lerp(pos[NECK], pos[PELVIS], 0.4f));
        fwd = Vector3Normalize(Vector3Subtract(fwd, Vector3Scale(up, Vector3DotProduct(fwd, up))));
    }
 
    // Decide where/which way to stand up, then start the animation
    void BeginGetUp() {
        Vector3 up, fwd;
        BodyFrame(up, fwd);
        origin = {pos[PELVIS].x, 0.0f, pos[PELVIS].z};
        if (adjustOrigin) adjustOrigin(origin);                       // not inside a pile of bricks
 
        float fl = std::sqrt(fwd.x * fwd.x + fwd.z * fwd.z);
        float fx, fz;
        if (fl > 0.5f) {                       // lying on a side / sitting: face where the chest faces
            fx = fwd.x / fl; fz = fwd.z / fl;
        } else {                               // lying on back/front: use the head direction
            float hx = up.x, hz = up.z;
            float hl = std::sqrt(hx * hx + hz * hz);
            if (hl < 1e-4f) { hx = fwd.x; hz = fwd.z; hl = std::max(std::sqrt(hx * hx + hz * hz), 1e-4f); }
            hx /= hl; hz /= hl;
            if (fwd.y > 0.0f) { fx = -hx; fz = -hz; }   // on back: sit up facing the feet
            else              { fx =  hx; fz =  hz; }   // on front: rise facing where the head points
        }
        yaw = std::atan2(fx, fz);
        mode = MODE_GETUP;
        getupT = 0.0f;
        groundTimer = 0.0f;
        grabbed = -1;
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
    }
 
    // Exhausted: he drops where he is (letting go of the mouse if held) and sleeps there.
    void PassOut() {
        if (mode != MODE_RAGDOLL) GoLimp(false);
        if (grabbed >= 0) Release();
        passedOut = true; asleep = true; bedTimer = 0.0f; groundTimer = 0.0f;
    }
 
    // Get out of bed: the get-into-bed animation in reverse (lie -> sit on the edge -> stand beside the bed)
    void WakeUp() {
        const bool onBed = OverBed(pos[PELVIS]) && pos[PELVIS].y > BED_H;
        asleep = false; bedTimer = 0.0f; grabbed = -1;
        if (passedOut && !onBed) { passedOut = false; BeginGetUp(); return; }   // collapsed on the floor: just get up
        passedOut = false;
        for (int i = 0; i < JOINT_COUNT; i++) wakeStart[i] = pos[i];
        mode = MODE_BEDANIM; bedReverse = true; wakeBlend = 0.0f;
        bedT = BED_SIT_T + BED_LIE_T;
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
 
    void FloorClamp() {
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (Pinned(i)) continue;
            pos[i].y = std::max(pos[i].y, Radius(i));
            PushOutOfBox(pos[i], Radius(i), BED);                // the bed is solid
            const float lim = ARENA - 0.07f * look.height;   // keep the body inside the walls
            pos[i].x = Clamp(pos[i].x, -lim, lim);
            pos[i].z = Clamp(pos[i].z, -lim, lim);
        }
    }
 
    // ------------------------------------------------------------------------
    // Held by the mouse: the hands that are free do something purposeful
    // ------------------------------------------------------------------------
    static bool IsLeg(int j) { return j == L_HIP || j == R_HIP || j == L_KNEE || j == R_KNEE || j == L_FOOT || j == R_FOOT; }
 
    // Spring a joint toward a goal (damped so it does not overshoot).
    void PullJoint(int j, Vector3 goal, float k) {
        pos[j]  = Vector3Add(pos[j], Vector3Scale(Vector3Subtract(goal, pos[j]), k));
        prev[j] = Vector3Lerp(prev[j], pos[j], 0.4f);
    }
 
    void ActiveHold(float dt) {
        activeClock += dt;
        const float H = look.height, scale = H / 1.75f;
        const float torsoLow = std::min(std::min(pos[PELVIS].y, pos[NECK].y), std::min(pos[L_SHO].y, pos[R_SHO].y));
        const bool onGround = torsoLow < 0.10f * H;
        const int hands[2] = {L_HAND, R_HAND}, shos[2] = {L_SHO, R_SHO}, elbs[2] = {L_ELB, R_ELB};
 
        if (!onGround) {
            // Hanging in the air: the free hands reach out for the target
            for (int k = 0; k < 2; k++) {
                if (hands[k] == grabbed) continue;
                const Vector3 sh = pos[shos[k]];
                const Vector3 d = Vector3Subtract(targetPos, sh);
                const float dist = Vector3Length(d);
                if (dist < 1e-4f) continue;
                const float arm = Bone(shos[k], elbs[k]) + Bone(elbs[k], hands[k]);
                const float grasp = 0.88f + 0.12f * std::sin(activeClock * 7.0f + k * PI);   // clutching at it
                const Vector3 goal = dist <= arm ? targetPos : Vector3Add(sh, Vector3Scale(d, arm * grasp / dist));
                PullJoint(hands[k], goal, 0.10f);
            }
        } else if (IsLeg(grabbed)) {
            // Dragged along the ground by a leg: crawl, the hands taking turns to reach ahead and pull
            const Vector3 chest = Vector3Lerp(pos[L_SHO], pos[R_SHO], 0.5f);
            Vector3 d = {targetPos.x - chest.x, 0.0f, targetPos.z - chest.z};
            const float dl = Vector3Length(d);
            if (dl < 0.05f) return;
            d = Vector3Scale(d, 1.0f / dl);
            const float stride = 0.32f * scale, ahead = 0.42f * scale, lift = 0.12f * scale, rad = Radius(L_HAND);
            for (int k = 0; k < 2; k++) {
                const float u = std::fmod(activeClock * 0.9f + 0.5f * k, 1.0f);       // the two hands are half a cycle apart
                const Vector3 sh = {pos[shos[k]].x, 0.0f, pos[shos[k]].z};
                const Vector3 front = Vector3Add(sh, Vector3Scale(d, ahead));
                const Vector3 back  = Vector3Add(sh, Vector3Scale(d, ahead - stride));
                Vector3 p; float y;
                if (u < 0.5f) { const float t = u * 2.0f;          p = Vector3Lerp(back, front, Smooth(t)); y = rad + lift * std::sin(PI * t); }   // swing forward, off the ground
                else          { const float t = (u - 0.5f) * 2.0f; p = Vector3Lerp(front, back, t);         y = rad; }                            // planted, sliding back
                PullJoint(hands[k], {p.x, y, p.z}, 0.25f);
            }
        }
    }
 
    void StepRagdoll(float dt) {
        const Vector3 g = {0.0f, -9.81f, 0.0f};
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (Pinned(i)) continue;
            Vector3 vel = Vector3Scale(Vector3Subtract(pos[i], prev[i]), 0.996f);
            prev[i] = pos[i];
            pos[i] = Vector3Add(Vector3Add(pos[i], vel), Vector3Scale(g, dt * dt));
        }
 
        if (grabbed >= 0 && !asleep && hasTarget) ActiveHold(dt);      // held by the mouse: hands crawl / reach
        if (painTimer > 0.0f && !asleep && grabbed < 0) {              // hurt: a hand clutches the sore spot
            const int hnd = painHand == 0 ? L_HAND : R_HAND;
            Vector3 spot = Vector3Add(Vector3Lerp(pos[painA], pos[painB], painSeg), Vector3Scale(painDir, painRadius));
            const float rub = painTimer * 2.0f * PI * 3.0f;
            spot.x += 0.02f * std::cos(rub); spot.z += 0.02f * std::sin(rub);
            PullJoint(hnd, spot, 0.12f);
        }
 
        // Collision detection: how fast is any joint about to hit the floor or a wall?
        impactCooldown -= dt;
        {
            const float lim = ARENA - 0.07f * look.height;
            float excess = 0.0f;                                   // speed above the "harmless" threshold
            for (int i = 0; i < JOINT_COUNT; i++) {
                if (Pinned(i)) continue;
                Vector3 v = Vector3Scale(Vector3Subtract(pos[i], prev[i]), 1.0f / dt);   // m/s
                if (pos[i].y < Radius(i) && v.y < 0.0f) excess = std::max(excess, -v.y - 3.0f);        // floor
                if (OverBed(pos[i]) && pos[i].y < BED_H + Radius(i) && pos[i].y > 0.5f * BED_H && v.y < 0.0f)
                    excess = std::max(excess, -v.y - 4.0f);                                             // bed (softer)
                if (pos[i].x >  lim && v.x > 0.0f)      excess = std::max(excess,  v.x - 1.5f);        // walls
                if (pos[i].x < -lim && v.x < 0.0f)      excess = std::max(excess, -v.x - 1.5f);
                if (pos[i].z >  lim && v.z > 0.0f)      excess = std::max(excess,  v.z - 1.5f);
                if (pos[i].z < -lim && v.z < 0.0f)      excess = std::max(excess, -v.z - 1.5f);
            }
            if (excess > 0.0f && impactCooldown <= 0.0f) {
                pendingMoodHit += std::min(3.0f + 2.5f * excess, 25.0f);   // harder hit = bigger mood drop
                impactCooldown = 0.3f;
            }
        }
 
        for (int it = 0; it < ITERS; it++) { SolveSticks(); FloorClamp(); }
        for (int i = 0; i < JOINT_COUNT; i++) {              // friction (once per step)
            if (Pinned(i)) continue;
            bool onFloor = pos[i].y <= Radius(i) + 1e-4f;
            bool onBed   = OverBed(pos[i]) && std::fabs(pos[i].y - (BED_H + Radius(i))) < 1e-3f;
            if (!onFloor && !onBed) continue;
            prev[i].x += (pos[i].x - prev[i].x) * 0.30f;
            prev[i].z += (pos[i].z - prev[i].z) * 0.30f;
            prev[i].y = pos[i].y;
        }
 
        // Bed: lying still on the mattress for a second puts the avatar to sleep
        if (!passedOut) {
            const bool over  = OverBed(pos[PELVIS]) && pos[PELVIS].y < BED_H + 0.25f;
            const float speed = Vector3Length(Vector3Subtract(pos[PELVIS], prev[PELVIS])) / dt;
            if (grabbed < 0 && over && speed < 0.4f) bedTimer += dt;
            else if (!over || grabbed >= 0) { bedTimer = 0.0f; asleep = false; }
            if (bedTimer >= 1.0f) asleep = true;
        }
 
        // Fall detection: pelvis near the floor and nobody holding the avatar
        {
            Vector3 bUp, bFwd; BodyFrame(bUp, bFwd);
            const float pelvisSpeed = Vector3Length(Vector3Subtract(pos[PELVIS], prev[PELVIS])) / dt;
            const bool lyingStill = std::fabs(bUp.y) < 0.6f && pelvisSpeed < 0.3f && !OverBed(pos[PELVIS]);   // e.g. draped over the bricks
            if (!passedOut && grabbed < 0 && (pos[PELVIS].y < 0.20f * look.height || lyingStill)) groundTimer += dt;
            else groundTimer = 0.0f;
        }
        if (groundTimer >= FALL_WAIT) BeginGetUp();
    }
 
    void StepGetUp(float dt) {
        getupT += dt;
        Vector3 target[JOINT_COUNT];
        ComputePose(GetUpKey(getupT), origin, yaw, target);
        for (int i = 0; i < JOINT_COUNT; i++) {              // pull joints toward the animation
            pos[i] = Vector3Add(pos[i], Vector3Scale(Vector3Subtract(target[i], pos[i]), GETUP_GAIN));
            prev[i] = pos[i];
        }
        for (int it = 0; it < ITERS; it++) { SolveSticks(); FloorClamp(); }
        if (getupT >= GETUP_TOTAL) Stand();                  // seamless: same pose as the last key
    }
 
    // ------------------------------------------------------------------------
    // Walking to the bed and getting in
    // ------------------------------------------------------------------------
    static Vector3 BedsideSpot()  { return {BED.min.x - 0.45f, 0.0f, BED_LIE_Z}; }                 // standing spot beside the bed
    static Vector3 CornerSpot()   { return {BED.min.x - 0.60f, 0.0f, BED.min.z - 0.60f}; }         // waypoint round the foot of the bed
 
    void StartWalkToBed() {
        mode = MODE_WALK; grabbed = -1;
        walkStage = 0; gaitPhase = 0.0f; gaitAmp = 0.0f; bedReverse = false;
        danceT = 0.0f; danceBlend = 0.0f;
    }
 
    void StepWalk(float dt) {
        if (walkStage == 0)   // pick a route that never cuts through the bed
            walkStage = (origin.x > BED.min.x - 0.2f && origin.z < BED.min.z - 0.1f) ? 1 : 2;
 
        const Vector3 goal = (walkStage == 1) ? CornerSpot() : BedsideSpot();
        float dx = goal.x - origin.x, dz = goal.z - origin.z;
        float dist = std::sqrt(dx * dx + dz * dz);
        bool  arrived = dist < 0.05f;
 
        float wantYaw = arrived ? PI * 0.5f : std::atan2(dx, dz);        // at the bedside: face the bed (+X)
        float dyaw = WrapPi(wantYaw - yaw);
        float maxTurn = TURN_RATE * dt;
        yaw += Clamp(dyaw, -maxTurn, maxTurn);
 
        bool moving = !arrived && std::fabs(dyaw) < 0.6f;                // turn on the spot first if facing the wrong way
        if (moving) {
            float step = std::min(WALK_SPEED * dt, dist);
            origin.x += std::sin(yaw) * step;
            origin.z += std::cos(yaw) * step;
        }
        if (arrived && walkStage == 1) { walkStage = 2; arrived = false; }
 
        ApplyWalkPose(dt, moving, arrived, arrived && walkStage == 2);   // keeps crying on the way, settles at the bedside
 
        if (arrived && walkStage == 2 && std::fabs(WrapPi(PI * 0.5f - yaw)) < 0.05f && gaitAmp < 0.05f && cryBlend < 0.01f) {
            origin = BedsideSpot(); yaw = PI * 0.5f;
            mode = MODE_BEDANIM; bedT = 0.0f; bedReverse = false;
        }
    }
 
    // Shared walking pose: stride while moving, a little shuffle while turning, fading out when done.
    // `settle` calms any crying so the hands are free for what comes next.
    void ApplyWalkPose(float dt, bool moving, bool arrived, bool settle) {
        float targetAmp = moving ? 1.0f : (arrived ? 0.0f : 0.5f);
        gaitAmp += Clamp(targetAmp - gaitAmp, -4.0f * dt, 4.0f * dt);
        gaitPhase += (moving ? 5.5f : 2.5f) * dt;
 
        PoseKey k = KEY_STAND;
        k.pelvisY = 0.515f - 0.022f * gaitAmp - 0.010f * gaitAmp * std::fabs(std::sin(gaitPhase));   // knees soften, body bobs
        k.lean = 4.0f * gaitAmp;
        UpdateEmotes(dt, false, settle);
        PoseTweak tw;
        ApplyEmotes(k, tw);                                  // same crying pose as when standing (no dancing on the move)
        ComputePose(k, origin, yaw, pos, gaitPhase, gaitAmp, &tw);
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
    }
 
    // ------------------------------------------------------------------------
    // Food: walk to the disk, crouch, pick it up, bring it to the mouth, eat
    // ------------------------------------------------------------------------
    void StartFetch(Vector3 food) {
        mode = MODE_FETCH; grabbed = -1;
        foodGoal = food; fetchStage = 0; gaitPhase = 0.0f; gaitAmp = 0.0f;
        foodHeld = false; foodEaten = false;
        danceT = 0.0f; danceBlend = 0.0f;
    }
 
    void StepFetch(float dt) {
        if (fetchStage == 0)   // go round the bed if it is in the way
            fetchStage = SegmentHitsBed(origin, foodGoal, 0.4f) ? 1 : 2;
 
        const bool toFood = (fetchStage == 2);
        const Vector3 goal = toFood ? foodGoal : CornerSpot();
        const float dx = goal.x - origin.x, dz = goal.z - origin.z;
        const float dist = std::sqrt(dx * dx + dz * dz);
        const float remaining = toFood ? dist - FETCH_STOP : dist - 0.05f;   // how much further to walk
        if (!toFood && remaining <= 0.003f) { fetchStage = 2; return; }     // reached the corner: now the food
 
        const bool arrived = remaining <= 0.003f;
        const float wantYaw = dist > 1e-4f ? std::atan2(dx, dz) : yaw;       // always face where he is going (the food, once there)
        const float dyaw = WrapPi(wantYaw - yaw);
        const float maxTurn = TURN_RATE * dt;
        yaw += Clamp(dyaw, -maxTurn, maxTurn);
 
        const bool moving = !arrived && std::fabs(dyaw) < 0.6f;
        if (moving) {
            float step = std::min(WALK_SPEED * dt, std::max(remaining, 0.0f));
            origin.x += std::sin(yaw) * step;
            origin.z += std::cos(yaw) * step;
        }
        ApplyWalkPose(dt, moving, arrived, arrived && toFood);
 
        if (toFood && arrived && std::fabs(dyaw) < 0.05f && gaitAmp < 0.05f && cryBlend < 0.01f) BeginEat();
    }
 
    void BeginEat() {
        const float H = look.height, cy = std::cos(yaw), sy = std::sin(yaw);
        const float dxw = foodGoal.x - origin.x, dzw = foodGoal.z - origin.z;
        eatFoodLocal = {(dxw * cy - dzw * sy) / H, 0.03f, (dxw * sy + dzw * cy) / H};   // food in his own frame
        eatT = 0.0f; foodHeld = false; foodEaten = false;
        mode = MODE_EAT;
    }
 
    void StepEat(float dt) {
        eatT += dt;
        const float H = look.height;
        const float tReach = 0.8f, tGrab = tReach + 0.25f, tLift = tGrab + 0.9f;
        const float tChew = tLift + 1.0f;                // one second at the mouth, then the food is gone
        const float tReturn = tChew + 0.5f;
 
        PoseKey k = KEY_STAND;
        PoseTweak tw;
        if (eatT < tReach) {                             // crouch down, right hand reaching for the disk
            float s = Smooth(eatT / tReach);
            k = KeyLerp(KEY_STAND, KEY_PICK, s);
            tw.rHandAt = eatFoodLocal; tw.rHandAtW = s;
        } else if (eatT < tGrab) {                       // grab it
            k = KEY_PICK;
            tw.rHandAt = eatFoodLocal; tw.rHandAtW = 1.0f;
        } else if (eatT < tLift) {                       // stand up, raise it to the mouth
            float s = Smooth((eatT - tGrab) / (tLift - tGrab));
            k = KeyLerp(KEY_PICK, KEY_MOUTH, s);
            tw.rHandAt = eatFoodLocal; tw.rHandAtW = 1.0f; tw.rToMouth = s; tw.rBendDown = s;
        } else if (eatT < tChew) {                       // eating
            k = KEY_MOUTH;
            k.lean += 1.5f * std::sin(2.0f * PI * 4.0f * (eatT - tLift));      // chewing
            tw.rHandAtW = 1.0f; tw.rToMouth = 1.0f; tw.rBendDown = 1.0f;
        } else {                                         // lower the hand again
            float s = Smooth((eatT - tChew) / (tReturn - tChew));
            k = KeyLerp(KEY_MOUTH, KEY_STAND, s);
            tw.rToMouth = 1.0f - s; tw.rBendDown = 1.0f - s;
        }
        ComputePose(k, origin, yaw, pos, 0.0f, 0.0f, &tw);
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
 
        // The disk: lies on the floor until grabbed, then rides in his hand, then is gone
        const float grabAt = tReach + 0.1f;
        if (!foodEaten && eatT >= grabAt) {
            const float rise = 0.037f * H * Smooth((eatT - grabAt) / 0.3f);    // lifts out from under the hand onto the palm
            foodPos = {pos[R_HAND].x, pos[R_HAND].y + rise, pos[R_HAND].z};
            foodHeld = true;
        }
        if (!foodEaten && eatT >= tChew) { foodEaten = true; foodHeld = false; }
        if (eatT >= tReturn) Stand();
    }
 
    // ------------------------------------------------------------------------
    // Watching TV: walk to a spot facing the screen, sit on the floor, watch, get up again
    // ------------------------------------------------------------------------
    void StartWatchTV(Vector3 spot) {
        mode = MODE_TV; grabbed = -1;
        tvSpot = spot; tvStage = TV_WALK; tvT = 0.0f; tvRiseRequested = false;
        walkStage = 0; gaitPhase = 0.0f; gaitAmp = 0.0f;
        danceT = 0.0f; danceBlend = 0.0f;
    }
 
    bool Watching() const { return mode == MODE_TV && tvStage == TV_WATCH; }
 
    static PoseKey SeatKey(float p) {                 // 0 = standing .. 1 = sitting on the floor
        return p < 0.5f ? KeyLerp(KEY_STAND, KEY_SIT_MID, Smooth(p * 2.0f))
                        : KeyLerp(KEY_SIT_MID, KEY_SIT, Smooth(p * 2.0f - 1.0f));
    }
 
    void StepTV(float dt) {
        if (tvStage == TV_WALK) {
            if (walkStage == 0)   // go round the bed if it is in the way
                walkStage = SegmentHitsBed(origin, tvSpot, 0.4f) ? 1 : 2;
            const bool toSpot = (walkStage == 2);
            const Vector3 goal = toSpot ? tvSpot : CornerSpot();
            const float dx = goal.x - origin.x, dz = goal.z - origin.z;
            const float dist = std::sqrt(dx * dx + dz * dz);
            const float remaining = toSpot ? dist : dist - 0.05f;
            if (!toSpot && remaining <= 0.003f) { walkStage = 2; return; }          // reached the corner: now the TV spot
 
            if (tvRiseRequested) {                                                  // asked to stop (sleepy, food...): stand still and finish
                ApplyWalkPose(dt, false, true, true);
                if (gaitAmp < 0.05f && cryBlend < 0.01f) { tvRiseRequested = false; Stand(); }
                return;
            }
            const bool arrived = remaining <= 0.003f;
            const float wantYaw = (toSpot && arrived) ? PI : (dist > 1e-4f ? std::atan2(dx, dz) : yaw);   // at the spot: face the screen (-Z)
            const float dyaw = WrapPi(wantYaw - yaw);
            const float maxTurn = TURN_RATE * dt;
            yaw += Clamp(dyaw, -maxTurn, maxTurn);
            const bool moving = !arrived && std::fabs(dyaw) < 0.6f;
            if (moving) {
                const float step = std::min(WALK_SPEED * dt, std::max(remaining, 0.0f));
                origin.x += std::sin(yaw) * step;
                origin.z += std::cos(yaw) * step;
            }
            ApplyWalkPose(dt, moving, arrived, arrived && toSpot);
            if (toSpot && arrived && std::fabs(WrapPi(PI - yaw)) < 0.05f && gaitAmp < 0.05f && cryBlend < 0.01f) {
                origin = tvSpot; yaw = PI;
                tvStage = TV_SIT; tvT = 0.0f;
            }
        } else if (tvStage == TV_SIT || tvStage == TV_RISE) {
            tvT += (tvStage == TV_SIT) ? dt : -dt;
            ComputePose(SeatKey(Clamp(tvT / TV_SIT_T, 0.0f, 1.0f)), origin, yaw, pos);
            for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
            if (tvStage == TV_SIT && tvT >= TV_SIT_T) { tvStage = TV_WATCH; tvT = 0.0f; }
            else if (tvStage == TV_RISE && tvT <= 0.0f) Stand();                    // back on his feet, facing the screen
        } else {                                                                    // TV_WATCH
            tvT += dt;
            PoseKey k = KEY_SIT;
            k.lean += 1.0f * std::sin(2.0f * PI * 0.25f * tvT);                      // gentle sway
            ComputePose(k, origin, yaw, pos);
            for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
            if (tvRiseRequested) { tvRiseRequested = false; tvStage = TV_RISE; tvT = TV_SIT_T; }
        }
    }
 
    struct BedKey { PoseKey k; float x, z, yaw; };
 
    void StepBedAnim(float dt) {
        if (!bedReverse) bedT += dt;
        else if (wakeBlend < 1.0f) wakeBlend = std::min(1.0f, wakeBlend + dt / 0.4f);   // first settle into the lying pose
        else bedT -= dt;
        const float H = look.height;
        const Vector3 sp = BedsideSpot();
        const float cx = (BED.min.x + BED.max.x) * 0.5f;
 
        const BedKey stand = {KEY_STAND, sp.x, sp.z, PI * 0.5f};
        // Sit on the edge facing out (-X), feet on the floor
        const BedKey sit = {{(BED_H + 0.09f) / H, 0.0f, 8.0f, 0.07f, 0.04f, 0.28f, 0.10f, (BED_H + 0.14f) / H, 0.08f},
                            BED.min.x + 0.20f, sp.z, PI * 1.5f};
        // Lie on the back, head to the +Z wall, feet toward the foot of the bed
        const float lieY = (BED_H + 0.06f) / H;
        const BedKey lie = {{lieY, 0.0f, -90.0f, 0.06f, lieY, 0.50f, 0.16f, lieY, -0.03f},
                            cx, BED_LIE_Z, PI};
 
        BedKey a, b; float s;
        if (bedT < BED_SIT_T)                { a = stand; b = sit; s = Smooth(bedT / BED_SIT_T); }
        else if (bedT < BED_SIT_T + BED_LIE_T) { a = sit;   b = lie; s = Smooth((bedT - BED_SIT_T) / BED_LIE_T); }
        else                                 { a = lie;   b = lie; s = 1.0f; }
 
        PoseKey k = KeyLerp(a.k, b.k, s);
        Vector3 org = {a.x + (b.x - a.x) * s, 0.0f, a.z + (b.z - a.z) * s};
        float   yw  = a.yaw + (b.yaw - a.yaw) * s;
        Vector3 target[JOINT_COUNT];
        ComputePose(k, org, yw, target);
        const float bl = bedReverse ? Smooth(wakeBlend) : 1.0f;
        for (int i = 0; i < JOINT_COUNT; i++) {
            pos[i] = bedReverse ? Vector3Lerp(wakeStart[i], target[i], bl) : target[i];
            prev[i] = pos[i];
        }
 
        if (!bedReverse && bedT >= BED_SIT_T + BED_LIE_T + 0.15f) GoLimp(false);   // settle onto the mattress; asleep after 1 s of lying still
        if (bedReverse && bedT <= 0.0f) { origin = BedsideSpot(); yaw = PI * 0.5f; Stand(); }
    }
 
    // ------------------------------------------------------------------------
    // Emotes: dance when happy, cry when miserable
    // ------------------------------------------------------------------------
    void UpdateEmotes(float dt, bool allowDance, bool calm = false) {
        emoteClock += dt;
        if (moodLevel < MOOD_CRY_BELOW) danceT = 0.0f;              // too sad to dance
        if (danceT > 0.0f) {
            danceT -= dt;
            if (danceT <= 0.0f) { danceT = 0.0f; nextDanceIn = (float)GetRandomValue(30, 100) / 10.0f; }   // next one in 3-10 s
        } else if (allowDance && moodLevel > MOOD_DANCE_ABOVE) {
            nextDanceIn -= dt;
            if (nextDanceIn <= 0.0f) { danceT = DANCE_TIME; danceClock = 0.0f; }
        }
        if (danceT > 0.0f || danceBlend > 0.0f) danceClock += dt;
 
        // Crying: continuous below MOOD_CRY_CONT; between that and MOOD_CRY_BELOW only a 1 s gesture,
        // once at a random moment in every 7 s window.
        const bool sad = moodLevel < MOOD_CRY_BELOW, weeping = moodLevel < MOOD_CRY_CONT;
        if (sad && !weeping && !calm) {
            cryWindowT += dt;
            if (!cryBurstDone && cryWindowT >= cryStartAt) { cryBurstT = CRY_BURST_TIME; cryBurstDone = true; }
            if (cryWindowT >= CRY_WINDOW) {                         // new window, new random moment (0..6 s in)
                cryWindowT -= CRY_WINDOW;
                cryStartAt = (float)GetRandomValue(0, 60) / 10.0f;
                cryBurstDone = false;
            }
        } else if (!sad) {                                          // cheered up: reset the schedule
            cryWindowT = 0.0f; cryBurstDone = false;
            cryStartAt = (float)GetRandomValue(0, 60) / 10.0f;
        }
        if (cryBurstT > 0.0f) cryBurstT = std::max(0.0f, cryBurstT - dt);
 
        const float dTarget = danceT > 0.0f ? 1.0f : 0.0f;
        const float cTarget = (!calm && ((sad && weeping) || (sad && cryBurstT > 0.0f))) ? 1.0f : 0.0f;
        danceBlend += Clamp(dTarget - danceBlend, -6.0f * dt, 6.0f * dt);
        const float cryRate = calm ? 2.5f : 5.0f;            // lower the hands gently when settling to eat / lie down
        cryBlend   += Clamp(cTarget - cryBlend,   -cryRate * dt, cryRate * dt);
    }
 
    // Layer the current emotes onto a key/tweak; returns an extra yaw (the dance shimmy).
    float ApplyEmotes(PoseKey& k, PoseTweak& tw) const {
        float yawOff = 0.0f;
        if (danceBlend > 0.0f) {
            const float t = danceClock, db = danceBlend;
            const float bounce = std::cos(4.0f * PI * t);            // 2 beats per second
            const float sway   = std::sin(2.0f * PI * t);
            const float pump   = std::sin(4.0f * PI * t);
            k.pelvisY -= db * (0.035f + 0.020f * bounce);            // knees bend on the beat
            tw.pelvisX += db * 0.035f * sway;                        // hips swing side to side
            yawOff     += db * 0.45f * std::cos(2.0f * PI * t);      // shoulders twist
            tw.lHand.x += 0.06f * db;  tw.lHand.y += db * (0.46f + 0.10f * pump);        // arms up, pumping in turn
            tw.rHand.x -= 0.06f * db;  tw.rHand.y += db * (0.46f - 0.10f * pump);
            tw.lFoot.y += db * 0.045f * std::max(0.0f,  sway);       // step from foot to foot
            tw.rFoot.y += db * 0.045f * std::max(0.0f, -sway);
        }
        if (cryBlend > 0.0f) {
            const float cb = cryBlend, sob = std::sin(2.0f * PI * 3.0f * emoteClock);
            k.lean    += cb * (8.0f + 1.5f * sob);                   // head bowed, shoulders shaking
            k.pelvisY -= cb * 0.012f;
            tw.coverFace = cb;
        }
        if (painBlend > 0.0f) {                                      // hunch over the sore spot: more the lower it is
            const Vector3 spotRest = Vector3Lerp(rest[painA], rest[painB], painSeg);
            const float low = Clamp((0.70f - spotRest.y / look.height) / 0.45f, 0.0f, 1.0f);   // 0 = head/shoulders .. 1 = knee/shin
            k.lean    += painBlend * (6.0f + 58.0f * low);          // (low spots: a deep crouch so the hand can reach)
            k.pelvisY -= painBlend * (0.01f + 0.29f * low);
            tw.painA = painA; tw.painB = painB; tw.painSeg = painSeg;
            tw.painDir = painDir; tw.painRadius = painRadius; tw.painHand = painHand;
            tw.painBlend = painBlend; tw.painClock = emoteClock;
        }
        return yawOff;
    }
 
    void StepStanding(float dt) {
        UpdateEmotes(dt, true);
        PoseKey k = KEY_STAND;
        PoseTweak tw;
        float yawOff = ApplyEmotes(k, tw);
        ComputePose(k, origin, yaw + yawOff, pos, 0.0f, 0.0f, &tw);
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
    }
 
    // Called when a brick hits him: lose mood, and put a hand to the spot as if it hurts.
    void RegisterHit(const Probe& pr, Vector3 towardBrick, float loss) {
        pendingMoodHit += loss;
        painA = pr.a; painB = pr.b; painSeg = pr.t;
        painDir = towardBrick; painRadius = pr.r;
        painTimer = PAIN_TIME;
        const Vector3 spotRest = Vector3Lerp(rest[pr.a], rest[pr.b], pr.t);
        painHand = (spotRest.x >= 0.0f) ? 0 : 1;                      // the hand on that side...
        if (pr.armSide >= 0) painHand = 1 - pr.armSide;               // ...or the other hand, if the arm itself was hit
        danceT = 0.0f;
    }
 
    // The probes along his body that bricks can hit (joints plus points along the bones).
    int BodyProbes(Probe out[]) const {
        const float H = look.height, b = look.build, sq = std::sqrt(b);
        int n = 0;
        auto add = [&](int a, int c, float t, float r, int arm) { out[n++] = {a, c, t, r, arm}; };
        add(HEAD, HEAD, 0.0f, 0.065f * H, -1);
        for (float t : {0.0f, 0.33f, 0.66f, 1.0f}) add(NECK, PELVIS, t, 0.07f * H * b, -1);     // torso
        add(L_SHO, L_SHO, 0.0f, 0.045f * H * sq, -1);  add(R_SHO, R_SHO, 0.0f, 0.045f * H * sq, -1);
        add(L_HIP, L_HIP, 0.0f, 0.050f * H * sq, -1);  add(R_HIP, R_HIP, 0.0f, 0.050f * H * sq, -1);
        const int sho[2] = {L_SHO, R_SHO}, elb[2] = {L_ELB, R_ELB}, han[2] = {L_HAND, R_HAND};
        const int hip[2] = {L_HIP, R_HIP}, kne[2] = {L_KNEE, R_KNEE}, foo[2] = {L_FOOT, R_FOOT};
        for (int k = 0; k < 2; k++) {
            add(sho[k], elb[k], 0.5f, 0.030f * H * sq, k);  add(elb[k], elb[k], 0.0f, 0.030f * H * sq, k);   // arm
            add(elb[k], han[k], 0.5f, 0.026f * H * sq, k);  add(han[k], han[k], 0.0f, 0.032f * H, k);
            add(hip[k], kne[k], 0.35f, 0.045f * H * sq, -1); add(hip[k], kne[k], 0.70f, 0.045f * H * sq, -1);   // leg
            add(kne[k], kne[k], 0.0f, 0.040f * H * sq, -1);
            add(kne[k], foo[k], 0.35f, 0.035f * H * sq, -1); add(kne[k], foo[k], 0.70f, 0.035f * H * sq, -1);
            add(foo[k], foo[k], 0.0f, 0.040f * H, -1);
        }
        return n;
    }
 
    void Step(float dt) {
        // pain timers (shared by every mode)
        if (painTimer > 0.0f) painTimer -= dt;
        brickHitCooldown -= dt;
        const bool onFeet = (mode == MODE_STANDING || mode == MODE_WALK || mode == MODE_FETCH);
        const float pTarget = (painTimer > 0.0f && onFeet) ? 1.0f : 0.0f;
        painBlend += Clamp(pTarget - painBlend, -(pTarget > painBlend ? 8.0f : 3.0f) * dt, (pTarget > painBlend ? 8.0f : 3.0f) * dt);
 
        if (mode == MODE_STANDING) StepStanding(dt);
        else if (mode == MODE_RAGDOLL) StepRagdoll(dt);
        else if (mode == MODE_GETUP) StepGetUp(dt);
        else if (mode == MODE_WALK) StepWalk(dt);
        else if (mode == MODE_BEDANIM) StepBedAnim(dt);
        else if (mode == MODE_FETCH) StepFetch(dt);
        else if (mode == MODE_EAT) StepEat(dt);
        else if (mode == MODE_TV) StepTV(dt);
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
        Vector3 up, fwd;
        BodyFrame(up, fwd);
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
// ----------------------------------------------------------------------------
// Needs / stat bars
// ----------------------------------------------------------------------------
static Color MixColor(Color a, Color b, float t) {
    t = Clamp(t, 0.0f, 1.0f);
    return {(unsigned char)(a.r + (b.r - a.r) * t), (unsigned char)(a.g + (b.g - a.g) * t),
            (unsigned char)(a.b + (b.b - a.b) * t), 255};
}
 
// red (0) -> yellow (50) -> green (100)
static Color MoodColor(float v) {
    const Color red = {220, 60, 50, 255}, yellow = {230, 200, 60, 255}, green = {70, 190, 80, 255};
    float t = Clamp(v / 100.0f, 0.0f, 1.0f);
    return t < 0.5f ? MixColor(red, yellow, t * 2.0f) : MixColor(yellow, green, (t - 0.5f) * 2.0f);
}
 
// light blue (full) -> dark blue (empty)
static Color SleepColor(float v) {
    const Color dark = {20, 50, 150, 255}, light = {150, 210, 255, 255};
    return MixColor(dark, light, v / 100.0f);
}
 
static const int STAT_LX = 135;              // width of the label column of the stat bars
 
static void DrawStatBar(int x, int y, const char* label, float value, Color fill) {
    const int w = 160, h = 18, lx = STAT_LX;
    DrawText(label, x, y, 18, DARKGRAY);
    DrawRectangle(x + lx, y, w, h, {50, 50, 50, 170});
    DrawRectangle(x + lx, y, (int)(w * Clamp(value, 0.0f, 100.0f) / 100.0f), h, fill);
    DrawRectangleLines(x + lx, y, w, h, {30, 30, 30, 255});
    DrawText(TextFormat("%.0f", value), x + lx + w + 8, y, 18, DARKGRAY);
}
 
static void DrawBed() {
    const float cx = (BED.min.x + BED.max.x) * 0.5f, cz = (BED.min.z + BED.max.z) * 0.5f;
    const Color frame = {120, 85, 60, 255}, frameEdge = {80, 55, 40, 255};
    const Color mattress = {236, 236, 244, 255}, blanket = {110, 140, 205, 255}, pillow = {255, 255, 255, 255};
 
    // frame (solid up to 0.30 m), mattress up to BED_H
    DrawCube({cx, 0.15f, cz}, BED_W, 0.30f, BED_L, frame);
    DrawCubeWires({cx, 0.15f, cz}, BED_W, 0.30f, BED_L, frameEdge);
    DrawCube({cx, 0.40f, cz}, BED_W - 0.08f, BED_H - 0.30f, BED_L - 0.08f, mattress);
    // blanket over the foot end, pillow + headboard at the wall end (+Z)
    const float bl = (BED_L - 0.08f) * 0.6f;
    DrawCube({cx, 0.40f, BED.min.z + 0.04f + bl * 0.5f}, BED_W - 0.04f, BED_H - 0.30f + 0.01f, bl, blanket);
    DrawCube({cx, BED_H + 0.05f, BED.max.z - 0.35f}, BED_W * 0.65f, 0.10f, 0.40f, pillow);
    DrawCube({cx, 0.45f, BED.max.z - 0.04f}, BED_W, 0.90f, 0.08f, frame);
    DrawCubeWires({cx, 0.45f, BED.max.z - 0.04f}, BED_W, 0.90f, 0.08f, frameEdge);
}
 
static void DrawArena() {
    const Color grass    = { 88, 156,  72, 255};
    const Color outer    = { 62, 118,  56, 255};
    const Color gridLine = { 25,  85,  35,  60};   // faint
    const Color wall     = {164, 132,  98, 255};
    const Color wallEdge = {110,  86,  62, 255};
 
    DrawPlane({0.0f, -0.03f, 0.0f}, {80.0f, 80.0f}, outer);
    DrawPlane({0.0f,  0.00f, 0.0f}, {2.0f * ARENA, 2.0f * ARENA}, grass);
    for (float g = -ARENA; g <= ARENA + 0.001f; g += 0.5f) {
        DrawLine3D({g, 0.01f, -ARENA}, {g, 0.01f, ARENA}, gridLine);
        DrawLine3D({-ARENA, 0.01f, g}, {ARENA, 0.01f, g}, gridLine);
    }
 
    const float span = 2.0f * ARENA + 2.0f * WALL_T;
    const float hy = WALL_H * 0.5f, off = ARENA + WALL_T * 0.5f;
    const Vector3 centre[4] = {{0.0f, hy,  off}, {0.0f, hy, -off}, { off, hy, 0.0f}, {-off, hy, 0.0f}};
    const Vector3 size[4]   = {{span, WALL_H, WALL_T}, {span, WALL_H, WALL_T},
                               {WALL_T, WALL_H, 2.0f * ARENA}, {WALL_T, WALL_H, 2.0f * ARENA}};
    for (int i = 0; i < 4; i++) {
        DrawCube(centre[i], size[i].x, size[i].y, size[i].z, wall);
        DrawCubeWires(centre[i], size[i].x, size[i].y, size[i].z, wallEdge);
    }
}
 
// ----------------------------------------------------------------------------
// Bricks: a pile in the corner opposite the bed. For now they can only be picked up (and thrown)
// with the mouse. They are axis-aligned boxes: they fall, stack, slide and settle, but do not tumble.
// ----------------------------------------------------------------------------
enum BrickState { BRICK_RESTING, BRICK_FALLING, BRICK_DRAGGED };
struct Brick { Vector3 pos; Vector3 vel; int state; Color color; };   // pos = centre
 
// About the size of his shoe, in the proportions 5 (wide) : 4 (high) : 8 (long)
static const float BRICK_UNIT = 0.035f;
static const float BRICK_W = 5.0f * BRICK_UNIT;    // 0.175 m  (x)
static const float BRICK_H = 4.0f * BRICK_UNIT;    // 0.140 m  (y)
static const float BRICK_L = 8.0f * BRICK_UNIT;    // 0.280 m  (z)
static const Vector3 BRICK_HALF = {BRICK_W * 0.5f, BRICK_H * 0.5f, BRICK_L * 0.5f};
 
// The pile: 5 bricks wide, 8 long, 4 high, in the -X/-Z corner (opposite the bed), long side along the wall
static const int   PILE_X = 5, PILE_Z = 8, PILE_Y = 4;
static const float PILE_GAP = 0.004f;
static const float PILE_X0 = -ARENA + 0.35f, PILE_Z0 = -ARENA + 0.35f;
static const float PILE_W = PILE_X * (BRICK_W + PILE_GAP), PILE_LEN = PILE_Z * (BRICK_L + PILE_GAP);
 
static std::vector<Brick> MakePile() {
    std::vector<Brick> v;
    for (int iy = 0; iy < PILE_Y; iy++)
        for (int ix = 0; ix < PILE_X; ix++)
            for (int iz = 0; iz < PILE_Z; iz++) {
                const int h = (ix * 7 + iy * 13 + iz * 5) % 5;                  // slight colour variation
                Brick b;
                b.pos = {PILE_X0 + (ix + 0.5f) * (BRICK_W + PILE_GAP), (iy + 0.5f) * BRICK_H, PILE_Z0 + (iz + 0.5f) * (BRICK_L + PILE_GAP)};
                b.vel = {0.0f, 0.0f, 0.0f};
                b.state = BRICK_RESTING;
                b.color = {(unsigned char)(172 + h * 4), (unsigned char)(80 + h * 3), (unsigned char)(60 + h * 2), 255};
                v.push_back(b);
            }
    return v;
}
 
static Box BrickBox(const Brick& b) {
    return {{b.pos.x - BRICK_HALF.x, b.pos.y - BRICK_HALF.y, b.pos.z - BRICK_HALF.z},
            {b.pos.x + BRICK_HALF.x, b.pos.y + BRICK_HALF.y, b.pos.z + BRICK_HALF.z}};
}
static BoundingBox BrickBounds(const Brick& b) { Box x = BrickBox(b); return {x.min, x.max}; }
 
// Push box (centre c, half-size h) out of box (centre bc, half-size bh) along the axis of least overlap.
// Returns 0 = no contact, 1 = pushed along x, 2 = pushed up, -2 = pushed down, 3 = pushed along z.
static int ResolveBoxBox(Vector3& c, Vector3 h, Vector3 bc, Vector3 bh) {
    const float dx = (h.x + bh.x) - std::fabs(c.x - bc.x);
    const float dy = (h.y + bh.y) - std::fabs(c.y - bc.y);
    const float dz = (h.z + bh.z) - std::fabs(c.z - bc.z);
    if (dx <= 0.0f || dy <= 0.0f || dz <= 0.0f) return 0;
    const float m = std::min(dx, std::min(dy, dz));
    if (m < 1e-5f) return 0;                                    // ignore hairline contact
    if (m == dy) { const bool up = c.y >= bc.y; c.y += up ? dy : -dy; return up ? 2 : -2; }
    if (m == dx) { c.x += (c.x >= bc.x) ? dx : -dx; return 1; }
    c.z += (c.z >= bc.z) ? dz : -dz; return 3;
}
 
static void StepBricks(std::vector<Brick>& bricks, float frameDt) {
    const Vector3 bedC = {(BED.min.x + BED.max.x) * 0.5f, (BED.min.y + BED.max.y) * 0.5f, (BED.min.z + BED.max.z) * 0.5f};
    const Vector3 bedH = {(BED.max.x - BED.min.x) * 0.5f, (BED.max.y - BED.min.y) * 0.5f, (BED.max.z - BED.min.z) * 0.5f};
    const float limX = ARENA - 0.05f - BRICK_HALF.x, limZ = ARENA - 0.05f - BRICK_HALF.z;
 
    // 1. A resting brick with nothing under it (its support was lifted away) starts to fall.
    for (Brick& b : bricks) {
        if (b.state != BRICK_RESTING) continue;
        const float bottom = b.pos.y - BRICK_HALF.y;
        bool supported = bottom < 0.003f;                                                   // the floor
        if (!supported && std::fabs(bottom - BED_H) < 0.003f &&                              // the mattress
            std::fabs(b.pos.x - bedC.x) < bedH.x + BRICK_HALF.x - 0.01f && std::fabs(b.pos.z - bedC.z) < bedH.z + BRICK_HALF.z - 0.01f)
            supported = true;
        for (size_t j = 0; j < bricks.size() && !supported; j++) {                          // another brick
            const Brick& o = bricks[j];
            if (&o == &b || o.state == BRICK_DRAGGED) continue;
            if (std::fabs(bottom - (o.pos.y + BRICK_HALF.y)) < 0.003f &&
                std::fabs(b.pos.x - o.pos.x) < BRICK_W - 0.01f && std::fabs(b.pos.z - o.pos.z) < BRICK_L - 0.01f)
                supported = true;
        }
        if (!supported) { b.state = BRICK_FALLING; b.vel = {0.0f, 0.0f, 0.0f}; }
    }
 
    // 2. Falling bricks (dropped, thrown, or whose support went): in small sub-steps so none tunnels through another
    const int SUB = 4;
    const float h = frameDt / SUB;
    for (int sub = 0; sub < SUB; sub++) {
        for (size_t i = 0; i < bricks.size(); i++) {
            Brick& b = bricks[i];
            if (b.state != BRICK_FALLING) continue;
            b.vel.y -= 9.81f * h;
            b.pos = Vector3Add(b.pos, Vector3Scale(b.vel, h));
            if (b.pos.x >  limX) { b.pos.x =  limX; b.vel.x *= -0.2f; }                     // walls
            if (b.pos.x < -limX) { b.pos.x = -limX; b.vel.x *= -0.2f; }
            if (b.pos.z >  limZ) { b.pos.z =  limZ; b.vel.z *= -0.2f; }
            if (b.pos.z < -limZ) { b.pos.z = -limZ; b.vel.z *= -0.2f; }
 
            bool landed = false;
            if (b.pos.y <= BRICK_HALF.y) { b.pos.y = BRICK_HALF.y; landed = true; }          // floor
            auto react = [&](int axis) {
                if (axis == 2) landed = true;
                else if (axis == 1) b.vel.x = 0.0f;
                else if (axis == 3) b.vel.z = 0.0f;
                else if (axis == -2) b.vel.y = std::min(b.vel.y, 0.0f);
            };
            react(ResolveBoxBox(b.pos, BRICK_HALF, bedC, bedH));                             // the bed
            for (size_t j = 0; j < bricks.size(); j++)                                       // other bricks
                if (j != i && bricks[j].state != BRICK_DRAGGED)
                    react(ResolveBoxBox(b.pos, BRICK_HALF, bricks[j].pos, BRICK_HALF));
 
            if (landed) {                                                                    // skid to a stop, then rest
                b.vel.y = 0.0f;
                const float fr = std::max(0.0f, 1.0f - 8.0f * h);
                b.vel.x *= fr; b.vel.z *= fr;
                if (std::sqrt(b.vel.x * b.vel.x + b.vel.z * b.vel.z) < 0.05f) { b.vel = {0.0f, 0.0f, 0.0f}; b.state = BRICK_RESTING; }
            }
        }
    }
}
 
// Somewhere he can stand: inside the arena, clear of the bed and of every brick (with room around him).
static bool SpotIsFree(const std::vector<Brick>& bricks, Vector3 p) {
    if (std::fabs(p.x) > ARENA - 0.5f || std::fabs(p.z) > ARENA - 0.5f) return false;
    if (p.x > BED.min.x - 0.4f && p.z > BED.min.z - 0.4f) return false;
    for (const Brick& b : bricks) {
        if (b.state == BRICK_DRAGGED) continue;
        if (std::fabs(p.x - b.pos.x) < BRICK_HALF.x + 0.45f && std::fabs(p.z - b.pos.z) < BRICK_HALF.z + 0.45f) return false;
    }
    return true;
}
static void ClearOfBricks(const std::vector<Brick>& bricks, Vector3& o) {
    if (SpotIsFree(bricks, o)) return;
    for (float r = 0.3f; r < 5.0f; r += 0.3f)                                  // spiral outwards to the nearest free spot
        for (int k = 0; k < 16; k++) {
            const float a = k * 2.0f * PI / 16.0f;
            const Vector3 c = {o.x + std::cos(a) * r, 0.0f, o.z + std::sin(a) * r};
            if (SpotIsFree(bricks, c)) { o = c; return; }
        }
}
 
// Is point e inside any brick other than `self`?
static bool InsideOtherBrick(const std::vector<Brick>& bricks, const Brick* self, Vector3 e) {
    for (const Brick& o : bricks) {
        if (&o == self || o.state == BRICK_DRAGGED) continue;
        if (std::fabs(e.x - o.pos.x) < BRICK_HALF.x && std::fabs(e.y - o.pos.y) < BRICK_HALF.y && std::fabs(e.z - o.pos.z) < BRICK_HALF.z) return true;
    }
    return false;
}
 
// A sphere centre p (radius r) is inside brick `self`: which way is out? Prefer a face the sphere is moving away from
// (rel = its velocity relative to the brick), and never one that leads into a neighbouring brick or the floor.
static void ExitThroughFace(const std::vector<Brick>& bricks, const Brick& self, Vector3 p, float r, Vector3 rel, Vector3& n, float& depth) {
    const Box bx = BrickBox(self);
    const float f[6] = {p.x - bx.min.x, bx.max.x - p.x, p.y - bx.min.y, bx.max.y - p.y, p.z - bx.min.z, bx.max.z - p.z};
    const Vector3 dirs[6] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
    bool open[6];
    for (int m = 0; m < 6; m++) {
        const Vector3 e = Vector3Add(p, Vector3Scale(dirs[m], f[m] + 0.5f * r));       // where the sphere would end up
        open[m] = e.y > 0.0f && !InsideOtherBrick(bricks, &self, e);
    }
    int k = -1;
    for (int m = 0; m < 6; m++)                                                          // 1st choice: open and against its motion
        if (open[m] && Vector3DotProduct(dirs[m], rel) <= 0.0f && (k < 0 || f[m] < f[k])) k = m;
    if (k < 0) for (int m = 0; m < 6; m++) if (open[m] && (k < 0 || f[m] < f[k])) k = m;           // 2nd: any open face
    if (k < 0) { k = 0; for (int m = 1; m < 6; m++) if (f[m] < f[k]) k = m; }                       // 3rd: simply the nearest
    n = dirs[k]; depth = f[k] + r;
}
 
// Bricks versus his body. Run once per frame after the bricks have moved.
//  - While he is posed by an animation (standing, walking, ...) he cannot be pushed: a falling brick bounces off him
//    (and slides off if it lands on top). While he is a ragdoll (or held) the brick pushes his body parts instead.
//  - A brick that hits fast enough hurts: mood is lost and he puts a hand to the spot (see Ragdoll::RegisterHit).
//  jointVel = how fast each of his joints moved this frame (m/s).
static void ResolveAvatarBricks(Ragdoll& av, std::vector<Brick>& bricks, const Vector3 jointVel[JOINT_COUNT], float frameDt) {
    // posed = driven by an animation that cannot be shoved (a ragdoll or a get-up can be pushed)
    const bool posed = (av.mode == MODE_STANDING || av.mode == MODE_WALK || av.mode == MODE_BEDANIM || av.mode == MODE_FETCH || av.mode == MODE_EAT || av.mode == MODE_TV);
    Probe probes[48];
    const int np = av.BodyProbes(probes);
 
    float bestClosing = 0.0f; int bestProbe = -1; Vector3 bestToward = {0, 0, 0};
    for (Brick& br : bricks) {
        if (Vector3Distance(br.pos, av.pos[PELVIS]) > 2.5f) continue;                  // far away: skip
        const Box bx = BrickBox(br);
        for (int i = 0; i < np; i++) {
            const Probe& pr = probes[i];
            const Vector3 p = Vector3Lerp(av.pos[pr.a], av.pos[pr.b], pr.t);
            const Vector3 q = {Clamp(p.x, bx.min.x, bx.max.x), Clamp(p.y, bx.min.y, bx.max.y), Clamp(p.z, bx.min.z, bx.max.z)};
            const Vector3 d = Vector3Subtract(p, q);
            const float dist = Vector3Length(d);
            if (dist >= pr.r) continue;                                                  // not touching
 
            const Vector3 vProbe = Vector3Lerp(jointVel[pr.a], jointVel[pr.b], pr.t);
            const Vector3 vBrick = br.state == BRICK_RESTING ? Vector3{0, 0, 0} : br.vel;
 
            Vector3 n; float depth;                                                      // n: from the brick toward the probe
            if (dist > 1e-5f) { n = Vector3Scale(d, 1.0f / dist); depth = pr.r - dist; }
            else ExitThroughFace(bricks, br, p, pr.r, Vector3Subtract(vProbe, vBrick), n, depth);   // centre inside the brick
 
            const float closing = Vector3DotProduct(Vector3Subtract(vBrick, vProbe), n);   // > 0: they are coming together
            if (posed && br.state == BRICK_RESTING) continue;                              // he walked into a resting brick: ignore
 
            if (closing > bestClosing) {
                bestClosing = closing; bestProbe = i; bestToward = Vector3Scale(n, -1.0f);   // toward the brick
            }
 
            const bool fromAbove = (n.y < -0.5f);                                           // the brick is on top of that part
            if (br.state == BRICK_DRAGGED) {
                if (posed) continue;                                                        // the mouse drags it through him
            }
            if (posed || (fromAbove && br.state == BRICK_FALLING)) {
                // he doesn't move: the brick is pushed out (and rebounds a little, or slides off)
                if (br.state == BRICK_FALLING) {
                    br.pos = Vector3Subtract(br.pos, Vector3Scale(n, depth));
                    if (closing > 0.0f) br.vel = Vector3Subtract(br.vel, Vector3Scale(n, closing * 1.3f));
                    if (fromAbove) {
                        br.vel.y = std::max(br.vel.y, 0.0f);
                        Vector3 away = {br.pos.x - av.pos[PELVIS].x, 0.0f, br.pos.z - av.pos[PELVIS].z};
                        const float al = Vector3Length(away);
                        away = al > 1e-4f ? Vector3Scale(away, 1.0f / al) : Vector3{1, 0, 0};
                        br.vel.x += away.x * 4.0f * frameDt;  br.vel.z += away.z * 4.0f * frameDt;   // slide off his head/shoulder
                    }
                }
            } else {
                // ragdoll: the brick pushes his body parts out of the way and gives them some of its momentum
                const float wa = (1.0f - pr.t), wb = pr.t;
                const float norm = (pr.a == pr.b) ? 1.0f : 1.0f / (wa * wa + wb * wb);
                const int ends[2] = {pr.a, pr.b};
                const float ws[2] = {(pr.a == pr.b) ? 1.0f : wa, (pr.a == pr.b) ? 0.0f : wb};
                for (int e = 0; e < 2; e++) {
                    const int j = ends[e];
                    if (ws[e] <= 0.0f || av.Pinned(j)) continue;
                    av.pos[j] = Vector3Add(av.pos[j], Vector3Scale(n, depth * ws[e] * norm));
                    if (closing > 0.0f && br.state != BRICK_RESTING)
                        av.prev[j] = Vector3Subtract(av.prev[j], Vector3Scale(n, closing * 0.5f * DT * ws[e]));   // shove
                }
                if (closing > 0.0f && br.state == BRICK_FALLING)
                    br.vel = Vector3Subtract(br.vel, Vector3Scale(n, closing * 0.3f));                          // the brick slows
            }
        }
    }
 
    // Clean-up (ragdoll only): a part pushed out of one brick can land inside its neighbour; sort that out against the pile.
    if (!posed) {
        for (int pass = 0; pass < 3; pass++) {
            bool any = false;
            for (Brick& br : bricks) {
                if (br.state != BRICK_RESTING || Vector3Distance(br.pos, av.pos[PELVIS]) > 2.5f) continue;
                const Box bx = BrickBox(br);
                for (int i = 0; i < np; i++) {
                    const Probe& pr = probes[i];
                    const Vector3 p = Vector3Lerp(av.pos[pr.a], av.pos[pr.b], pr.t);
                    const Vector3 q = {Clamp(p.x, bx.min.x, bx.max.x), Clamp(p.y, bx.min.y, bx.max.y), Clamp(p.z, bx.min.z, bx.max.z)};
                    const Vector3 d = Vector3Subtract(p, q);
                    const float dist = Vector3Length(d);
                    if (dist >= pr.r - 1e-4f) continue;
                    Vector3 n; float depth;
                    if (dist > 1e-5f) { n = Vector3Scale(d, 1.0f / dist); depth = pr.r - dist; }
                    else ExitThroughFace(bricks, br, p, pr.r, {0, 0, 0}, n, depth);
                    const float wa = (1.0f - pr.t), wb = pr.t;
                    const float norm = (pr.a == pr.b) ? 1.0f : 1.0f / (wa * wa + wb * wb);
                    const int ends[2] = {pr.a, pr.b};
                    const float ws[2] = {(pr.a == pr.b) ? 1.0f : wa, (pr.a == pr.b) ? 0.0f : wb};
                    for (int e = 0; e < 2; e++) {
                        if (ws[e] <= 0.0f || av.Pinned(ends[e])) continue;
                        av.pos[ends[e]] = Vector3Add(av.pos[ends[e]], Vector3Scale(n, depth * ws[e] * norm));
                        any = true;
                    }
                }
            }
            if (!any) break;
        }
    }
 
    // One hurtful hit at a time: the hardest contact this frame
    if (bestProbe >= 0 && bestClosing > BRICK_HURT_SPEED && av.brickHitCooldown <= 0.0f && !av.asleep) {
        const float loss = std::min(4.0f + 3.0f * (bestClosing - BRICK_HURT_SPEED), 30.0f);
        av.RegisterHit(probes[bestProbe], bestToward, loss);
        av.brickHitCooldown = 0.5f;
    } else if (bestProbe >= 0 && bestClosing > BRICK_HURT_SPEED && av.asleep && av.brickHitCooldown <= 0.0f) {
        av.pendingMoodHit += std::min(4.0f + 3.0f * (bestClosing - BRICK_HURT_SPEED), 30.0f);     // asleep: it still hurts, but no reaction
        av.brickHitCooldown = 0.5f;
    }
}
 
static void DrawBrick(const Brick& b) {
    Color c = b.color;
    if (b.state == BRICK_DRAGGED) c = {(unsigned char)std::min(255, c.r + 45), (unsigned char)std::min(255, c.g + 45), (unsigned char)std::min(255, c.b + 45), 255};
    DrawCube(b.pos, BRICK_W, BRICK_H, BRICK_L, c);
    DrawCubeWires(b.pos, BRICK_W, BRICK_H, BRICK_L, {95, 40, 30, 255});
}
 
// ----------------------------------------------------------------------------
// The screen: a TV on the wall at his eye level, as wide as he is tall. It plays a GIF when one is provided
// (screen.gif next to the program, or pass a path as the first argument). Until then it shows a test pattern.
// Clicking it switches it on / off.
// ----------------------------------------------------------------------------
static const char* SCREEN_GIF_PATH = "screen.gif";
static const float SCREEN_GIF_FPS  = 10.0f;     // raylib does not expose per-frame GIF delays, so GIFs play at this fixed rate
static const float SCREEN_PANEL_T  = 0.15f;     // thickness of the raised wall section the screen is mounted on (the arena walls are only 1 m tall)
 
struct ScreenMedia {
    Texture2D tex = {};
    Image gif = {};                              // every frame of the GIF, stacked (when one was loaded)
    std::vector<unsigned char> generated;        // test-pattern frames (when there is no GIF)
    int w = 320, h = 180, frames = 1, shown = -1;
    float fps = SCREEN_GIF_FPS;
    bool fromFile = false;
    const unsigned char* Frame(int i) const {
        const unsigned char* base = fromFile ? (const unsigned char*)gif.data : generated.data();
        return base + (size_t)i * (size_t)w * (size_t)h * 4;
    }
};
 
static ScreenMedia LoadScreenMedia(const char* path) {
    ScreenMedia m;
    int frames = 0;
    Image g = FileExists(path) ? LoadImageAnim(path, &frames) : Image{};
    if (g.data != nullptr && frames > 0 && g.width > 0 && g.height > 0) {
        m.gif = g; m.w = g.width; m.h = g.height; m.frames = frames; m.fromFile = true;
    } else {
        if (g.data != nullptr) UnloadImage(g);
        // No GIF yet: colour bars with a sweeping band and a bouncing block
        m.w = 320; m.h = 180; m.frames = 24; m.fps = 12.0f; m.fromFile = false;
        m.generated.resize((size_t)m.w * m.h * 4 * m.frames);
        const Color bars[7] = {{190, 190, 190, 255}, {190, 190, 40, 255}, {40, 190, 190, 255}, {40, 190, 40, 255},
                               {190, 40, 190, 255}, {190, 40, 40, 255}, {40, 40, 190, 255}};
        const int barH = m.h * 2 / 3;
        for (int f = 0; f < m.frames; f++) {
            Image im = GenImageColor(m.w, m.h, {16, 18, 26, 255});
            for (int i = 0; i < 7; i++) ImageDrawRectangle(&im, i * m.w / 7, 0, m.w / 7 + 1, barH, bars[i]);
            ImageDrawRectangle(&im, 0, f * barH / m.frames, m.w, 6, {240, 240, 240, 255});
            const int bx = (int)((0.5f - 0.5f * std::cos(2.0f * PI * f / m.frames)) * (m.w - 40));
            ImageDrawRectangle(&im, bx, barH + 14, 40, 28, WHITE);
            ImageDrawText(&im, "NO SIGNAL", 12, m.h - 38, 22, {220, 220, 220, 255});
            ImageDrawText(&im, "put screen.gif next to the program", 12, m.h - 16, 10, {160, 160, 170, 255});
            std::memcpy(m.generated.data() + (size_t)f * m.w * m.h * 4, im.data, (size_t)m.w * m.h * 4);
            UnloadImage(im);
        }
    }
    Image first = {(void*)m.Frame(0), m.w, m.h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};   // (shares the memory; not unloaded)
    m.tex = LoadTextureFromImage(first);
    SetTextureFilter(m.tex, TEXTURE_FILTER_BILINEAR);
    m.shown = 0;
    return m;
}
 
static void UnloadScreenMedia(ScreenMedia& m) {
    UnloadTexture(m.tex);
    if (m.fromFile) UnloadImage(m.gif);
}
 
static void UpdateScreenFrame(ScreenMedia& m, float clock) {            // show the right GIF frame for this time
    const int fi = m.frames > 0 ? (int)(clock * m.fps) % m.frames : 0;
    if (fi != m.shown) { UpdateTexture(m.tex, m.Frame(fi)); m.shown = fi; }
}
 
// Where the screen is: centred on the -Z wall (facing into the room), centred on his eye level.
struct ScreenGeom { float cx, cy, z, w, h, panelW, panelH; };
static ScreenGeom ScreenGeometry(float H, float aspect) {            // aspect = picture height / width
    ScreenGeom g;
    g.w = H;                                   // as wide as he is tall
    g.h = g.w * aspect;
    if (g.h > 2.3f) { g.h = 2.3f; g.w = g.h / aspect; }               // (a very tall GIF: keep it within the room)
    g.cx = 0.0f;
    g.cy = 0.94f * H;                          // his eyes
    g.z  = -ARENA + SCREEN_PANEL_T + 0.06f + 0.002f;                  // the picture plane, just in front of the bezel
    g.panelW = g.w + 0.5f;
    g.panelH = g.cy + g.h * 0.5f + 0.25f;
    return g;
}
 
// Distance along the ray to the screen (bezel included), or -1 if it misses or comes from behind the wall.
static float ScreenHitDistance(const Ray& ray, const ScreenGeom& g) {
    if (ray.direction.z >= -1e-6f) return -1.0f;
    const float t = (g.z - ray.position.z) / ray.direction.z;
    if (t < 0.0f) return -1.0f;
    const float x = ray.position.x + ray.direction.x * t, y = ray.position.y + ray.direction.y * t;
    const float m = 0.06f;
    if (std::fabs(x - g.cx) > g.w * 0.5f + m || std::fabs(y - g.cy) > g.h * 0.5f + m) return -1.0f;
    return t;
}
 
// The part of the raised wall panel just below the screen: clicking it sends him to watch TV.
static float TvBaseHitDistance(const Ray& ray, const ScreenGeom& g) {
    if (ray.direction.z >= -1e-6f) return -1.0f;
    const float zFront = -ARENA + SCREEN_PANEL_T;
    const float t = (zFront - ray.position.z) / ray.direction.z;
    if (t < 0.0f) return -1.0f;
    const float x = ray.position.x + ray.direction.x * t, y = ray.position.y + ray.direction.y * t;
    const float top = g.cy - g.h * 0.5f - 0.06f;                    // just under the bezel
    if (std::fabs(x - g.cx) > g.panelW * 0.5f || y < 0.0f || y > top) return -1.0f;
    return t;
}
 
static void DrawTexturedQuad(Texture2D tex, const ScreenGeom& g) {
    const float x0 = g.cx - g.w * 0.5f, x1 = g.cx + g.w * 0.5f, y0 = g.cy - g.h * 0.5f, y1 = g.cy + g.h * 0.5f;
    rlSetTexture(tex.id);
    rlBegin(RL_QUADS);
        rlColor4ub(255, 255, 255, 255);
        rlNormal3f(0.0f, 0.0f, 1.0f);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x0, y0, g.z);      // bottom left (v = 1 is the bottom row of the picture)
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x1, y0, g.z);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x1, y1, g.z);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x0, y1, g.z);
    rlEnd();
    rlSetTexture(0);
}
 
static void DrawScreenOnWall(const ScreenMedia& m, const ScreenGeom& g, bool on, bool hover, bool baseHover) {
    // the raised section of wall it hangs on
    const Vector3 panelC = {g.cx, g.panelH * 0.5f, -ARENA + SCREEN_PANEL_T * 0.5f};
    DrawCube(panelC, g.panelW, g.panelH, SCREEN_PANEL_T, {206, 200, 188, 255});
    DrawCubeWires(panelC, g.panelW, g.panelH, SCREEN_PANEL_T, {150, 144, 132, 255});
    // bezel, picture (or a dark glass when off), power light
    const Vector3 bezelC = {g.cx, g.cy, -ARENA + SCREEN_PANEL_T + 0.03f};
    DrawCube(bezelC, g.w + 0.12f, g.h + 0.12f, 0.06f, {24, 24, 28, 255});
    if (on) DrawTexturedQuad(m.tex, g);
    else    DrawCube({g.cx, g.cy, g.z - 0.002f}, g.w, g.h, 0.004f, {12, 14, 20, 255});
    DrawCube({g.cx + g.w * 0.5f - 0.04f, g.cy - g.h * 0.5f - 0.03f, g.z}, 0.03f, 0.015f, 0.01f,
             on ? Color{70, 230, 100, 255} : Color{210, 50, 50, 255});
    if (hover) DrawCubeWires(bezelC, g.w + 0.12f, g.h + 0.12f, 0.06f, WHITE);
    if (baseHover) {                                              // the click zone under the screen lights up
        const float top = g.cy - g.h * 0.5f - 0.06f;
        const Vector3 c = {g.cx, top * 0.5f, -ARENA + SCREEN_PANEL_T + 0.003f};
        DrawCube(c, g.panelW - 0.1f, top - 0.05f, 0.004f, {255, 255, 255, 45});
        DrawCubeWires(c, g.panelW - 0.1f, top - 0.05f, 0.004f, WHITE);
    }
}
 
static void DrawFood(const Food& f, float H) {
    const float r = 0.03f * H, th = 0.012f * H;            // a disk the size of his hand
    const Vector3 base = {f.pos.x, f.pos.y - th * 0.5f, f.pos.z};
    DrawCylinder(base, r, r, th, 28, {255, 140, 185, 255});
    DrawCylinderWires(base, r, r, th, 28, {190, 70, 120, 255});
}
 
int main(int argc, char** argv) {
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
 
    // The screen on the wall (a GIF can be given as the first argument; default screen.gif)
    ScreenMedia screen = LoadScreenMedia(argc > 1 ? argv[1] : SCREEN_GIF_PATH);
    bool  screenOn = true;
    float screenClock = 0.0f;               // runs only while the screen is on, so the GIF resumes where it left off
 
    float yaw = 0.6f, pitch = 0.25f, dist = 4.0f;
    float grabDist = 0.0f;
    bool  draggingCam = false;              // left-drag that started on empty space
 
    // Needs (0..100). Health and hunger are placeholders for now; only mood is live.
    float health = 100.0f, hunger = 100.0f, mood = 100.0f;
    float entertainment = 100.0f;           // drains slowly; refills while he watches the TV with the screen on
    float entTick = 0.0f;                   // counts up to ENT_INTERVAL, then entertainment drops
    bool  watchRequested = false;           // the click under the TV is waiting for him to be free
    float tvOffTimer = 0.0f;                // seconds he has been staring at a switched-off screen
    float moodTick = 0.0f;                  // counts up to MOOD_REGEN_INTERVAL, then mood recovers by 1 point
    float sleepiness = 100.0f;              // 100 = rested (light blue), 0 = exhausted (dark blue)
    float sleepTick = 0.0f;                 // counts up to SLEEP_INTERVAL, then sleepiness drops
    Vector3 focus = {0.0f, 0.0f, 0.0f};     // camera follows the avatar on the ground plane
    Vector3 lastAvatarPos[JOINT_COUNT];     // for estimating how fast his joints move
    for (int i = 0; i < JOINT_COUNT; i++) lastAvatarPos[i] = avatar.pos[i];
 
    // Food: disks dropped with the button. He eats them one at a time (oldest first).
    std::vector<Food> foods;
    int targetFood = -1;                    // index of the disk he is fetching / eating, or -1
    int grabbedFood = -1;                   // index of the disk the mouse is holding, or -1
    std::vector<Brick> bricks = MakePile(); // the brick pile (opposite corner from the bed)
    int grabbedBrick = -1;                  // index of the brick the mouse is holding, or -1
    avatar.adjustOrigin = [&](Vector3& o) { ClearOfBricks(bricks, o); };   // he never gets up inside the pile
    float brickGrabDist = 0.0f;
    float foodGrabDist = 0.0f;
    float zeroSleepTimer = 0.0f;            // seconds spent at zero sleepiness
    auto SpawnFood = [&]() {                // drops from high above a random spot on the grass (not near the bed)
        const float L = ARENA - 1.5f;
        Vector3 p = {0.0f, 10.0f, 0.0f};
        for (int tries = 0; tries < 50; tries++) {
            p.x = (float)GetRandomValue((int)(-L * 100), (int)(L * 100)) / 100.0f;
            p.z = (float)GetRandomValue((int)(-L * 100), (int)(L * 100)) / 100.0f;
            const bool nearBed  = p.x > BED.min.x - 1.0f && p.z > BED.min.z - 1.0f;
            const bool nearPile = p.x < PILE_X0 + PILE_W + 1.0f && p.z < PILE_Z0 + PILE_LEN + 1.0f;
            if (!nearBed && !nearPile) break;
        }
        foods.push_back({p, {0.0f, 0.0f, 0.0f}, FOOD_FALLING});
    };
 
    while (!WindowShouldClose()) {
        // ---- "Drop food" button (top of the screen, left of the bars) ----
        const int px = GetScreenWidth() - 345;
        const Rectangle dropBtn = {(float)(px - 145), 12.0f, 128.0f, 32.0f};
        const bool overBtn = CheckCollisionPointRec(GetMousePosition(), dropBtn);
        const bool uiClick = overBtn && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
        if (uiClick) SpawnFood();
 
        // ---- camera: drag on empty space (left) or hold right mouse to turn ----
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || (draggingCam && IsMouseButtonDown(MOUSE_BUTTON_LEFT))) {
            Vector2 d = GetMouseDelta();
            yaw   -= d.x * 0.005f;
            pitch += d.y * 0.005f;
            pitch = Clamp(pitch, -0.1f, 1.45f);
        }
        dist = Clamp(dist - GetMouseWheelMove() * 0.3f, 1.5f, 14.0f);
 
        focus.x = Lerp(focus.x, avatar.pos[PELVIS].x, 0.05f);
        focus.z = Lerp(focus.z, avatar.pos[PELVIS].z, 0.05f);
 
        Camera3D cam = {};
        Vector3 target = {focus.x, 0.55f * avatar.look.height, focus.z};
        cam.target = target;
        cam.position = {target.x + dist * std::cos(pitch) * std::sin(yaw),
                        target.y + dist * std::sin(pitch),
                        target.z + dist * std::cos(pitch) * std::cos(yaw)};
        cam.up = {0.0f, 1.0f, 0.0f};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;
 
        // ---- template editing (rebuilds the skeleton, keeps position) ----
        bool rebuild = false;
        if (HEIGHT_KEYS && IsKeyPressed(KEY_UP))   { look.height = Clamp(look.height + 0.05f, 1.20f, 2.10f); rebuild = true; }   // switched off for now
        if (HEIGHT_KEYS && IsKeyPressed(KEY_DOWN)) { look.height = Clamp(look.height - 0.05f, 1.20f, 2.10f); rebuild = true; }
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
        if (IsKeyPressed(KEY_R)) avatar.HardReset();
        if (IsKeyPressed(KEY_SPACE) && (avatar.mode == MODE_STANDING || avatar.mode == MODE_WALK || avatar.mode == MODE_FETCH || avatar.mode == MODE_EAT || avatar.mode == MODE_TV)) avatar.GoLimp(true);   // knock over
        if (IsKeyPressed(KEY_B)) mood = 5.0f;                                      // test shortcuts: miserable (continuous crying) ...
        if (IsKeyPressed(KEY_M)) mood = 15.0f;                                     // ... sad (occasional 1 s cry) ...
        if (IsKeyPressed(KEY_N)) mood = 100.0f;                                    // ... and happy (dances)
        if (IsKeyPressed(KEY_X)) sleepiness = 0.0f;                                // test shortcut: exhausted (passes out after 10 s)
        if (IsKeyPressed(KEY_Z)) sleepiness = std::min(sleepiness, SLEEPY_AT);     // test shortcut: make him sleepy now
 
        // ---- the screen on the wall: play it, and check whether the mouse is over it ----
        const ScreenGeom sg = ScreenGeometry(avatar.look.height, (float)screen.h / (float)screen.w);
        if (screenOn) { screenClock += GetFrameTime(); UpdateScreenFrame(screen, screenClock); }
        const Ray mouseRay = ScreenRay(GetMousePosition(), cam);
        const float screenDist = ScreenHitDistance(mouseRay, sg);
        const bool screenHover = screenDist >= 0.0f && !overBtn;
        const float tvBaseDist = TvBaseHitDistance(mouseRay, sg);          // the wall under the screen
        const bool tvBaseHover = tvBaseDist >= 0.0f && !overBtn && !screenHover;
 
        // ---- mouse grab ----
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !uiClick) {
            Ray ray = ScreenRay(GetMousePosition(), cam);
            float best = 1e9f; int hit = -1;
            for (int i = 0; i < JOINT_COUNT; i++) {
                RayCollision c = GetRayCollisionSphere(ray, avatar.pos[i], 0.07f * avatar.look.height);
                if (c.hit && c.distance < best) { best = c.distance; hit = i; }
            }
            int hitFood = -1; float bestFood = 1e9f;
            for (size_t i = 0; i < foods.size(); i++) {
                if (foods[i].state == FOOD_TARGETED) continue;           // he is already fetching / eating this one
                RayCollision c = GetRayCollisionSphere(ray, foods[i].pos, 0.09f);
                if (c.hit && c.distance < bestFood) { bestFood = c.distance; hitFood = (int)i; }
            }
            int hitBrick = -1; float bestBrick = 1e9f;
            for (size_t i = 0; i < bricks.size(); i++) {
                if (bricks[i].state == BRICK_DRAGGED) continue;
                RayCollision c = GetRayCollisionBox(ray, BrickBounds(bricks[i]));
                if (c.hit && c.distance < bestBrick) { bestBrick = c.distance; hitBrick = (int)i; }
            }
            const bool screenFirst = screenDist >= 0.0f && screenDist < best && screenDist < bestFood && screenDist < bestBrick;
            if (screenFirst) {
                screenOn = !screenOn;                                    // click the screen: on / off
            } else if (tvBaseDist >= 0.0f && tvBaseDist < best && tvBaseDist < bestFood && tvBaseDist < bestBrick) {
                if (!avatar.asleep && avatar.mode != MODE_TV) watchRequested = true;    // click under the TV: go and watch it
            } else if (hitBrick >= 0 && bestBrick < best && bestBrick < bestFood) {   // a brick is the nearest thing under the cursor
                grabbedBrick = hitBrick; brickGrabDist = bestBrick;
                bricks[hitBrick].state = BRICK_DRAGGED; bricks[hitBrick].vel = {0.0f, 0.0f, 0.0f};
            } else if (hitFood >= 0 && bestFood < best) {                // the disk is closer than the avatar: pick it up
                grabbedFood = hitFood; foodGrabDist = bestFood;
                foods[hitFood].state = FOOD_DRAGGED; foods[hitFood].vel = {0.0f, 0.0f, 0.0f};
            } else if (hit >= 0) {
                if (avatar.mode != MODE_RAGDOLL) avatar.GoLimp(false);   // grabbing cancels a get-up too
                avatar.grabbed = hit;
                grabDist = best;
            } else {
                draggingCam = true;                                      // missed the avatar: turn the camera
            }
        }
        if (avatar.grabbed >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = ScreenRay(GetMousePosition(), cam);
            Vector3 t = Vector3Add(ray.position, Vector3Scale(ray.direction, grabDist));
            const float lim = ARENA - 0.07f * avatar.look.height;       // held joint can't go through walls
            t.x = Clamp(t.x, -lim, lim);
            t.z = Clamp(t.z, -lim, lim);
            t.y = std::max(t.y, 0.03f);
            PushOutOfBox(t, 0.05f, BED);                                // ...or through the bed
            avatar.prev[avatar.grabbed] = avatar.pos[avatar.grabbed];
            avatar.pos[avatar.grabbed]  = t;
        }
        if (grabbedFood >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = ScreenRay(GetMousePosition(), cam);
            Vector3 t = Vector3Add(ray.position, Vector3Scale(ray.direction, foodGrabDist));
            const float lim = ARENA - 0.1f, half = 0.006f * avatar.look.height;
            t.x = Clamp(t.x, -lim, lim); t.z = Clamp(t.z, -lim, lim);
            t.y = std::max(t.y, half + 0.002f);
            PushOutOfBox(t, half, BED);
            Food& f = foods[grabbedFood];
            const float fdt = std::max(GetFrameTime(), 1e-4f);
            f.vel = Vector3Lerp(f.vel, Vector3Scale(Vector3Subtract(t, f.pos), 1.0f / fdt), 0.5f);   // smoothed, for throwing
            f.pos = t;
        }
        if (grabbedBrick >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = ScreenRay(GetMousePosition(), cam);
            Vector3 t = Vector3Add(ray.position, Vector3Scale(ray.direction, brickGrabDist));
            t.x = Clamp(t.x, -(ARENA - 0.05f - BRICK_HALF.x), ARENA - 0.05f - BRICK_HALF.x);
            t.z = Clamp(t.z, -(ARENA - 0.05f - BRICK_HALF.z), ARENA - 0.05f - BRICK_HALF.z);
            t.y = std::max(t.y, BRICK_HALF.y);
            ResolveBoxBox(t, BRICK_HALF, {(BED.min.x + BED.max.x) * 0.5f, BED_H * 0.5f, (BED.min.z + BED.max.z) * 0.5f},
                          {(BED.max.x - BED.min.x) * 0.5f, BED_H * 0.5f, (BED.max.z - BED.min.z) * 0.5f});   // not through the bed
            Brick& b = bricks[grabbedBrick];
            const float bdt = std::max(GetFrameTime(), 1e-4f);
            b.vel = Vector3Lerp(b.vel, Vector3Scale(Vector3Subtract(t, b.pos), 1.0f / bdt), 0.5f);     // smoothed, for throwing
            b.pos = t;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if (grabbedBrick >= 0) {                                     // let go: the brick drops (or flies, if thrown)
                Brick& b = bricks[grabbedBrick];
                const float sp = Vector3Length(b.vel);
                if (sp > 8.0f) b.vel = Vector3Scale(b.vel, 8.0f / sp);
                b.state = BRICK_FALLING;
                grabbedBrick = -1;
            }
            avatar.Release(); draggingCam = false;
            if (grabbedFood >= 0) {                                      // let go: the disk drops (or flies, if thrown)
                Food& f = foods[grabbedFood];
                const float sp = Vector3Length(f.vel);
                if (sp > 12.0f) f.vel = Vector3Scale(f.vel, 12.0f / sp);
                f.state = FOOD_FALLING;
                grabbedFood = -1;
            }
        }
 
        avatar.moodLevel = mood;
        avatar.hasTarget = false;                                       // what his free hands go for while he is held:
        {                                                               // the nearest disk, else the bed when he is sleepy
            float bestD = 1e9f;
            for (const Food& f : foods) {
                if (f.state == FOOD_TARGETED) continue;
                float d = Vector3Distance(f.pos, avatar.pos[PELVIS]);
                if (d < bestD) { bestD = d; avatar.targetPos = f.pos; avatar.hasTarget = true; }
            }
            if (!avatar.hasTarget && sleepiness <= SLEEPY_AT) {
                avatar.targetPos = {(BED.min.x + BED.max.x) * 0.5f, BED_H, BED_LIE_Z};
                avatar.hasTarget = true;
            }
        }
        for (int i = 0; i < STEPS; i++) avatar.Step(DT);
 
        // ---- mood: drops on collisions, recovers +1 every second ----
        mood -= avatar.pendingMoodHit;
        avatar.pendingMoodHit = 0.0f;
        moodTick += GetFrameTime();
        while (moodTick >= MOOD_REGEN_INTERVAL) { moodTick -= MOOD_REGEN_INTERVAL; mood += 1.0f; }
        if (avatar.grabbed >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT))        // being held isn't fun
            mood -= HELD_MOOD_DRAIN * GetFrameTime();
        mood = Clamp(mood, 0.0f, 100.0f);
 
        // ---- sleepiness: drops every SLEEP_INTERVAL s, refills while asleep in the bed ----
        if (avatar.asleep) {
            sleepiness += SLEEP_REGEN * GetFrameTime();
            if (sleepiness >= 100.0f) { sleepiness = 100.0f; avatar.WakeUp(); }
        } else {
            sleepTick += GetFrameTime();
            while (sleepTick >= SLEEP_INTERVAL) { sleepTick -= SLEEP_INTERVAL; sleepiness -= SLEEP_DROP; }
        }
        sleepiness = Clamp(sleepiness, 0.0f, 100.0f);
 
        // ---- entertainment: drains slowly; refills only while he sits watching with the screen ON ----
        const bool watchingTV = avatar.Watching();
        if (watchingTV && screenOn) entertainment += ENT_REGEN * GetFrameTime();
        else {
            entTick += GetFrameTime();
            while (entTick >= ENT_INTERVAL) { entTick -= ENT_INTERVAL; entertainment -= ENT_DROP; }
        }
        entertainment = Clamp(entertainment, 0.0f, 100.0f);
        tvOffTimer = (watchingTV && !screenOn) ? tvOffTimer + GetFrameTime() : 0.0f;
        if (entertainment >= 100.0f) watchRequested = false;           // not in the mood for TV
 
        // He gets up from the TV when he has had enough, is sleepy, there is food on the floor, or the screen stays off.
        if (avatar.mode == MODE_TV) {
            bool foodWaiting = false;
            for (const Food& f : foods) if (f.state == FOOD_LANDED && f.pos.y <= 0.05f) foodWaiting = true;
            if (entertainment >= 100.0f || sleepiness <= SLEEPY_AT || foodWaiting || tvOffTimer >= TV_OFF_PATIENCE) avatar.tvRiseRequested = true;
        }
        // ---- exhaustion: 10 s at zero sleepiness and he drops where he is and falls asleep ----
        if (sleepiness <= 0.0f && !avatar.asleep) zeroSleepTimer += GetFrameTime(); else zeroSleepTimer = 0.0f;
        if (zeroSleepTimer >= PASS_OUT_AFTER) { avatar.PassOut(); zeroSleepTimer = 0.0f; }
 
        // ---- food: falling, landing, and the disk he is eating ----
        {
            const float H = avatar.look.height, half = 0.006f * H;
            const float dt = GetFrameTime();
            const float wallLim = ARENA - 0.1f;
            for (Food& f : foods) {
                if (f.state != FOOD_FALLING) continue;                  // (dropped, thrown or just released)
                f.vel.y -= 9.81f * dt;
                f.pos = Vector3Add(f.pos, Vector3Scale(f.vel, dt));
                if (f.pos.x >  wallLim) { f.pos.x =  wallLim; f.vel.x *= -0.3f; }   // walls
                if (f.pos.x < -wallLim) { f.pos.x = -wallLim; f.vel.x *= -0.3f; }
                if (f.pos.z >  wallLim) { f.pos.z =  wallLim; f.vel.z *= -0.3f; }
                if (f.pos.z < -wallLim) { f.pos.z = -wallLim; f.vel.z *= -0.3f; }
                PushOutOfBox(f.pos, half, BED);                         // the bed is solid; a disk can land on it
                bool onBrick = false;
                for (const Brick& b : bricks) {                         // so are the bricks
                    if (b.state == BRICK_DRAGGED) continue;
                    const float before = f.pos.y;
                    PushOutOfBox(f.pos, half, BrickBox(b));
                    if (f.pos.y > before + 1e-6f) onBrick = true;
                }
                const bool onFloor = f.pos.y <= half + 0.002f;
                if (onFloor) f.pos.y = half + 0.002f;
                const bool onBed = OverBed(f.pos) && std::fabs(f.pos.y - (BED_H + half)) < 0.002f;
                if (onFloor || onBed || onBrick) {                                 // skids to a stop, then it has landed
                    f.vel.y = 0.0f;
                    const float fr = std::max(0.0f, 1.0f - 8.0f * dt);
                    f.vel.x *= fr; f.vel.z *= fr;
                    if (std::sqrt(f.vel.x * f.vel.x + f.vel.z * f.vel.z) < 0.05f) { f.vel = {0.0f, 0.0f, 0.0f}; f.state = FOOD_LANDED; }
                }
            }
            for (Food& f : foods) {                                     // a disk resting on a brick that was lifted away falls again
                if (f.state != FOOD_LANDED || f.pos.y < 0.05f) continue;
                bool supported = OverBed(f.pos) && std::fabs(f.pos.y - (BED_H + half)) < 0.004f;
                for (const Brick& b : bricks)
                    if (!supported && b.state != BRICK_DRAGGED && std::fabs(f.pos.x - b.pos.x) < BRICK_HALF.x && std::fabs(f.pos.z - b.pos.z) < BRICK_HALF.z &&
                        std::fabs(f.pos.y - (b.pos.y + BRICK_HALF.y + half)) < 0.004f) supported = true;
                if (!supported) { f.state = FOOD_FALLING; f.vel = {0.0f, 0.0f, 0.0f}; }
            }
            if (targetFood >= 0) {
                Food& f = foods[targetFood];
                if (avatar.foodEaten) {                                           // eaten: the disk is gone
                    foods.erase(foods.begin() + targetFood);
                    if (grabbedFood > targetFood) grabbedFood--;
                    targetFood = -1; avatar.foodEaten = false;
                } else if (avatar.mode != MODE_FETCH && avatar.mode != MODE_EAT) { // interrupted (grabbed, knocked over): drop it
                    f.state = FOOD_LANDED; f.pos.y = half + 0.002f;
                    targetFood = -1;
                } else if (avatar.foodHeld) {
                    f.pos = avatar.foodPos;                                       // rides in his hand
                }
            }
            // On his feet and not busy dancing? Fetch the oldest disk lying on the floor.
            if (targetFood < 0 && avatar.mode == MODE_STANDING && avatar.danceT <= 0.0f && avatar.danceBlend < 0.02f) {
                for (size_t i = 0; i < foods.size(); i++) {
                    if (foods[i].state != FOOD_LANDED || foods[i].pos.y > 0.05f) continue;   // (can't reach one lying on the bed or the bricks)
                    targetFood = (int)i; foods[i].state = FOOD_TARGETED;
                    avatar.StartFetch(foods[i].pos);
                    break;
                }
            }
        }
 
        StepBricks(bricks, GetFrameTime());
        {                                                               // bricks hurt: collide with his body
            Vector3 avatarVel[JOINT_COUNT];
            const float fdt = std::max(GetFrameTime(), 1e-4f);
            for (int i = 0; i < JOINT_COUNT; i++) {
                Vector3 v = Vector3Scale(Vector3Subtract(avatar.pos[i], lastAvatarPos[i]), 1.0f / fdt);
                const float vl = Vector3Length(v);
                avatarVel[i] = vl > 12.0f ? Vector3Scale(v, 12.0f / vl) : v;
                lastAvatarPos[i] = avatar.pos[i];
            }
            ResolveAvatarBricks(avatar, bricks, avatarVel, fdt);
        }
 
        // Tired enough (and on his feet)? Walk to the bed.
        if (avatar.mode == MODE_STANDING && sleepiness <= SLEEPY_AT && avatar.danceT <= 0.0f && avatar.danceBlend < 0.02f)
            avatar.StartWalkToBed();                            // (a dance in progress finishes first)
        if (avatar.mode == MODE_STANDING && watchRequested && sleepiness > SLEEPY_AT && entertainment < 100.0f &&
            avatar.danceT <= 0.0f && avatar.danceBlend < 0.02f) {  // free, rested and wants to watch: off to the TV
            avatar.StartWatchTV({sg.cx, 0.0f, -ARENA + TV_DISTANCE});
            watchRequested = false;
        }
 
        // ---- draw ----
        BeginDrawing();
        ClearBackground({205, 220, 235, 255});
        BeginMode3D(cam);
            DrawArena();
            DrawBed();
            DrawScreenOnWall(screen, sg, screenOn, screenHover, tvBaseHover);
            for (const Brick& b : bricks) DrawBrick(b);
            for (const Food& f : foods) DrawFood(f, avatar.look.height);
            avatar.Draw();
        EndMode3D();
 
        if (avatar.asleep) {                                    // floating "Zzz" above the sleeping head
            Vector2 sp = GetWorldToScreen(Vector3Add(avatar.pos[HEAD], {0.0f, 0.35f, 0.0f}), cam);
            DrawText("Zzz", (int)sp.x, (int)(sp.y + 6.0f * std::sin((float)GetTime() * 2.0f)), 26, DARKBLUE);
        }
 
        const int sh = GetScreenHeight();
        DrawText("SPACE knock over   R reset   drag avatar joint / drag empty space = turn camera   Wheel zoom", 12, sh - 54, 18, DARKGRAY);
        DrawText("LEFT/RIGHT build   K skin  H hair  C shirt  P pants   B/M/N mood   Z sleepy   X exhausted   (height keys off)", 12, sh - 30, 18, DARKGRAY);
        const char* state = "STANDING";
        if (avatar.mode == MODE_STANDING && avatar.painBlend > 0.5f) state = "OUCH!";
        else if (avatar.mode == MODE_STANDING && avatar.danceT > 0.0f) state = "DANCING";
        else if (avatar.mode == MODE_STANDING && avatar.cryBlend > 0.5f) state = "CRYING";
        if (avatar.mode == MODE_GETUP) state = "GETTING UP";
        else if (avatar.mode == MODE_WALK) state = "WALKING TO BED";
        else if (avatar.mode == MODE_FETCH) state = "WALKING TO FOOD";
        else if (avatar.mode == MODE_EAT) state = "EATING";
        else if (avatar.mode == MODE_TV) state = avatar.tvStage == TV_WALK ? "WALKING TO TV" : avatar.tvStage == TV_SIT ? "SITTING DOWN" :
                                                 avatar.tvStage == TV_RISE ? "GETTING UP" : (screenOn ? "WATCHING TV" : "STARING AT A DEAD SCREEN");
        else if (avatar.mode == MODE_BEDANIM) state = avatar.bedReverse ? "GETTING OUT OF BED" : "GETTING INTO BED";
        else if (avatar.mode == MODE_RAGDOLL)
            state = avatar.asleep ? (avatar.passedOut ? "PASSED OUT" : "ASLEEP") : avatar.groundTimer > 0.0f ? TextFormat("DOWN - getting up in %.1fs", FALL_WAIT - avatar.groundTimer)
                                              : "RAGDOLL";
        DrawText(TextFormat("height %.2f m   build %.1f   %s", look.height, look.build, state), 12, 12, 18, DARKGRAY);
 
        DrawStatBar(px, 12, "Health", health, {220, 70, 70, 255});
        DrawStatBar(px, 38, "Mood",   mood,   MoodColor(mood));
        DrawStatBar(px, 64, "Hunger", hunger, {230, 160, 50, 255});
        DrawStatBar(px, 90, "Sleepiness", sleepiness, SleepColor(sleepiness));
        DrawStatBar(px, 116, "Entertainment", entertainment, {185, 110, 225, 255});
        if (zeroSleepTimer > 0.0f)
            DrawText(TextFormat("Exhausted - passing out in %.0fs", std::ceil(PASS_OUT_AFTER - zeroSleepTimer)), px, 142, 18, MAROON);
        DrawLine(px + STAT_LX + (int)(160 * SLEEPY_AT / 100.0f), 90, px + STAT_LX + (int)(160 * SLEEPY_AT / 100.0f), 108, WHITE);   // "goes to bed" mark
        if (tvBaseHover) {                                                // a hint while the mouse is under the screen
            const Vector2 mp = GetMousePosition();
            DrawText("Click: sit and watch TV", (int)mp.x + 17, (int)mp.y + 19, 18, BLACK);
            DrawText("Click: sit and watch TV", (int)mp.x + 16, (int)mp.y + 18, 18, WHITE);
        }
        if (screenHover) {                                                // a hint while the mouse is over the screen
            const Vector2 mp = GetMousePosition();
            DrawText(screenOn ? "Click: switch the screen off" : "Click: switch the screen on", (int)mp.x + 17, (int)mp.y + 19, 18, BLACK);
            DrawText(screenOn ? "Click: switch the screen off" : "Click: switch the screen on", (int)mp.x + 16, (int)mp.y + 18, 18, WHITE);
        }
 
        DrawRectangleRec(dropBtn, overBtn ? Color{255, 160, 195, 255} : Color{240, 115, 165, 255});
        DrawRectangleLinesEx(dropBtn, 2.0f, {150, 50, 90, 255});
        const int labelW = MeasureText("Drop food", 18);
        DrawText("Drop food", (int)(dropBtn.x + (dropBtn.width - labelW) / 2), (int)dropBtn.y + 7, 18, WHITE);
        EndDrawing();
    }
    UnloadScreenMedia(screen);
    CloseWindow();
    return 0;
}