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
//   Bed (corner): carry the avatar onto the bed and let go. Lying still for 1 s = asleep.
//                sleepiness refills while asleep; at 100 it wakes and stands up beside the bed.
//   Mood > 60: he dances for 2.5 s at random intervals.  Mood < 25: he cries (hands over his face).
//   M / N        test shortcuts: set mood to 15 (sad) / 100 (happy)
//   Z            make him sleepy right now (sets sleepiness to the go-to-bed level)
//   Below 25 sleepiness a standing avatar walks to the bed, sits on the edge, lies down and sleeps.
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
 
enum Mode { MODE_STANDING, MODE_RAGDOLL, MODE_GETUP, MODE_WALK, MODE_BEDANIM };
 
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
static const float MOOD_CRY_BELOW   = 25.0f;    // below this he covers his face and cries
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
 
    // Bed interaction
    float bedTimer = 0.0f;                 // time spent lying still on the mattress
    int   walkStage = 0;                   // 0 = pick route, 1 = via the corner waypoint, 2 = to the bedside
    float gaitPhase = 0.0f;                // walk cycle position (radians)
    float gaitAmp   = 0.0f;                // 0 = standing, 1 = full stride (ramps in/out)
    float bedT      = 0.0f;                // clock of the bed animation (runs backwards when getting out)
    bool  bedReverse = false;              // true while getting OUT of bed
    float wakeBlend = 0.0f;                // 0..1: eases from wherever he was lying into the animation
    Vector3 wakeStart[JOINT_COUNT];
 
    // Emotes (set moodLevel from the game loop each frame)
    float moodLevel   = 100.0f;
    float danceT      = 0.0f;                  // seconds of the current dance left (0 = not dancing)
    float danceClock  = 0.0f;                  // drives the dance moves
    float danceBlend  = 0.0f;                  // 0..1 ease in/out of the dance pose
    float cryBlend    = 0.0f;                  // 0..1 ease in/out of hands-on-face
    float emoteClock  = 0.0f;
    float nextDanceIn = 5.0f;                  // random countdown to the next dance
    bool  asleep   = false;
 
    bool  Pinned(int i) const { return mode == MODE_STANDING || mode == MODE_WALK || mode == MODE_BEDANIM || i == grabbed; }
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
 
        const Vector3 fwd = {0.0f, 0.0f, 1.0f}, back = {0.0f, 0.0f, -1.0f};
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
        }
        loc[L_KNEE] = TwoBone(loc[L_HIP], loc[L_FOOT], Bone(L_HIP, L_KNEE), Bone(L_KNEE, L_FOOT), fwd);
        loc[R_KNEE] = TwoBone(loc[R_HIP], loc[R_FOOT], Bone(R_HIP, R_KNEE), Bone(R_KNEE, R_FOOT), fwd);
        loc[L_ELB]  = TwoBone(loc[L_SHO], loc[L_HAND], Bone(L_SHO, L_ELB),  Bone(L_ELB, L_HAND),  bendL);
        loc[R_ELB]  = TwoBone(loc[R_SHO], loc[R_HAND], Bone(R_SHO, R_ELB),  Bone(R_ELB, R_HAND),  bendR);
 
        for (int j = 0; j < JOINT_COUNT; j++) {
            out[j] = {org.x + loc[j].x * cy + loc[j].z * sy,
                      loc[j].y,
                      org.z - loc[j].x * sy + loc[j].z * cy};
        }
    }
 
    // Snap to the standing pose at the current origin/yaw
    void Stand() {
        mode = MODE_STANDING; grabbed = -1; groundTimer = 0.0f; getupT = 0.0f;
        bedTimer = 0.0f; asleep = false; walkStage = 0; gaitPhase = 0.0f; gaitAmp = 0.0f; bedT = 0.0f; bedReverse = false;
        danceT = 0.0f; danceBlend = 0.0f; cryBlend = 0.0f; nextDanceIn = (float)GetRandomValue(30, 100) / 10.0f;
        ComputePose(KEY_STAND, origin, yaw, pos);
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
    }
 
    void HardReset() { origin = {0.0f, 0.0f, 0.0f}; yaw = 0.0f; Stand(); }
 
    // Switch to physics. `push` gives the upper body a shove so it topples.
    void GoLimp(bool push) {
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
        mode = MODE_RAGDOLL; groundTimer = 0.0f; getupT = 0.0f; bedReverse = false;
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
 
    // Get out of bed: the get-into-bed animation in reverse (lie -> sit on the edge -> stand beside the bed)
    void WakeUp() {
        asleep = false; bedTimer = 0.0f; grabbed = -1;
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
 
    void StepRagdoll(float dt) {
        const Vector3 g = {0.0f, -9.81f, 0.0f};
        for (int i = 0; i < JOINT_COUNT; i++) {
            if (Pinned(i)) continue;
            Vector3 vel = Vector3Scale(Vector3Subtract(pos[i], prev[i]), 0.996f);
            prev[i] = pos[i];
            pos[i] = Vector3Add(Vector3Add(pos[i], vel), Vector3Scale(g, dt * dt));
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
        {
            const bool over  = OverBed(pos[PELVIS]) && pos[PELVIS].y < BED_H + 0.25f;
            const float speed = Vector3Length(Vector3Subtract(pos[PELVIS], prev[PELVIS])) / dt;
            if (grabbed < 0 && over && speed < 0.4f) bedTimer += dt;
            else if (!over || grabbed >= 0) { bedTimer = 0.0f; asleep = false; }
            if (bedTimer >= 1.0f) asleep = true;
        }
 
        // Fall detection: pelvis near the floor and nobody holding the avatar
        if (grabbed < 0 && pos[PELVIS].y < 0.20f * look.height) groundTimer += dt;
        else groundTimer = 0.0f;
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
 
        // gait: full stride while moving, small shuffle while turning, fade out when done
        float targetAmp = moving ? 1.0f : (arrived ? 0.0f : 0.5f);
        gaitAmp += Clamp(targetAmp - gaitAmp, -4.0f * dt, 4.0f * dt);
        gaitPhase += (moving ? 5.5f : 2.5f) * dt;
 
        PoseKey k = KEY_STAND;
        k.pelvisY = 0.515f - 0.022f * gaitAmp - 0.010f * gaitAmp * std::fabs(std::sin(gaitPhase));   // knees soften, body bobs
        k.lean = 4.0f * gaitAmp;
        k.lean += 6.0f * cryBlend;
        UpdateEmotes(dt, false, arrived && walkStage == 2);   // keeps crying on the way, settles at the bedside
        PoseTweak tw; tw.coverFace = cryBlend;
        ComputePose(k, origin, yaw, pos, gaitPhase, gaitAmp, &tw);
        for (int i = 0; i < JOINT_COUNT; i++) prev[i] = pos[i];
 
        if (arrived && walkStage == 2 && std::fabs(WrapPi(PI * 0.5f - yaw)) < 0.05f && gaitAmp < 0.05f && cryBlend < 0.01f) {
            origin = BedsideSpot(); yaw = PI * 0.5f;
            mode = MODE_BEDANIM; bedT = 0.0f; bedReverse = false;
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
        const float dTarget = danceT > 0.0f ? 1.0f : 0.0f;
        const float cTarget = (!calm && moodLevel < MOOD_CRY_BELOW) ? 1.0f : 0.0f;
        danceBlend += Clamp(dTarget - danceBlend, -6.0f * dt, 6.0f * dt);
        cryBlend   += Clamp(cTarget - cryBlend,   -2.5f * dt, 2.5f * dt);
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
 
    void Step(float dt) {
        if (mode == MODE_STANDING) StepStanding(dt);
        else if (mode == MODE_RAGDOLL) StepRagdoll(dt);
        else if (mode == MODE_GETUP) StepGetUp(dt);
        else if (mode == MODE_WALK) StepWalk(dt);
        else if (mode == MODE_BEDANIM) StepBedAnim(dt);
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
 
static void DrawStatBar(int x, int y, const char* label, float value, Color fill) {
    const int w = 160, h = 18, lx = 100;     // lx = label column width
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
    bool  draggingCam = false;              // left-drag that started on empty space
 
    // Needs (0..100). Health and hunger are placeholders for now; only mood is live.
    float health = 100.0f, hunger = 100.0f, mood = 100.0f;
    float moodTick = 0.0f;                  // counts up to MOOD_REGEN_INTERVAL, then mood recovers by 1 point
    float sleepiness = 100.0f;              // 100 = rested (light blue), 0 = exhausted (dark blue)
    float sleepTick = 0.0f;                 // counts up to SLEEP_INTERVAL, then sleepiness drops
    Vector3 focus = {0.0f, 0.0f, 0.0f};     // camera follows the avatar on the ground plane
 
    while (!WindowShouldClose()) {
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
        if (IsKeyPressed(KEY_R)) avatar.HardReset();
        if (IsKeyPressed(KEY_SPACE) && (avatar.mode == MODE_STANDING || avatar.mode == MODE_WALK)) avatar.GoLimp(true);   // knock over
        if (IsKeyPressed(KEY_M)) mood = 15.0f;                                     // test shortcuts: sad ...
        if (IsKeyPressed(KEY_N)) mood = 100.0f;                                    // ... and happy
        if (IsKeyPressed(KEY_Z)) sleepiness = std::min(sleepiness, SLEEPY_AT);     // test shortcut: make him sleepy now
 
        // ---- mouse grab ----
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetMouseRay(GetMousePosition(), cam);
            float best = 1e9f; int hit = -1;
            for (int i = 0; i < JOINT_COUNT; i++) {
                RayCollision c = GetRayCollisionSphere(ray, avatar.pos[i], 0.07f * avatar.look.height);
                if (c.hit && c.distance < best) { best = c.distance; hit = i; }
            }
            if (hit >= 0) {
                if (avatar.mode != MODE_RAGDOLL) avatar.GoLimp(false);   // grabbing cancels a get-up too
                avatar.grabbed = hit;
                grabDist = best;
            } else {
                draggingCam = true;                                      // missed the avatar: turn the camera
            }
        }
        if (avatar.grabbed >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetMouseRay(GetMousePosition(), cam);
            Vector3 t = Vector3Add(ray.position, Vector3Scale(ray.direction, grabDist));
            const float lim = ARENA - 0.07f * avatar.look.height;       // held joint can't go through walls
            t.x = Clamp(t.x, -lim, lim);
            t.z = Clamp(t.z, -lim, lim);
            t.y = std::max(t.y, 0.03f);
            PushOutOfBox(t, 0.05f, BED);                                // ...or through the bed
            avatar.prev[avatar.grabbed] = avatar.pos[avatar.grabbed];
            avatar.pos[avatar.grabbed]  = t;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) { avatar.Release(); draggingCam = false; }
 
        avatar.moodLevel = mood;
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
        // Tired enough (and on his feet)? Walk to the bed.
        if (avatar.mode == MODE_STANDING && sleepiness <= SLEEPY_AT && avatar.danceT <= 0.0f && avatar.danceBlend < 0.02f)
            avatar.StartWalkToBed();                            // (a dance in progress finishes first)
 
        // ---- draw ----
        BeginDrawing();
        ClearBackground({205, 220, 235, 255});
        BeginMode3D(cam);
            DrawArena();
            DrawBed();
            avatar.Draw();
        EndMode3D();
 
        if (avatar.asleep) {                                    // floating "Zzz" above the sleeping head
            Vector2 sp = GetWorldToScreen(Vector3Add(avatar.pos[HEAD], {0.0f, 0.35f, 0.0f}), cam);
            DrawText("Zzz", (int)sp.x, (int)(sp.y + 6.0f * std::sin((float)GetTime() * 2.0f)), 26, DARKBLUE);
        }
 
        DrawText("SPACE knock over   R reset   drag avatar joint / drag empty space = turn camera   Wheel zoom", 12, 10, 18, DARKGRAY);
        DrawText("UP/DOWN height   LEFT/RIGHT build   K skin  H hair  C shirt  P pants", 12, 34, 18, DARKGRAY);
        const char* state = "STANDING";
        if (avatar.mode == MODE_STANDING && avatar.danceT > 0.0f) state = "DANCING";
        else if (avatar.mode == MODE_STANDING && avatar.cryBlend > 0.5f) state = "CRYING";
        if (avatar.mode == MODE_GETUP) state = "GETTING UP";
        else if (avatar.mode == MODE_WALK) state = "WALKING TO BED";
        else if (avatar.mode == MODE_BEDANIM) state = avatar.bedReverse ? "GETTING OUT OF BED" : "GETTING INTO BED";
        else if (avatar.mode == MODE_RAGDOLL)
            state = avatar.asleep ? "ASLEEP" : avatar.groundTimer > 0.0f ? TextFormat("DOWN - getting up in %.1fs", FALL_WAIT - avatar.groundTimer)
                                              : "RAGDOLL";
        DrawText(TextFormat("height %.2f m   build %.1f   %s", look.height, look.build, state), 12, 58, 18, DARKGRAY);
 
        const int px = GetScreenWidth() - 310;
        DrawStatBar(px, 12, "Health", health, {220, 70, 70, 255});
        DrawStatBar(px, 38, "Mood",   mood,   MoodColor(mood));
        DrawStatBar(px, 64, "Hunger", hunger, {230, 160, 50, 255});
        DrawStatBar(px, 90, "Sleepiness", sleepiness, SleepColor(sleepiness));
        DrawLine(px + 100 + (int)(160 * SLEEPY_AT / 100.0f), 90, px + 100 + (int)(160 * SLEEPY_AT / 100.0f), 108, WHITE);   // "goes to bed" mark
        EndDrawing();
    }
    CloseWindow();
    return 0;
}