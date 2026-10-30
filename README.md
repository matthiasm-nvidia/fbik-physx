# FBIK PhysX — viewer foundation

A Windows USD viewer with ovphysx simulation, OpenGL, GLFW, Dear ImGui, ovstage, and ovrtx.
It retains the FBIK viewer's ImGui theme, fonts, camera navigation, and Windows
build setup. USD animation playback, kinematic character collision, and dynamic
ragdolls are implemented. Dynamic Follow mode uses PhysX animation drives; custom IK remains future work.

## Build and run

Requires Windows x64, Visual Studio with the C++ desktop tools and bundled
CMake/Ninja, and an RTX-capable NVIDIA GPU with a driver compatible with the
pinned ovrtx package.

```bat
setup_deps.bat
build.bat
run.bat
```

`setup_deps.bat` installs the checksum-pinned SDK packages into this project's
`deps` directory. `build.bat` discovers Visual Studio and builds Debug and Release.
All sources, fonts, third-party libraries, and SDK paths are local to this project;
building and running do not require the original FBIK checkout.

The viewer opens the **Basic walk** demo in **Follow** mode on startup, with playback paused.
Click **Load USD...** to open a `.usd`, `.usda`, `.usdc`,
or `.usdz` file, or select a previously opened file from **Recent**. The last
ten successfully loaded paths are stored in `.cache/recent-usd.txt` across runs.
The current file's full path appears in the window title. Hover over a recent
filename to see its full path. A failed load reports an error
and retains the previous stage when the SDK permits recovery.

You can also start with a file:

```bat
run.bat --scene "D:\assets\scene.usd"
```

The first usable authored camera is selected on load; otherwise the viewer frames
the scene bounds. **Frame stage** or Home restores automatic framing. Left drag
orbits, right drag looks around, middle drag pans, the wheel moves forward/back,
and W/A/S/D and Q/E move the camera. Escape closes the viewer.

USD animation is sampled at the authored start time on load and advanced during playback.
The viewer evaluates the skeleton and updates skinned mesh points, smooth normals,
and bounds for the native renderer. FBIK fits collision shapes at initialization. Referenced
assets must be available to the SDK. Source files are never saved or modified;
the viewer camera and studio light are added only to the in-memory stage.
Initial RTX shader preparation may take several minutes. Runtime logs are under
`deps/ovrtx/bin`.

## Falling box simulation

Select **Falling box** from the scene dropdown, then open **Scene** and click
**Start simulation**. **Pause simulation** freezes the current state without resetting
velocities or contacts. **Reset** restores the starting scene and animation time;
if physics playback was running, it restarts from there with character animation
paused. **Step** pauses playback and advances
one 1/60-second physics tick. Space toggles play/pause. Loading another USD opens it stopped.

The compact panel follows the original FBIK viewer: load/reload, **Scene**, **Info**,
and **Camera controls**. **Recent** retains this viewer's file history.

The editable sample is `resources/scenes/falling_box.usda` (copied beside the
executable by the build). It uses meters, Y-up, gravity of 9.81 m/s², a 1 kg box with 10 cm sides
with rigid-body and collision APIs, and a 5 m by 5 m static floor whose top is y=0. The box starts with its center 1 m above the floor. It has no character.
The sample explicitly selects CPU physics; RTX still renders on the GPU.
Other loaded USD files need enabled rigid bodies and collision/scene settings.
Point-instanced physics output is not supported in this milestone.

**Shift + left-drag a movable body** to pick it up, including before starting
simulation. A successful grab starts or resumes physics without starting character
animation. A temporary
D6 joint pulls the clicked point toward the cursor at the original picking
distance, using damped translation drives while allowing free rotation. The body
continues colliding with the scene. Release the mouse to let go; releasing over
the UI or switching away from the window also ends the drag. Clicking empty space
or static geometry retains camera orbit. Reset and scene replacement clear the
joint. No picking helpers are saved into the source USD.

Physics advances at a fixed 1/60 second timestep. Catch-up is capped to 0.1 seconds
per displayed frame; minimizing or opening a file dialog does not accumulate
simulation time. Body poses are written back to the shared in-memory ovstage,
including child geometry. Physics output is never fed back as new physics input.
Source USD files are unchanged.

Kinematic targets use cached, ordered path queries and one batched local/world transform
write per physics tick. Follow updates native spring targets instead. Skeleton topology, rest poses, animation mappings, and mesh
bindings are cached when a scene loads. Intermediate catch-up ticks stay in PhysX;
body output, ragdoll reconstruction, and skinning are published once for the displayed
frame (or when picking, scrubbing, or switching modes needs the latest poses).
The OpenGL display texture is reused until the viewport size changes. SOMA uses 23
bodies after collapsing hand and head descendants, with a 1/60-second simulation timestep.

## Walking collision test

Select **Basic walk** from the scene dropdown and click **Start simulation**.
The scene opens with a closer camera view and a 60 x 30 x 30 cm box.
Animation stays paused after loading, resetting, or starting physics. Click
**Play animation** to make SOMA walk toward the box and push it along the floor.
This starts physics if needed. **Pause animation** holds the current character
pose while physics continues; **Play animation** resumes it. **Loop** repeats the
clip; turn it off to hold the final pose. The **Time** slider scrubs and pauses
animation, including while simulation is stopped. **Speed** scales playback while
preserving the fixed physics timestep. **Reset** restores the character, skin,
cube, and animation time.

**Frame character** focuses the first character, and **Follow camera** moves the camera
with it. **Info** reports its joint and collider counts. **Record movie** saves
rendered frames to an MP4 through FFmpeg (which must be on PATH); **Stop recording**
finalizes it. Recording excludes the GUI, advances time at 30 frames per second,
and stops when the window is resized or a new scene is loaded.

`resources/scenes/soma_walk.usda` contains the authored Quaternius **Walk_Loop**,
retargeted onto SOMA's existing skeleton. `basic_walk.usda` references it and authors the character's
forward movement. `soma.usd` remains unchanged. This is not a Mixamo/motion-capture
clip. Cadence matches the route's 0.65 m/s speed, with relaxed start/end poses and
floor clearance calculated from the skinned mesh. Source motion, licenses and
regeneration details are in [resources/animations/README.md](resources/animations/README.md).
`tools/create_test_walk.py` regenerates the clip with Python USD; Python is
not required to build or run the viewer.

Select **Two characters** for two SOMA characters walking side by side. The demo
is `resources/scenes/two_characters.usda`; it shares the walk clip, floor, box and
physics scene from Basic walk. Play/pause, the timeline, mode changes and Reset
apply to both characters together. Each character has its own 23 capsules and 22
joints, and the opening camera frames both.

The viewer owns the fixed-step clock, USD sampling, forward kinematics and CPU
linear skinning. FBIK receives evaluated character-space bone poses and maps them
through fixed bone-to-body offsets. Physics receives matching local/world body
transforms on a separate control ordinal; ovphysx converts those changes to
kinematic targets before its 1/60-second step. Physics output is never drained back
as control input. This path uses the sample's CPU physics configuration (DirectGPU
must be disabled).

## Character modes

The **Mode** selector in **Scene** applies to all characters. Newly loaded scenes use **Follow** by default.
**Kinematic** keeps bodies kinematic and driven exactly by animation. **Follow** uses dynamic bodies with damped
position and orientation springs toward the sampled animation. Contacts can deflect
the bones, and their simulated poses drive the visible skin. Character gravity is
disabled in Follow; scene props keep their normal gravity. **Ragdoll** releases the current pose as dynamic
bodies with zero initial velocities and holds animation/root motion. **Quasi static** also
keeps bodies dynamic, but disables their gravity and clears linear and angular velocities
before and after each fixed simulation step. Contacts, joint limits, and dragging can still
correct positions; bodies retain no momentum between steps. Scene props keep their normal
physics. This is a quasi static approximation, not an equilibrium solver. Physics updates
the skeleton and visible skin after every step. Select a mode while paused or playing;
selecting a mode alone does not start simulation. Returning to Kinematic snaps to the held
animation pose. **Reset** restores the initial scene and time while retaining the mode.

Limited joints connect each body to its nearest ancestor with a body, anchored at
the child bone's skeleton base. Knees and elbows use revolute hinges; other connections
use D6 ball joints with locked translation, limited twist, and circular swing cones.
Root bodies have no skeletal attachment to the world. SOMA has 23 bodies and 22 joints: four hinges and 18 ball
joints. Skeletal joints are disabled in Kinematic and enabled in Follow, Ragdoll and Quasi static. Bone collisions
exclude connected bodies and pairs whose padded rest-pose boxes overlap within a character.
Other bone pairs, different characters, and the environment can collide. Shift-drag works on dynamic bone bodies as well as scene objects.

`fbik::Module::init(stage, ordinal, minSkinWeight, jointDamping)` builds the generated layer.
`setMode(mode, ordinal)` publishes a dedicated control ordinal that the physics wrapper
consumes through `applyControlChanges`. `updateTargets(poses)` supplies animation targets for Kinematic or Follow.
After a dynamic step, `readPoses(referencePoses, ordinal)` returns the existing `Pose`
type for skinning. Reference poses track animation in Follow and are captured at release in Ragdoll/Quasi static; bones without bodies retain
their local relationship to their simulated ancestors. Fingers, eyes, jaw, and other
collapsed descendants keep their reference local joint angles while following the hand
or head body. FBIK does not advance physics or sample animation, and simulation output is never submitted as control input.

### Follow animation drives

Each bone has an additional world-to-body D6 joint with all six axes free. Native
PhysX acceleration drives apply linear springs and a quaternion SLERP orientation
spring; skeletal attachments, angular limits and contacts are solved in the same
physics step. This is force-based tracking, not a blend or a body-transform override.
All bones, including the root and feet, can yield; there is no hard foot pinning or
balance controller. Incompatible animation, limits and contacts can leave tracking error.

**Stiffness** controls how strongly animation is held; **Damping** reduces oscillation.
Defaults are 400 per second squared and 40 per second, shared by translation and
rotation. Acceleration drives account for mass/inertia when applying the gains;
the bodies retain finite mass for two-way interaction with scene objects.
Sampled linear/angular target velocities reduce playback lag. Pausing animation
holds its target while physics and recovery continue. Pausing simulation freezes
recovery. Seeking or looping replaces the target without injecting a discontinuous
target velocity; the bodies approach it through simulation rather than teleporting.

`PhysicsSimulation::setFollowTargets()` caches world-space goals without writing
body transforms. `setFollowSettings()` changes the gains. Native joints are reacquired
and configured each tick so mouse-joint population changes cannot discard the goals.
Selecting Follow preserves the current body pose; selecting Ragdoll or Quasi static
disables all animation drives. Reset rebuilds the initial pose while retaining the mode.

## Joint frames and angular limits

Frames are derived once from the skeleton rest pose, independently of the initially
sampled animation and collider rotations. The head-to-feet direction selects positive
global Y or Z as up; without those landmarks, the larger rest-skin AABB extent on Y/Z
is the fallback. This avoids mistaking a rotated T-pose's arm span for height. Foot-to-toe directions
(preferably ToeBase) are projected horizontally, normalized per foot, and averaged to
resolve forward. Right is forward cross up. Without usable toes, paired left/right
landmarks resolve forward; otherwise the shorter horizontal AABB axis is the fallback.
This assumes an upright character; the AABB fallback also assumes height exceeds arm span on Y/Z.

Knees/shins/calves/lower legs and elbows/forearms/lower arms become hinges. The cross
product of adjacent rest-pose segments determines the hinge axis when the bend is at
least five degrees from collinearity. Near straight, knees use character-right and
elbows use character-up, projected perpendicular to the upper segment. The axis sign
matches that convention. The two joint frames preserve the measured initial bend, so
zero degrees means a straight limb. Knee limits are -60 to 0 degrees about character-right:
negative rotation bends the shin backward, opposite foot-forward, while zero prevents
hyperextension. Elbows use 0 to 120 on the right and -120 to 0 on the left. Axes are
stored in body-local frames.

Ball-joint twist follows the bone's longitudinal direction, with zero rotation at the
rest pose. Head twist uses the incoming neck-to-head direction. These are initial,
symmetric simulation presets; swing values are cone half-angles, in degrees:

| Joint | Twist | Maximum swing |
| --- | --- | --- |
| Internal joints (default) | -5 to 5 | 5 |
| Shoulder: parent lacks arm, child contains arm | -60 to 60 | 100 |
| Hip: parent lacks leg, child contains leg | -30 to 30 | 60 |
| Head: child contains head | -20 to 20 | 30 |

Names are case-insensitive and use only the final component of each bone path. Hinge
classification takes precedence. Shoulder/clavicle helpers, spine, neck, wrists and ankles
use the tight default unless they meet an exception above. SOMA has 13 tight internal
ball joints and five wider ones. Generated bodies request 32 position and four velocity
solver iterations to enforce the tight limits through the connected chain.

Presets and frame inference are in `src/fbik/JointSetup.cpp`. USD D6 limits provide
translation locks and twist bounds. `PhysicsSimulation` reads the generated
`fbik:swingLimit` attribute and applies the native PhysX circular cone as well, since
USD's per-axis swing bounds alone form a pyramid. It reapplies cones after mode and
population changes and reacquires native pointers. Enabled joints are realized with a
minimal initialization step when necessary; all starting body poses and velocities
are restored afterward to avoid startup impulses. Custom stage population must include
`fbik:swingLimit` and `fbik:followBody` for this integration. Limits are active in Follow, Ragdoll and Quasi static; Kinematic remains
kinematic and unconstrained.

All generated skeletal joints have angular damping through acceleration-mode
`PhysicsDriveAPI` drives: the hinge axis for knees/elbows and all three angular
axes for ball joints. Stiffness and target velocity are zero, so damping resists
relative rotation while leaving a stationary joint free to hold any permitted pose.
The `jointDamping` initialization parameter defaults to **5 per second**; set it
to zero to disable damping. Acceleration mode accounts for body inertia, making the
setting consistent across different bone sizes. The generated USD converts this
rate to the importer's degree-based angular gains. These passive dampers remain separate from Follow animation drives.

## Character colliders

On load, `fbik::Module::init` discovers skeletons under USD `SkelRoot` prims.
Every bone collider is a capsule.
Hands and heads each become one body including all descendant skin. Detection is
case-insensitive and examines only the final component of each joint path. A name
containing `hand` or `wrist` identifies a hand, and a name containing `head` identifies
a head. If an arm subtree has no named hand or wrist, a child without `arm` below a
parent containing `arm` is treated as the hand. Named hands beyond helper bones take
precedence over this fallback. The outermost detected root absorbs its whole subtree;
descendants receive no separate bodies or physics joints.

For each retained non-leaf joint, it selects vertices with skin weights strictly above
**0.3**, combining bound meshes in the initial evaluated pose. For a hand or head,
qualifying influences on any descendant select vertices for that root's capsule.
Each mesh vertex is included only once per body even when several influences map to it.
The threshold applies to individual influences, not their sum over a subtree.
The threshold is an optional `Module::init` parameter. Full skin weights still
determine each vertex position; the threshold only selects fitting vertices.

Selected skin points are projected into a frame with X along
the bone, Y from `Vec3::perp()`, and Z from X cross Y. The capsule is centered at
the projected bounds' midpoint. Its radius is half the smaller Y/Z span, and its
cylinder length is `max(0, spanX - 2 * radius)`. A zero-length cylinder is a sphere;
the minimum diameter is 1 mm in world space. Bone direction points toward the average
immediate child position. Terminal hands/heads and coincident joints use an incoming
ancestor direction, falling back to the bone's X axis. Capsules are native USD/PhysX
primitives, with matching bounds for rendering.

Without qualifying vertices it creates a capsule from the joint to the average
position of its immediate children, with radius 10% of that distance. Zero-length
segments are skipped. Leaf joints receive no collider, except a detected hand, wrist,
or head with qualifying skin vertices. The complete source skeleton stays intact.

Generated geometry lives under each character's `__FbikBoneColliders` Xform in
an in-memory layer. It has kinematic rigid-body and collision APIs and becomes part of the
shared simulation on Start simulation. Use **Show bone colliders** to inspect the shapes.
SOMA produces 23 capsules, absorbs 52 hand/head descendants, and omits two
remaining leaf joints. Reinitializing replaces the
module's generated layer. Source USD files are never changed.

Bone self-collision is enabled. At initialization, the module excludes directly
connected physics bodies, including connections across omitted bones, and pairs within
the same `SkelRoot` whose capsules overlap in the rest pose. Fitted capsules are moved
into the rest pose using their bone-to-body offsets. The test compares the shortest
distance between their center segments with the sum of their radii, including zero-length
sphere cases. Touching counts as overlap, with only a floating-point roundoff allowance.
Separate capsules remain collision-enabled even if their bounding boxes overlap.
Different characters are never automatically excluded from each other.

Exclusions are authored as
`PhysicsFilteredPairsAPI` relationships and remain fixed through animation, mode changes,
and reset. Initial animation overlaps do not create extra exclusions.

Open **Info > Inspect exclusions** to see each excluded bone pair and its reason
(**Connected** or **Rest overlap**); hover over bone names for full paths. The same list
is available through `Module::collisionExclusions()`, and initialization reports counts
for both reasons. Rest-pose exclusions remain fixed; animation-pose sampling and per-pair overrides are not implemented.

This first version supports classic linear skinning, including joint remapping
and constant/indexed weight primvars. Blend shapes and dual-quaternion skinning
are not supported yet. Shapes retain their fitted geometry; in Kinematic mode they follow animation rigidly.
Follow mode retains these shapes while allowing physics to deflect their poses.

## Verification

```bat
ctest --test-dir build -C Release --output-on-failure
```

To include the hidden-window GPU smoke test:

```bat
build.bat -DVIEWER_TEST_RTX=ON
ctest --test-dir build -C Release --output-on-failure
```

Run the same viewer workflow with the two-character demo using:

```bat
build\Release\viewer_smoke.exe . --two-characters
```

The GPU test checks USD loading, rendered output, resize, failed-load recovery,
reload, visible simulation motion/reset, timeline seeking/looping, character framing,
and the ImGui/OpenGL backend. Movie recorder tests exercise MP4 output and encoder failures. It needs a working GPU session.
The CPU physics test verifies gravity, ground contact, child transforms, exact
reset, repeatable trajectories, ray picking, dragging, collision resistance, and release. The small
cube under `tests/data` is a test fixture, not a startup demo. The FBIK test covers
multiple characters and skeletons, skin bindings, fitted and fallback capsules,
rotated/scaled capsule bounds, zero-length spheres, sparse inputs, leaf omission,
capsule overlap (including spheres and overlapping bounds), rest-versus-initial pose filters, native self/cross-character contacts, and joint damping without pose springs,
hand/head subtree aggregation, helper-aware name detection, preserved descendant angles,
Y/Z-up character frames, bent and straight hinge calibration, signed limits, circular
swing versus pyramidal bounds, actual ragdoll angular limits, and reset repeatability,
repeat initialization, character transforms, kinematic target tracking, skin deformation,
physical cube displacement, repeatable animation/reset, Follow tracking and recovery, stiffness/damping response, dynamic impact, mode transitions, gravity and momentum clearing, dragging, joint attachment,
collision filtering, omitted-bone reconstruction, and ragdoll skinning. The RTX test also checks
visible cube displacement during the walk.

See [project.md](project.md) for the future ovphysx direction and
[source provenance](third_party/PROVENANCE.md) for copied components and licenses.

For a hidden-window GUI capture (including the panel):

```bat
build\Release\fbik_viewer.exe --scene resources\scenes\basic_walk.usda --frames 8 --screenshot build\gui.ppm
```
