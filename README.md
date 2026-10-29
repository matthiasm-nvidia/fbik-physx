# FBIK PhysX â€” viewer foundation

A Windows USD viewer with ovphysx simulation, OpenGL, GLFW, Dear ImGui, ovstage, and ovrtx.
It retains the FBIK viewer's ImGui theme, fonts, camera navigation, and Windows
build setup. USD animation playback, kinematic character collision, and dynamic
ragdolls are implemented; animation drives and IK are future work.

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

The viewer starts empty. Click **Load USD...** to open a `.usd`, `.usda`, `.usdc`,
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

Bone targets use cached, ordered path queries and one batched local/world transform
write per physics tick. Skeleton topology, rest poses, animation mappings, and mesh
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

`resources/scenes/soma_walk.usda` contains a simple procedural test gait bound to
SOMA's existing skeleton. `basic_walk.usda` references it and authors the character's
forward movement. `soma.usd` remains unchanged. This is not a Mixamo/motion-capture
clip. `tools/create_test_walk.py` regenerates the clip with Python USD; Python is
not required to build or run the viewer.

The viewer owns the fixed-step clock, USD sampling, forward kinematics and CPU
linear skinning. FBIK receives evaluated character-space bone poses and maps them
through fixed bone-to-body offsets. Physics receives matching local/world body
transforms on a separate control ordinal; ovphysx converts those changes to
kinematic targets before its 1/60-second step. Physics output is never drained back
as control input. This path uses the sample's CPU physics configuration (DirectGPU
must be disabled).

## Character modes

The **Mode** selector in **Scene** applies to all characters. **Follow** keeps bodies
kinematic and driven by animation. **Ragdoll** releases the current pose as dynamic
bodies with zero initial velocities and holds animation/root motion. **Quasi static** also
keeps bodies dynamic, but disables their gravity and clears linear and angular velocities
before and after each fixed simulation step. Contacts, joint limits, and dragging can still
correct positions; bodies retain no momentum between steps. Scene props keep their normal
physics. This is a quasi static approximation, not an equilibrium solver. Physics updates
the skeleton and visible skin after every step. Select a mode while paused or playing;
selecting a mode alone does not start simulation. Returning to Follow snaps to the held
animation pose. **Reset** restores the initial scene and time while retaining the mode.

Limited joints connect each body to its nearest ancestor with a body, anchored at
the child bone's skeleton base. Knees and elbows use revolute hinges; other connections
use D6 ball joints with locked translation, limited twist, and circular swing cones.
Root bodies are unanchored. SOMA has 23 bodies and 22 joints: four hinges and 18 ball
joints. Joints are disabled in Follow and enabled in Ragdoll and Quasi static. A collision group disables collisions among all
generated bone colliders (including across characters); environment contacts remain
active. Shift-drag works on dynamic bone bodies as well as scene objects.

`fbik::Module::init(stage, ordinal, minSkinWeight)` still builds the generated layer.
`setMode(mode, ordinal)` publishes a dedicated control ordinal that the physics wrapper
consumes through `applyControlChanges`. `updateTargets(poses)` supplies Follow targets.
After a dynamic step, `readPoses(referencePoses, ordinal)` returns the existing `Pose`
type for skinning. Reference poses are captured at release; bones without bodies retain
their local relationship to their simulated ancestors. Fingers, eyes, jaw, and other
collapsed descendants keep their reference local joint angles while following the hand
or head body. FBIK does not advance physics or sample animation, and simulation output is never submitted as control input.

## Joint frames and angular limits

Frames are derived once from the skeleton rest pose, independently of the initially
sampled animation and collider OBB rotations. The head-to-feet direction selects positive
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
`fbik:swingLimit` for this integration. Limits are active in Ragdoll and Quasi static; Follow remains
kinematic and unconstrained. Animation drives are not implemented yet.

## Character colliders

On load, `fbik::Module::init` discovers skeletons under USD `SkelRoot` prims.
Hands and heads each become one body including all descendant skin. Detection is
case-insensitive and examines only the final component of each joint path. A name
containing `hand` or `wrist` identifies a hand, and a name containing `head` identifies
a head. If an arm subtree has no named hand or wrist, a child without `arm` below a
parent containing `arm` is treated as the hand. Named hands beyond helper bones take
precedence over this fallback. The outermost detected root absorbs its whole subtree;
descendants receive no separate bodies or physics joints.

For each retained non-leaf joint it fits an oriented box to vertices with skin weights
strictly above **0.3**, combining bound meshes in the initial evaluated pose. For a hand
or head, qualifying influences on any descendant select vertices for that root's hull.
Each mesh vertex is included only once per body even when several influences map to it.
The threshold still applies to individual influences, not their sum over a subtree.
It then refines the box using 64 support-plane cuts from the decomposer algorithm.
The resulting polygon mesh uses `PhysicsMeshCollisionAPI` with `convexHull`
approximation, so PhysX cooks a convex mesh body shape. A 0.5 mm support allowance
and minimum 1 mm OBB thickness keep sparse or flat input sets usable.
The threshold is an optional `Module::init` parameter. Full skin weights still
determine each vertex position; the threshold only selects hull vertices.
Without qualifying vertices it creates a capsule from the joint to the average
position of its immediate children, with radius 10% of that distance. Zero-length
segments are skipped. Leaf joints receive no collider, except a detected hand, wrist,
or head with qualifying skin vertices. The complete source skeleton stays intact.

Generated geometry lives under each character's `__FbikBoneColliders` Xform in
an in-memory layer. It has kinematic rigid-body and collision APIs and becomes part of the
shared simulation on Start simulation. Use **Show bone colliders** to inspect the shapes.
SOMA produces 23 convex meshes, absorbs 52 hand/head descendants, and omits two
remaining leaf joints. Reinitializing replaces the
module's generated layer. Source USD files are never changed.

This first version supports classic linear skinning, including joint remapping
and constant/indexed weight primvars. Blend shapes and dual-quaternion skinning
are not supported yet. Shapes retain their fitted geometry; in Follow mode they follow animation rigidly.
Animation drives and collision-driven animation tracking follow later.

## Verification

```bat
ctest --test-dir build -C Release --output-on-failure
```

To include the hidden-window GPU smoke test:

```bat
build.bat -DVIEWER_TEST_RTX=ON
ctest --test-dir build -C Release --output-on-failure
```

The GPU test checks USD loading, rendered output, resize, failed-load recovery,
reload, visible simulation motion/reset, timeline seeking/looping, character framing,
and the ImGui/OpenGL backend. Movie recorder tests exercise MP4 output and encoder failures. It needs a working GPU session.
The CPU physics test verifies gravity, ground contact, child transforms, exact
reset, repeatable trajectories, ray picking, dragging, collision resistance, and release. The small
cube under `tests/data` is a test fixture, not a startup demo. The FBIK test covers
multiple characters and skeletons, skin bindings, fallback capsules, leaf omission,
hand/head subtree aggregation, helper-aware name detection, preserved descendant angles,
Y/Z-up character frames, bent and straight hinge calibration, signed limits, circular
swing versus pyramidal bounds, actual ragdoll angular limits, and reset repeatability,
repeat initialization, character transforms, kinematic target tracking, skin deformation,
physical cube displacement, repeatable animation/reset, ragdoll/quasi static transitions, gravity and momentum clearing, dragging, joint attachment,
collision filtering, omitted-bone reconstruction, and ragdoll skinning. The RTX test also checks
visible cube displacement during the walk.

See [project.md](project.md) for the future ovphysx direction and
[source provenance](third_party/PROVENANCE.md) for copied components and licenses.

For a hidden-window GUI capture (including the panel):

```bat
build\Release\fbik_viewer.exe --scene resources\scenes\basic_walk.usda --frames 8 --screenshot build\gui.ppm
```
