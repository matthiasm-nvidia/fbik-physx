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
The OpenGL display texture is reused until the viewport size changes. All 61 SOMA
bodies and the 1/60-second simulation timestep are retained.

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

## Follow and ragdoll modes

The **Mode** selector in **Scene** applies to all characters. **Follow** keeps bodies
kinematic and driven by animation. **Ragdoll** releases the current pose as dynamic
bodies with zero initial velocities and holds animation/root motion. Physics updates
the skeleton and visible skin after every step. Select a mode while paused or playing;
selecting a mode alone does not start simulation. Returning to Follow snaps to the held
animation pose. **Reset** restores the initial scene and time while retaining the mode.

Spherical joints connect each body to its nearest ancestor with a body, anchored at
the child bone's skeleton base. All rotations are free, with no drives or angular
limits. Root bodies are unanchored. SOMA has 61 bodies and 60 joints. Joints are disabled
in Follow and enabled in Ragdoll. A collision group disables collisions among all
generated bone colliders (including across characters); environment contacts remain
active. Shift-drag works on dynamic bone bodies as well as scene objects.

`fbik::Module::init(stage, ordinal, minSkinWeight)` still builds the generated layer.
`setMode(mode, ordinal)` publishes a dedicated control ordinal that the physics wrapper
consumes through `applyControlChanges`. `updateTargets(poses)` supplies Follow targets.
After a ragdoll step, `readPoses(referencePoses, ordinal)` returns the existing `Pose`
type for skinning. Reference poses are captured at release; bones without bodies retain
their local relationship to their simulated ancestors. FBIK does not advance physics
or sample animation, and simulation output is never submitted as control input.

## Character colliders

On load, `fbik::Module::init` discovers skeletons under USD `SkelRoot` prims.
For each non-leaf joint it fits an oriented box to vertices with skin weights strictly above **0.3**
for that joint, combining bound meshes in the initial evaluated pose.
It then refines the box using 64 support-plane cuts from the decomposer algorithm.
The resulting polygon mesh uses `PhysicsMeshCollisionAPI` with `convexHull`
approximation, so PhysX cooks a convex mesh body shape. A 0.5 mm support allowance
and minimum 1 mm OBB thickness keep sparse or flat input sets usable.
The threshold is an optional `Module::init` parameter. Full skin weights still
determine each vertex position; the threshold only selects hull vertices.
Without qualifying vertices it creates a capsule from the joint to the average
position of its immediate children, with radius 10% of that distance. Zero-length
segments are skipped. Leaf joints receive no collider; the source skeleton stays
intact and joint names have no special meaning.

Generated geometry lives under each character's `__FbikBoneColliders` Xform in
an in-memory layer. It has kinematic rigid-body and collision APIs and becomes part of the
shared simulation on Start simulation. Use **Show bone colliders** to inspect the shapes.
SOMA produces 61 convex meshes and omits 16 leaf joints. Reinitializing replaces the
module's generated layer. Source USD files are never changed.

This first version supports classic linear skinning, including joint remapping
and constant/indexed weight primvars. Blend shapes and dual-quaternion skinning
are not supported yet. Shapes retain their fitted geometry; in Follow mode they follow animation rigidly.
Animation drives, angular limits, and collision-driven animation tracking follow later.

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
repeat initialization, character transforms, kinematic target tracking, skin deformation,
physical cube displacement, repeatable animation/reset, ragdoll mode transitions, joint attachment,
collision filtering, omitted-bone reconstruction, and ragdoll skinning. The RTX test also checks
visible cube displacement during the walk.

See [project.md](project.md) for the future ovphysx direction and
[source provenance](third_party/PROVENANCE.md) for copied components and licenses.

For a hidden-window GUI capture (including the panel):

```bat
build\Release\fbik_viewer.exe --scene resources\scenes\basic_walk.usda --frames 8 --screenshot build\gui.ppm
```
