# FBIK PhysX â€” viewer foundation

A Windows USD viewer with ovphysx simulation, OpenGL, GLFW, Dear ImGui, ovstage, and ovrtx.
It retains the FBIK viewer's ImGui theme, fonts, camera navigation, and Windows
build setup. USD animation playback and kinematic character collision are implemented;
dynamic character joints and IK are future work.

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
or `.usdz` file, or select a previously opened file from **Recent USD**. The last
ten successfully loaded paths are stored in `.cache/recent-usd.txt` across runs.
Hover over a recent filename to see its full path. A failed load reports an error
and retains the previous stage when the SDK permits recovery.

You can also start with a file:

```bat
run.bat --scene "D:\assets\scene.usd"
```

The first usable authored camera is selected on load; otherwise the viewer frames
the scene bounds. **Frame stage** or Home restores automatic framing. Left drag
orbits, right drag looks around, middle drag pans, the wheel moves forward/back,
and W/A/S/D and Q/E move the camera. Escape closes the viewer.

USD animation is sampled at time zero on load and advanced by Start animation.
The viewer evaluates the skeleton and updates skinned mesh points, smooth normals,
and bounds for the native renderer. FBIK fits collision shapes at initialization. Referenced
assets must be available to the SDK. Source files are never saved or modified;
the viewer camera and studio light are added only to the in-memory stage.
Initial RTX shader preparation may take several minutes. Runtime logs are under
`deps/ovrtx/bin`.

## Falling box simulation

Click **Load falling box**, then **Start simulation**. The button changes to
**Stop simulation** while the orange box falls onto the static ground. Stop
restores the original pose and discards the physics instance, including velocities
and contact state. Start again to repeat the drop. Loading another USD ends the
previous simulation and opens the new stage stopped.

The editable sample is `resources/scenes/falling_box.usda` (copied beside the
executable by the build). It uses meters, Y-up, gravity of 9.81 m/s², a 1 kg box
with rigid-body and collision APIs, and a thin static floor whose top is y=0.
The sample explicitly selects CPU physics; RTX still renders on the GPU.
Other loaded USD files need enabled rigid bodies and collision/scene settings.
Point-instanced physics output is not supported in this milestone.

While simulation runs, **left-drag a movable body** to pick it up. A temporary
D6 joint pulls the clicked point toward the cursor at the original picking
distance, using damped translation drives while allowing free rotation. The body
continues colliding with the scene. Release the mouse to let go; releasing over
the UI or switching away from the window also ends the drag. Clicking empty space
or static geometry retains camera orbit. Stop and scene replacement clear the
joint. No picking helpers are saved into the source USD.

Physics advances at a fixed 1/60 second timestep. Catch-up is capped to 0.1 seconds
per displayed frame; minimizing or opening a file dialog does not accumulate
simulation time. Body poses are written back to the shared in-memory ovstage,
including child geometry. Physics output is never fed back as new physics input.
Source USD files are unchanged.

## Walking collision test

Load the falling-box scene, click **Start simulation**, then **Start animation**.
SOMA walks toward the cube and pushes it along the floor. The walk runs once and
holds its final pose. **Stop animation** pauses playback at its current pose while
physics keeps running; **Start animation** resumes from that time. Animation
controls are available while simulating and never start or reset physics.
**Stop simulation** ends playback and restores the character, skin, cube, and
animation time. Loading another scene starts stopped.

`resources/scenes/soma_walk.usda` contains a simple procedural test gait bound to
SOMA's existing skeleton. `falling_box.usda` references it and authors the character's
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
shared simulation on Start. Use **Show bone colliders** to inspect the shapes.
SOMA produces 61 convex meshes and omits 16 leaf joints. Reinitializing replaces the
module's generated layer. Source USD files are never changed.

This first version supports classic linear skinning, including joint remapping
and constant/indexed weight primvars. Blend shapes and dual-quaternion skinning
are not supported yet. Shapes retain their fitted geometry and follow animation rigidly.
Dynamic character joints and collision-driven pose correction follow later.

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
reload, visible simulation motion/reset, and the ImGui/OpenGL backend. It needs a working GPU session.
The CPU physics test verifies gravity, ground contact, child transforms, exact
reset, repeatable trajectories, ray picking, dragging, collision resistance, and release. The small
cube under `tests/data` is a test fixture, not a startup demo. The FBIK test covers
multiple characters and skeletons, skin bindings, fallback capsules, leaf omission,
repeat initialization, character transforms, kinematic target tracking, skin deformation,
physical cube displacement, and repeatable animation/reset. The RTX test also checks
visible cube displacement during the walk.

See [project.md](project.md) for the future ovphysx direction and
[source provenance](third_party/PROVENANCE.md) for copied components and licenses.
