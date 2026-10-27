# FBIK PhysX â€” viewer foundation

A Windows USD viewer with ovphysx simulation, OpenGL, GLFW, Dear ImGui, ovstage, and ovrtx.
It retains the FBIK viewer's ImGui theme, fonts, camera navigation, and Windows
build setup. Character animation, joints, and IK are future work.

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

USD rendering is delegated to the pinned native SDKs, sampled at time zero.
There is no custom skeleton evaluation or skinning in this milestone. Referenced
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

Physics advances at a fixed 1/60 second timestep. Catch-up is capped to 0.1 seconds
per displayed frame; minimizing or opening a file dialog does not accumulate
simulation time. Body poses are written back to the shared in-memory ovstage,
including child geometry. Physics output is never fed back as new physics input.
Source USD files are unchanged.

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
reset, and repeatable trajectories. The small
cube under `tests/data` is a test fixture, not a startup demo.

See [project.md](project.md) for the future ovphysx direction and
[source provenance](third_party/PROVENANCE.md) for copied components and licenses.
