# Source provenance

The viewer foundation is derived from the local `D:\git\fbik` repository.
The standalone renderer and navigation shell come from its initial viewer
milestone, commit `c2d4e82`. Camera and stage helpers, the ImGui wrapper, Windows
build/dependency scripts, and the required math types were taken from commit
`7835643590e469d6b7d6ba47caa6448e839a7abc` and stripped to viewer needs.

These components originally derive from the Play viewer, recorded by FBIK at
revision `b7ff085090247f67d361cccb04720e75ee8b6520`. Existing source notices are
retained. No source checkout outside this project is required to build or run it.

Bundled dependencies:

- GLFW 3.4: `third_party/glfw`, with `LICENSE.md`.
- Dear ImGui 1.52 WIP and its matching GLFW/OpenGL backend: `third_party/imgui`,
  with `LICENSE.txt`.
- Noto Sans: `resources/fonts`, with `LICENSE`.

ovstage and ovrtx are pinned in `dependencies.json` and installed under ignored
`deps` directories with their distributed licenses and third-party notices.
No animation, solver, collision, ragdoll, or physics-demo implementation was imported
in the viewer foundation. The GUI alignment later imports `MovieRecorder.{h,cpp}`
and its test from `D:\git\fbik` revision
`9600c79d520815dda9d6118cf37a42935b3a6157`, and adapts its MP4 file dialog and
panel layout. Recording launches FFmpeg from PATH and does not bundle its SDK.

## Character geometry

`src/fbik/MeshTools/Vec.h` is copied from `include/MeshTools/Vec.h` in
`D:\GIT\mesh-tools-lib`, revision `49bb32a395fc2040d71126e2db755cca295f2cc6`.
`Geometry.cpp` contains the point-cloud PCA OBB path extracted from
`source/lib/MeshTools/Geometry.cpp`; `HeaderMath.h` contains its Jacobi/eigenvector
helpers from `source/lib/MeshTools/utils/HeaderMath.h`. Surface fitting, convex
optimization, and other unrelated geometry functions are excluded; temporary
storage uses `std::vector`. The original NVIDIA source notices are retained.
These files are bundled here and require no external source checkout.

The viewer-side skinning implementation follows the linear-skinning and ovstage
mesh-write approach inspected in the original FBIK `src/animation/Skeleton.cpp`
and `src/io/usd/UsdAnimation.cpp`. It uses this project's pose/primvar helpers and
supports only the existing classic-linear character path. No solver was imported.
`resources/scenes/soma_walk.usda` is generated locally by
`tools/create_test_walk.py`; it contains no downloaded animation.

## Convex bone colliders

`src/fbik/MeshTools/ConvexHull.{h,cpp}` adapts the convex polyhedron representation,
Fibonacci support directions, and `splitBelow` clipping implementation from
`D:\GIT\decomposer/include/decomposer/ConvexShape.h` and `src/ConvexShape.cpp`
(revision `b525be958c6c11f8c0f0fcfa330ca265f1ad37ed`). It uses the already bundled
MeshTools vector types and OBB fitter. The adapted entry point refines an existing
OBB in its local frame using 64 support planes and a small thickness allowance.
No decomposition pipeline or external source dependency is added.
