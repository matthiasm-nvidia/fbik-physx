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
No animation, solver, collision, ragdoll, recording, or physics-demo implementation
was imported.
