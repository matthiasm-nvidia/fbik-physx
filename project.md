# FBIK with ovphysx

## Purpose

Build a full-body pose correction system for animated characters using ovphysx for most of the physics simulation. The character should follow its animation while responding to environment collisions, self-collisions, and joint limits.

Scene objects and character bones will participate in the same PhysX simulation. PhysX will handle their collision detection and physical interaction. The first implementation will use PhysX joints for the character's constraints and animation tracking. A custom IK solver will be considered only if this approach cannot provide sufficient stability or the desired behavior.

Status: the standalone viewer and initial ovphysx integration are implemented. The falling-box USD verifies gravity and contact; Start/Stop runs and resets the simulation. Kinematic character bodies, USD animation playback, and viewer-side skinning are implemented. A procedural SOMA walk pushes the cube. Follow/ragdoll mode switching, dynamic character bodies, unrestricted spherical joints, bone collision filtering, and physics-to-skin pose reconstruction are implemented. Animation drives, angular limits, and IK remain future work.

## First step: minimal standalone viewer

Create a self-contained viewer by stripping down the existing FBIK repository.
Keep OpenGL/GLFW presentation, ImGui with the same styling wrapper and fonts,
ovrtx rendering, ovstage USD loading, and the existing `build.bat` discovery and
Debug/Release build logic. Provide a native USD file picker and persistent recent
USD files that can be selected to load.

This step includes only the viewer and its necessary dependencies. Do not import
or implement animation, skinning, IK, physics simulation, recording, or demo
features. Preserve the assets already present in this project. The ovphysx
milestones below follow this foundation.

## Agreed direction

- Use ovphysx as the integration layer for PhysX simulation.
- Simulate the environment and its dynamic objects with PhysX.
- Represent character bones as PhysX bodies with collision shapes and mass properties.
- Use PhysX joints first to connect the bones, constrain their motion, and support tracking of the animation pose.
- Let character bodies interact with scene objects through the shared simulation.
- Support character self-collision, with collision filtering for connected or intentionally overlapping bodies.
- Evaluate the native joint approach before deciding whether a custom IK solver is necessary.
- Retain PhysX scene simulation and collision handling if custom IK is introduced.

## Reference project

The existing project at `D:\git\fbik` is the reference for character setup, animation integration, and expected behavior.

Relevant components include:

- Humanoid recognition and mapping between animation joints and physics bodies.
- The reference character physics asset, including body shapes, joint frames, limits, mass, and inertia.
- USD skeleton and animation loading, corrected-pose writeback, CPU skinning, and the RTX viewer.
- Animation tracking, contact response, ragdoll behavior, and diagnostics.
- Test scenes and recorded investigations of unstable contact behavior.

The reference currently includes both a quasi-static rotation-based solver and a custom rigid-body simulation with compliant animation drives. This project will use that work to guide behavior and integration while assigning physics simulation to ovphysx.

## Intended architecture

### Scene simulation

ovphysx will own the shared simulation containing static environment geometry, dynamic scene objects, and character bodies. Collision filtering and physical properties must be configured so contacts produce the intended interaction between them.

The concrete ovphysx interfaces for creating bodies, shapes, joints, and drives must be checked during implementation planning. This document does not assume that every underlying PhysX feature is exposed through ovphysx.

### Character representation

Each simulated bone will have a rigid body and collision geometry. A mapping will relate its physics transform to the corresponding animation joint, including any offset between the joint and body frames. Bones used only for rendering will retain their relationship to the corrected skeleton.

Connected bodies will use joints with appropriate attachment frames and angular limits. Mass and inertia should allow character bodies and scene objects to respond to each other. Policies for the root and feet must be explicit because fixed targets can conflict with collision constraints.

### Animation tracking and IK

The original animation supplies the desired pose. The first approach will use PhysX joints and the available drive mechanisms to pull the character toward that pose while contacts and limits constrain the result.

Here, IK means correcting an animated pose through the jointed physical character. A separate end-effector IK API is not an initial requirement.

Original animation targets and simulated output must remain separate. Corrected output must not become the next animation target unintentionally.

### Frame flow

1. Sample the original animation and character placement.
2. Convert animation transforms into physics body or joint targets.
3. Advance the shared PhysX simulation, resolving character constraints and scene contacts.
4. Read the resulting character body transforms.
5. Reconstruct the corrected skeleton pose and skin the character for rendering.

Simulation must have a clearly defined timestep and a single owner of time advancement. Target updates, substeps, paused animation, and paused simulation must have explicit behavior.

### Custom IK fallback

If the native joint approach remains unstable or cannot meet the required behavior, introduce a custom solver for the character constraints or pose correction that need it.

Before implementing that fallback, define when it runs relative to PhysX simulation, how it consumes contact information, and how its corrections affect body poses and velocities. Its output must cooperate with PhysX contact resolution and preserve meaningful interaction with dynamic objects.

The decision should follow reproducible failures of the native approach and measured attempts to address them through setup, timestep, drives, and solver settings.

## Behavior to establish

- In free space, the character tracks its animated pose with acceptable error.
- Static obstacles cause the body to yield while respecting the configured joint limits.
- Dynamic objects can displace movable character bodies, and the character can move those objects.
- Self-collisions prevent unwanted overlap without forcing adjacent bodies apart at their joints.
- After contact ends, the character returns toward the animation without sustained oscillation or abrupt snapping.
- Holding the animation pose still allows physics interaction and recovery while simulation continues.
- Pausing simulation freezes the simulated state.
- Different rendering rates produce comparable behavior at the same physics timestep.
- Conflicting targets or constraints produce visible diagnostics rather than being treated as a successful solve.

## Implementation milestones

### 1. Verify ovphysx integration

Implemented with ovphysx 0.6.4, ovstage 0.2.1, and ovrtx 0.5.0. The shared ovstage is populated with physics and rendering data after registering the bundled PhysX schemas. Simulation uses fixed 1/60 second steps, publishes body transforms, converts them to local transforms, and recomputes the hierarchy for child geometry. Simulation output ordinals are never drained back into physics.

`resources/scenes/falling_box.usda` contains a static ground slab and a dynamic 1 kg box with 10 cm sides. The viewer's Start simulation button changes to Stop simulation. Stop restores the starting transforms and destroys the physics instance so restarting clears velocities and contacts. Tests cover gravity, settling, child geometry, exact reset, repeatability, and visible RTX motion/reset. Mouse picking uses an in-memory D6 joint with translation spring/damper drives and free rotation. The hit point remains body-local; the world anchor follows the cursor at the initial ray distance. Joint controls use isolated input ordinals, and release, focus loss, Stop, or scene replacement clears the drag. Tests verify picking, lift, collision resistance, release, repeat grabs, and rendered motion. Point-instanced body writeback is outside this milestone.

### 2. Build the physical character

The first pass is implemented behind `fbik::Module`: discover `SkelRoot` characters, fit OBBs to skin vertices whose weight for that bone is strictly above 0.3 and refine them with 64 support-plane cuts into convex mesh colliders, and fall back to hierarchy-based capsules when no vertices are available. Leaf joints have no colliders, without altering the skeleton or using name conventions. A viewer checkbox displays the generated geometry. SOMA yields 61 convex meshes and 16 omitted leaves. Shapes are fitted in the initial evaluated pose and live in a replaceable in-memory USD layer. They now have kinematic rigid-body APIs and follow evaluated bone poses through fixed offsets.

Ragdoll mode now makes the bodies dynamic and enables spherical joints anchored at the skeleton bone bases. All rotation axes are free. Root bodies remain unanchored, and a collision group filters all generated bone-to-bone contacts while preserving environment contact. Physics poses reconstruct the visible skeleton, retaining local poses for omitted bones. Follow remains kinematic. Mode changes preserve the current release pose; returning to Follow snaps to the held animation pose. Next, add angular limits and animation drives.

### 2a. Kinematic walking collision test — implemented

The viewer owns the playback clock and samples USD animation at each physics tick.
It computes skeletal forward kinematics and updates mesh points, smooth vertex normals,
and bounds for rendering. `fbik::Module::characters()` provides skeleton/character
paths and joint ordering; `updateTargets(poses)` accepts a world character transform
and character-space bone matrices and returns local/world body targets. FBIK never
samples animation or steps the solver. The physics module submits a dedicated
ovstage control ordinal before stepping; CPU ovphysx turns these edits into native
kinematic targets. Bone dimensions and offsets are fixed after initialization. Cached ordered queries batch all body targets in one local/world write. Animation sampling caches static skeleton metadata; physics output, ragdoll pose reconstruction, and CPU skinning are deferred until a frame or a control operation needs them. Intermediate physics ticks do not publish to the stage, and output is never drained back as input. Query lifetimes are bounded by their owning stage.

The `basic_walk.usda` sample references `soma_walk.usda`, a generated procedural gait, and separately
authors character translation toward the box. Animation stays paused on load and
reset until Play animation is clicked; that button also starts physics if needed.
Pause animation holds the character pose while physics continues. Stop simulation restores the initial scene,
skin, and animation time. The clip holds its final pose rather than looping root motion.
Tests cover body target tracking, skin deformation/reset, cube displacement, repeatable
restarts, and visible RTX motion. This is one-way kinematic contact: the cube responds,
while the character continues following its animation. Dynamic bones and joints will
provide collision-driven character response in the next milestone.

### 3. Track animation with native joints

Connect sampled animation targets to the available joint drives. Implement pose reconstruction and assess tracking error, attachment error, and recovery behavior in free space.

### 4. Evaluate coupled interaction

Test static obstacles, self-contact, sustained loads, and impacts from dynamic objects. Compare stiffness, damping, timestep, and solver settings using repeatable scenes. Include cases with conflicting root or foot targets once their policy is chosen.

### 5. Decide whether custom IK is needed

Document the remaining failures and the limits of the native approach. Continue with PhysX joints if they meet the requirements. Otherwise, define and test a narrowly scoped custom solver integration while retaining PhysX simulation for the scene.

## Validation and diagnostics

Use small controlled tests alongside the animated humanoid scene. Measure animation tracking error, joint attachment error, joint-limit violations, penetration, pose changes between frames, and unexpected velocity spikes. Record the timestep and solver settings with each comparison.

Initial test cases should include a free-space animated chain, a limb against a static obstacle, character self-contact, a dynamic object pressing against a held pose, an impact followed by recovery, and a walking character encountering obstacles.

Set numerical acceptance thresholds after establishing scene scale and a baseline. Visual inspection alone is insufficient to establish stability, and low penetration alone is insufficient if the pose visibly oscillates.

## Open decisions

- Which ovphysx body, joint, and drive interfaces will be used?
- Which parts of the existing viewer and animation adapter should be reused?
- What joint types and drive formulation best express the animation targets?
- Should joint limits remain authored, or expand from original animation as in the reference project?
- How should root placement and foot targets constrain the physical character?
- How should independent body transforms be written back while preserving skeletal attachment and bone-length requirements?
- What timestep, substep policy, and target interpolation should be used?
- What measured failures would justify introducing custom IK?

The immediate priority is to validate the PhysX-joint approach. Whole-character locomotion planning, automatic foot placement, and get-up behavior are outside the initial scope.
