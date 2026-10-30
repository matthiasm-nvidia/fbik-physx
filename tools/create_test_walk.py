"""Retarget the bundled CC0 Quaternius Walk_Loop onto SOMA.

Run with Python/OpenUSD (Blender 5.1's bundled Python works). Runtime playback
only needs the generated USD clip. Source provenance: resources/animations/README.md.
"""
from pathlib import Path
from statistics import median
import math
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

ROOT = Path(__file__).resolve().parents[1]
FPS = 60
START, STOP, END = 1.25, 6.75, 8.0
TRAVEL_SPEED = 0.65  # metres/second, matching basic_walk.usda and two_characters.usda
SOURCE_FPS, CYCLE = 30, 40


def quat(matrix):
    return Gf.Quatf(matrix.ExtractRotationQuat()).GetNormalized()


def average(values):
    reference = values[0]
    result = Gf.Quatf(0, Gf.Vec3f(0))
    for value in values:
        result += value if Gf.Dot(reference, value) >= 0 else -value
    return result.GetNormalized()


def smooth(value):
    value = max(0, min(1, value))
    return value * value * (3 - 2 * value)


def main():
    source = Usd.Stage.Open(str(ROOT / 'resources/animations/quaternius_walk.usdc'))
    src_skel = UsdSkel.Skeleton.Get(source, '/Source/Skeleton')
    src_anim = UsdSkel.Animation.Get(source, '/Source/Walk')
    src_names = list(src_skel.GetJointsAttr().Get())
    src_indices = {name.rsplit('/', 1)[-1]: i for i, name in enumerate(src_names)}
    src_topology = UsdSkel.Topology(src_names)
    src_rest = UsdSkel.ConcatJointTransforms(src_topology, src_skel.GetRestTransformsAttr().Get())

    target = Usd.Stage.Open(str(ROOT / 'resources/scenes/soma.usd'))
    skel = UsdSkel.Skeleton.Get(target, '/OUTPUT/c_skeleton_grp/Root')
    names = list(skel.GetJointsAttr().Get())
    leaves = [name.rsplit('/', 1)[-1] for name in names]
    indices = {name: i for i, name in enumerate(leaves)}
    topology = UsdSkel.Topology(names)
    parents = list(topology.GetParentIndices())
    cache = UsdSkel.Cache()
    cache.Populate(UsdSkel.Root.Get(target, '/OUTPUT'), Usd.PrimDefaultPredicate)
    query = cache.GetSkelQuery(skel)
    rest_local = query.ComputeJointLocalTransforms(0)
    rest = UsdSkel.ConcatJointTransforms(topology, rest_local)
    trs = [UsdSkel.DecomposeTransform(m) for m in rest_local]
    positions = [Gf.Vec3f(t[0]) for t in trs]
    scales = Vt.Vec3hArray([t[2] for t in trs])
    mapping = {'Hips': 'pelvis', 'Spine1': 'spine_01', 'Spine2': 'spine_02',
               'Chest': 'spine_03', 'Neck1': 'neck_01', 'Neck2': 'neck_01', 'Head': 'Head'}
    for side, suffix in [('Left', 'l'), ('Right', 'r')]:
        for dst, src in [('Shoulder', 'clavicle'), ('Arm', 'upperarm'), ('ForeArm', 'lowerarm'),
                         ('Hand', 'hand'), ('Leg', 'thigh'), ('Shin', 'calf'), ('Foot', 'foot'),
                         ('ToeBase', 'ball')]:
            mapping[side + dst] = src + '_' + suffix
    mapping = {indices[dst]: src_indices[src] for dst, src in mapping.items()}
    src_hip = src_indices['pelvis']
    # Scale root bob/sway by leg length, preserving SOMA's own rest translations.
    def leg_length(world, hip, knee, foot):
        p = [world[i].ExtractTranslation() for i in (hip, knee, foot)]
        return (p[1] - p[0]).GetLength() + (p[2] - p[1]).GetLength()
    motion_scale = leg_length(rest, indices['LeftLeg'], indices['LeftShin'], indices['LeftFoot']) / leg_length(
        src_rest, src_indices['thigh_l'], src_indices['calf_l'], src_indices['foot_l'])

    def retarget(frame):
        src_world = UsdSkel.ConcatJointTransforms(src_topology, UsdSkel.MakeTransforms(
            src_anim.GetTranslationsAttr().Get(frame), src_anim.GetRotationsAttr().Get(frame),
            src_anim.GetScalesAttr().Get(frame)))
        local, world = [], []
        for i in range(len(names)):
            parent = world[parents[i]] if parents[i] >= 0 else Gf.Quatf(1)
            if i in mapping:
                j = mapping[i]
                # Rest-frame conversion handles different bone rolls and joint axes.
                desired = quat(src_world[j]) * quat(src_rest[j]).GetInverse() * quat(rest[i])
                rotation = parent.GetInverse() * desired
            else:
                rotation = Gf.Quatf(trs[i][1])
            local.append(rotation.GetNormalized())
            world.append((parent * rotation).GetNormalized())
        delta = (src_world[src_hip].ExtractTranslation() - src_rest[src_hip].ExtractTranslation()) * motion_scale
        return local, Gf.Vec3f(delta)

    cycle = [retarget(frame) for frame in range(CYCLE)]
    idle = [average([q[i] for q, _ in cycle]) for i in range(len(names))]
    # Neutral legs and a relaxed, slightly bent upper body for the held start/end pose.
    for i, name in enumerate(leaves):
        if i == 0 or name.startswith(('LeftLeg', 'RightLeg', 'LeftShin', 'RightShin',
                                       'LeftFoot', 'RightFoot', 'LeftToe', 'RightToe')):
            idle[i] = Gf.Quatf(trs[i][1])

    # Calibrate cadence from the retargeted stance feet, not a guessed cycle duration.
    feet = [indices['LeftToeBase'], indices['RightToeBase']]
    foot_samples = []
    for frame in range(CYCLE + 1):
        q, delta = retarget(frame)
        t = Vt.Vec3fArray(positions)
        t[0] += delta
        world = UsdSkel.ConcatJointTransforms(topology, UsdSkel.MakeTransforms(t, Vt.QuatfArray(q), scales))
        foot_samples.append([world[i].ExtractTranslation() for i in feet])
    speeds = []
    for foot in range(2):
        ground = min(sample[foot][1] for sample in foot_samples)
        for a, b in zip(foot_samples, foot_samples[1:]):
            speed = (a[foot][2] - b[foot][2]) * SOURCE_FPS * .01
            if max(a[foot][1], b[foot][1]) < ground + 3 and speed > .2:
                speeds.append(speed)
    if not speeds:
        raise RuntimeError('No stance samples for cadence calibration')
    natural_speed = median(speeds)
    rate = TRAVEL_SPEED / natural_speed

    inverse_bind = [m.GetInverse() for m in skel.GetBindTransformsAttr().Get()]
    meshes = [(cache.GetSkinningQuery(prim), UsdGeom.Mesh(prim).GetPointsAttr().Get())
              for prim in target.Traverse() if prim.IsA(UsdGeom.Mesh) and cache.GetSkinningQuery(prim)]

    def ground_height(translations, rotations):
        world = UsdSkel.ConcatJointTransforms(topology, UsdSkel.MakeTransforms(translations, rotations, scales))
        transforms = Vt.Matrix4dArray([bind * joint for bind, joint in zip(inverse_bind, world)])
        low = math.inf
        for skin, original in meshes:
            points = Vt.Vec3fArray(original)
            if not skin.ComputeSkinnedPoints(transforms, points, 0):
                raise RuntimeError('Unable to skin SOMA for floor clearance')
            low = min(low, min(p[1] for p in points))
        return low

    walk = Usd.Stage.CreateInMemory()
    walk.SetTimeCodesPerSecond(FPS)
    walk.SetStartTimeCode(0)
    walk.SetEndTimeCode(int(END * FPS))
    anim = UsdSkel.Animation.Define(walk, '/Walk')
    walk.SetDefaultPrim(anim.GetPrim())
    anim.CreateJointsAttr(names)
    anim.CreateScalesAttr(scales)
    translations, rotations = anim.CreateTranslationsAttr(), anim.CreateRotationsAttr()
    previous = None
    for frame in range(int(END * FPS) + 1):
        seconds = frame / FPS
        elapsed = max(0, min(seconds - START, STOP - START))
        blend = smooth((seconds - START) / .4) * smooth((STOP - seconds) / .4)
        animated, delta = retarget((elapsed * SOURCE_FPS * rate) % CYCLE)
        q = Vt.QuatfArray([Gf.Slerp(blend, a, b).GetNormalized() for a, b in zip(idle, animated)])
        if previous is not None:
            q = Vt.QuatfArray([b if Gf.Dot(a, b) >= 0 else -b for a, b in zip(previous, q)])
        t = Vt.Vec3fArray(positions)
        t[0] += delta * blend
        # Keep the lowest point of the actual skinned soles 5 mm above the floor.
        t[0] += Gf.Vec3f(0, .5 - ground_height(t, q), 0)
        translations.Set(t, frame)
        rotations.Set(q, frame)
        previous = q
    walk.GetRootLayer().customLayerData = {'source': 'Quaternius Walk_Loop, CC0-1.0',
                                          'retargetedBy': 'tools/create_test_walk.py',
                                          'sourcePlaybackRate': rate}
    walk.GetRootLayer().Export(str(ROOT / 'resources/scenes/soma_walk.usda'))
    print(f'Retargeted {len(mapping)} joints; stance speed {natural_speed:.3f} m/s; playback rate {rate:.3f}')


if __name__ == '__main__':
    main()
