"""Generate the small SOMA collision-test walk. Requires Python USD only to regenerate.
This is a procedural test gait, not a motion-capture/Mixamo animation.
"""
from pathlib import Path
import math
from pxr import Usd, UsdSkel, Gf, Vt
root = Path(__file__).resolve().parents[1]
source = Usd.Stage.Open(str(root / 'resources/scenes/soma.usd'))
skel = UsdSkel.Skeleton.Get(source, '/OUTPUT/c_skeleton_grp/Root')
cache = UsdSkel.Cache(); cache.Populate(UsdSkel.Root.Get(source, '/OUTPUT'), Usd.PrimDefaultPredicate)
query = cache.GetSkelQuery(skel)
names = list(skel.GetJointsAttr().Get())
local = query.ComputeJointLocalTransforms(0)
global_pose = query.ComputeJointSkelTransforms(0)
parents = list(query.GetTopology().GetParentIndices())
trs = [UsdSkel.DecomposeTransform(m) for m in local]
walk = Usd.Stage.CreateInMemory()
walk.SetTimeCodesPerSecond(60);walk.SetStartTimeCode(0);walk.SetEndTimeCode(480)
a = UsdSkel.Animation.Define(walk, '/Walk')
a.CreateJointsAttr(names)
a.CreateScalesAttr(Vt.Vec3hArray([v[2] for v in trs]))
translations=a.CreateTranslationsAttr();rotations=a.CreateRotationsAttr()
for frame in range(0,481,2):
 t=frame/60
 moving=max(0,min(t-1.25,5.5))
 blend=min(moving/.35,1)*min(max(0,5.5-moving)/.35,1)
 phase=2*math.pi*moving/1.05
 ts=[];qs=[]
 for i,name in enumerate(names):
  leaf=name.rsplit('/',1)[-1];p,q,scale=trs[i];p=Gf.Vec3f(p);q=Gf.Quatf(q)
  side=1 if leaf.startswith('Left') else -1
  if leaf in ('LeftArm','RightArm'):
   parent_rotation=Gf.Matrix4d(global_pose[parents[i]]).ExtractRotation()
   down_axis=parent_rotation.GetInverse().TransformDir(Gf.Vec3d(0,0,1))
   q=Gf.Quatf(Gf.Rotation(down_axis,-70*side).GetQuat())*q
  angle=0
  if leaf in ('LeftLeg','RightLeg'):angle=24*math.sin(phase+(0 if side==1 else math.pi))*blend
  elif leaf in ('LeftShin','RightShin'):angle=32*max(0,math.sin(phase+(0 if side==1 else math.pi)))*blend
  elif leaf in ('LeftArm','RightArm'):angle=-18*math.sin(phase+(0 if side==1 else math.pi))*blend
  if angle:
   parent_rotation=Gf.Matrix4d(global_pose[parents[i]]).ExtractRotation()
   axis=parent_rotation.GetInverse().TransformDir(Gf.Vec3d(1,0,0))
   q=Gf.Quatf(Gf.Rotation(axis,angle).GetQuat())*q
  if leaf=='Hips':p[1]+=1.5*(1-math.cos(phase*2))*blend
  ts.append(p);qs.append(q)
 translations.Set(Vt.Vec3fArray(ts),frame);rotations.Set(Vt.QuatfArray(qs),frame)
walk.SetDefaultPrim(a.GetPrim())
walk.GetRootLayer().Export(str(root/'resources/scenes/soma_walk.usda'))
# Scene placement is kept separate from the reusable skeletal clip.
p=root/'resources/scenes/falling_box.usda'
s=p.read_text()
s=s.replace('    upAxis = "Y"','    upAxis = "Y"\n    startTimeCode = 0\n    endTimeCode = 480\n    timeCodesPerSecond = 60')
s=s.replace('        double3 xformOp:scale', '''        double3 xformOp:translate.timeSamples = {
            0: (2, 0, 0),
            75: (2, 0, 0),
            405: (-1.575, 0, 0),
            480: (-1.575, 0, 0),
        }
        float xformOp:rotateY = -90
        double3 xformOp:scale''',1)
s=s.replace('["xformOp:translate", "xformOp:scale"]','["xformOp:translate", "xformOp:rotateY", "xformOp:scale"]',1)
s=s.replace('''        {
        }
    }
''','''        {
            over "c_skeleton_grp" {
                over "Root" {
                    rel skel:animationSource = </World/Walk>
                }
            }
        }
    }
    def SkelAnimation "Walk" (prepend references = @soma_walk.usda@</Walk>) {}
''',1)
# Do not reapply scene changes when regenerating just the clip.
if '@soma_walk.usda@' not in p.read_text():p.write_text(s)

