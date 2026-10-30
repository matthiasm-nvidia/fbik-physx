# Walk source and retargeting

`quaternius_walk.usdc` contains one 40-frame, 30 fps cycle of **Walk_Loop** from
Quaternius's **Universal Animation Library, Standard**, plus the source skeleton's
rest/bind transforms. It was extracted from the existing `D:/git/fbik` example's
`resources/characters/walker/{character,walk}.usdc` assets. It contains no meshes.

The reference project obtained `UAL1_Standard.glb` (upload 17958403) and the
**Superhero_Male_FullBody** rig (Universal Base Characters, Standard, upload
15861669) on October 12, 2026. Both are dedicated to the public domain under CC0
1.0. Original notices are preserved in `LICENSE-Quaternius.txt` and
`LICENSE-source-rig.txt`. Animation credits: Quaternius and Gonzalo Furnier.

- [Universal Animation Library](https://quaternius.itch.io/universal-animation-library)
- [Universal Base Characters](https://quaternius.itch.io/universal-base-characters)

`tools/create_test_walk.py` retargets the authored motion onto SOMA's existing
skeleton. It converts between rest frames, preserves bone lengths, transfers
pelvis motion in proportion to leg length, and retains the original arm swing,
ankle articulation and toe roll. Cadence is calibrated from stance-foot motion
to the demos' 0.65 m/s travel. The clip blends from/to a relaxed held pose, and
floor clearance is calculated from the actual skinned mesh at each sample.

The generated `resources/scenes/soma_walk.usda` is self-contained. Neither the
other repository nor Blender is required to build or run the viewer. Regenerate
with Python and OpenUSD, for example Blender 5.1's bundled Python:

```powershell
& 'C:/Program Files/Blender Foundation/Blender 5.1/5.1/python/bin/python.exe' tools/create_test_walk.py
```

Then run `build.bat` to copy the updated clip beside the viewer executables.
