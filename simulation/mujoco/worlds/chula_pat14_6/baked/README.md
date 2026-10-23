# Baked lighting variant

The `baked` variant of the Chula Pat14 world. `scene.xml` adds a static visual atlas
with baked lighting to the colliders, movable objects, hinged door and analytic floor
plane of the low-poly world. `baked.blend` holds the hidden procedural originals, the
lights and the packed HDR lightmap used for rebaking. The PNG lightmap in
`../scene_assets/` is the texture that MuJoCo reads.

Keep this directory next to the shared `../scene_assets/` directory, which the XML
references. To edit, rebake and export this variant, see
[Edit and export the room](../../../../../docs/guides/simulation-chula-pat14.md#edit-and-export-the-room)
and the [world README](../README.md).
