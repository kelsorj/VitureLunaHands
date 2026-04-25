# Third-party notices

This repository bundles or references third-party code/assets under their
respective licenses.

## Bundled in this repository

| Path | License | Source |
|---|---|---|
| `vendor/mujoco_menagerie/shadow_hand/` | Apache-2.0 | [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) |
| `vendor/dex_urdf/robots/hands/shadow_hand/` | MIT | [dexsuite/dex-urdf](https://github.com/dexsuite/dex-urdf) |

The bundled subsets are unmodified except for `scene_teleop.xml` and
`left_hand_prefixed.xml` under `vendor/mujoco_menagerie/shadow_hand/`,
which were added to the menagerie tree to support dual-hand rendering.
Both new files are released under the same Apache-2.0 license as the
surrounding directory.

## Referenced but not bundled

| Component | Why excluded |
|---|---|
| Viture Unity XR SDK (`com.viture.xr`) | Proprietary; non-redistributable per its license |
| Viture native libglasses SDK (`viture_native_sdk`) | Proprietary; non-redistributable per its license |

To run the live-from-glasses path, sign up at
[developer.viture.com](https://developer.viture.com), download the Unity XR
SDK, and place it at `vendor/com.viture.xr/` (the path referenced from
`unity-app/Packages/manifest.json`). The sim and mock-sender paths run
without it.

## Python dependencies

Pulled by `uv sync` from PyPI. Notable transitive licenses include:

- `mujoco` — Apache-2.0
- `dex-retargeting` — MIT
- `pinocchio` — BSD-2-Clause
- `pydantic` — MIT
- `websockets` — BSD-3-Clause

See `service/uv.lock` for the full pinned dependency graph.
