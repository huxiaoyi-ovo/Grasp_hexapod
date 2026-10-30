# Compact climb speed evidence, 2026-09-08

| Artifact | SHA-256 | Role |
| --- | --- | --- |
| `baseline_climb_compact.json` | `771236223a784638476c4640f1859f4cb927ec0328848d94e35c7f6ed9e12214` | Pre-speed active plan |
| `baseline_metrics.json` | `13e4de5c04dbcabb6e7778fbbe2844505d56fe32396d9f1eaf2c86d0035c641f` | CUDA PhysX 1x baseline |
| `refined_climb_compact.json` | `af3dca695cf61279af7e41a79595ce0d5b2c8415c0d8de8e7db8df61fc1b944c` | Accepted active plan |
| `refined_metrics.json` | `fd3ad959f99ce4507e742015fd0c59e4e12d0106d55a1dfab770d45243470043` | CUDA PhysX 1x refined replay |

Both metrics use GPU PhysX, 240 Hz physics, 30 Hz control/actuation,
`--climb-speed 1`, and `--climb-joint-speed 1.2`. They are simulation
diagnostics only and do not prove physical contact, load, stability, friction,
or hardware speed.
