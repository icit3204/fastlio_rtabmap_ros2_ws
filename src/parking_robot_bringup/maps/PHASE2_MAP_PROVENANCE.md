# Phase 2 Map Provenance

## Source assets

| Source | Size | SHA256 |
|---|---:|---|
| `scripts/offline_nav_maps/clean_map.yaml` | 142 | `7b1bc006ccedb0aa119d5cdad41212921b53e60ca95bafd97e0de3dc40acbd83` |
| `scripts/offline_nav_maps/clean_map.pgm` | 4679169 | `69428c94a8032fd54939492afe848156ef1687faf7a8023609a3e26d9604beb0` |

## Copied Phase 2 package assets

| Copy | Size | SHA256 |
|---|---:|---|
| `src/parking_robot_bringup/maps/phase2_clean_map.yaml` | 149 | `e0add8a549d19cc75b5a5f4d1b362e401eb7eac811949196c7fd09b9487bdaaa` |
| `src/parking_robot_bringup/maps/phase2_clean_map.pgm` | 4679169 | `69428c94a8032fd54939492afe848156ef1687faf7a8023609a3e26d9604beb0` |

The copied PGM is byte-identical to the source PGM. The copied YAML differs only in the `image:` field, changed from `clean_map.pgm` to `phase2_clean_map.pgm` so the installed YAML resolves to the installed image in the isolated package.

## Map metadata

| Field | Value |
|---|---|
| image | `phase2_clean_map.pgm` |
| mode | `trinary` |
| resolution | `0.0500` m/cell |
| origin | `[-39.1000, -85.1500, 0.0000]` |
| negate | `0` |
| occupied_thresh | `0.65` |
| free_thresh | `0.196` |
| dimensions | 1744 x 2683 cells |

## Authority statement

This copy is an isolated Phase 2 static navigation test asset. It is not a new site-map authority and does not modify or supersede the original map/database assets.

