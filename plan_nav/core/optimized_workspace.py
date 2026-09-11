"""Identity binding for PlanNav workspaces authored from optimized RTAB poses."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from core.db_loader import OPTIMIZED_IMPORT_MODE, OPTIMIZED_IMPORTER_VERSION
from core.topology_identity import atomic_write_text


WORKSPACE_MANIFEST_NAME = 'plannav_workspace_manifest.json'
WORKSPACE_SCHEMA_VERSION = 1


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def prepare_optimized_workspace(
    database_path: str | Path,
    workspace_path: str | Path,
    session_id: str,
) -> dict:
    """Create or verify an immutable DB/frame binding for a new workspace.

    Existing matching manifests are reused. A mismatched manifest is rejected
    rather than silently rebinding topology to another database.
    """
    database = Path(database_path).resolve()
    workspace = Path(workspace_path).resolve()
    database_sha = sha256_file(database)
    identity = {
        'schema_version': WORKSPACE_SCHEMA_VERSION,
        'mapping_session_id': session_id,
        'database_path': str(database),
        'database_sha256': database_sha,
        'frame': 'map',
        'coordinate_semantic': 'OPTIMIZED_RTAB_GRAPH',
        'import_mode': OPTIMIZED_IMPORT_MODE,
        'importer_version': OPTIMIZED_IMPORTER_VERSION,
    }
    manifest_path = workspace / WORKSPACE_MANIFEST_NAME
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding='utf-8'))
        for key, expected in identity.items():
            if existing.get(key) != expected:
                raise ValueError(
                    f'workspace identity mismatch for {key}: '
                    f'{existing.get(key)!r} != {expected!r}'
                )
        return existing

    workspace.mkdir(parents=True, exist_ok=True)
    manifest = {
        **identity,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'topology_status': 'PROPOSAL_NOT_YET_APPROVED',
    }
    atomic_write_text(
        manifest_path,
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + '\n',
    )
    return manifest


def finalize_optimized_workspace(workspace_path: str | Path) -> dict:
    """Record operator-approved topology identity without changing DB binding."""
    workspace = Path(workspace_path).resolve()
    manifest_path = workspace / WORKSPACE_MANIFEST_NAME
    if not manifest_path.exists():
        raise ValueError(f'{WORKSPACE_MANIFEST_NAME} is missing')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    database = Path(manifest['database_path'])
    actual_sha = sha256_file(database)
    if actual_sha != manifest.get('database_sha256'):
        raise ValueError(
            f'database hash changed: {actual_sha} != {manifest.get("database_sha256")}'
        )
    topology_path = workspace / 'topology_manifest.json'
    if not topology_path.exists():
        raise ValueError('topology_manifest.json is missing')
    topology = json.loads(topology_path.read_text(encoding='utf-8'))
    manifest.update({
        'topology_status': 'OPERATOR_APPROVED',
        'topology_id': topology['topology_id'],
        'topology_version': topology['topology_version'],
        'node_count': int(topology['node_count']),
        'edge_count': int(topology['edge_count']),
        'approved_at_utc': datetime.now(timezone.utc).isoformat(),
    })
    atomic_write_text(
        manifest_path,
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + '\n',
    )
    return manifest
