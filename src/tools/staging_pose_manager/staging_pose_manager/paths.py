"""Shared package-local data paths."""

import os
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent.parent


def records_dir():
    """Return the records directory, allowing an optional environment override."""
    override = os.environ.get("STAGING_POSE_RECORD_DIR")
    if override:
        return Path(override).expanduser().resolve()
    return PACKAGE_DIR / "records"
