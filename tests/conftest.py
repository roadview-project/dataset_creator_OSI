"""Shared fixtures for the test suite."""

import pytest
from pathlib import Path


PLAYGROUND_DIR = Path(__file__).resolve().parent.parent / ".playground"


@pytest.fixture
def mcap_path(tmp_path):
    """Provide a temporary path for an MCAP output file."""
    return tmp_path / "test_output.mcap"
