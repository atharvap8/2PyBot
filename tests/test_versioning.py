import importlib
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"software/radxa/console"))
versioning=importlib.import_module("console.versioning")


@pytest.mark.parametrize("current,next_value",[("3.9.0","3.10.0"),("3.10.0","3.20.0"),
    ("3.20.0","3.30.0"),("3.30.0","3.40.0"),("3.90.0","3.100.0"),("4.0.0","4.1.0")])
def test_requested_revision_sequence(current,next_value):
    assert versioning.next_revision(current)==next_value


@pytest.mark.parametrize("value",["3.11.0","3.12.0","3.13.0","3.21.0","3.20.1"])
def test_intermediate_numbering_cannot_be_built(value):
    with pytest.raises(ValueError):versioning.validate_version(value)


def test_archived_build_identity_can_still_be_read_for_rollback():
    assert versioning.validate_version("3.12.0",allow_legacy=True)==(3,12,0)
    assert versioning.validate_version("3.20.0-rc.1",allow_prerelease=True)==(3,20,0)
