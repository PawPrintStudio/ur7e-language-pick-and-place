# Docstring-convention gate (PEP 257), run by `colcon test`.
from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found docstring style errors / warnings"
