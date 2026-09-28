# Style gate: flake8 over the package, run by `colcon test` (CI runs this on
# every PR). Style checks in CI keep review about substance, not formatting.
from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, "Found %d code style errors / warnings:\n" % len(errors) + "\n".join(
        errors
    )
