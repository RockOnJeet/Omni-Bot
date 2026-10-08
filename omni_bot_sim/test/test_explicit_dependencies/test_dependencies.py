"""Audit for undeclared dependencies in an ament_python ROS packages."""

from pathlib import Path

from python_dependencies import get_py_imports
from xml_dependencies import get_package_xml_dependencies

# Set of dirs to ignore checking
IGNORED_DIRS: set[str] = {'__pycache__', '.pytest_cache', 'test'}

# Constants
package_root: Path = Path.cwd()
declared: set[str] = get_package_xml_dependencies(package_root / 'package.xml')


def get_dependencies(path: Path, fileType: str) -> set[str]:
    """Discovers dependencies for a particular file type recursively."""
    found: set[str] = set()

    for file_path in path.rglob(fileType):
        if any(part in file_path.parts for part in IGNORED_DIRS):
            continue  # Skip ignored dirs
        if file_path.name == 'setup.py':
            continue  # Skip setup.py
        found.update(get_py_imports(file_path))

    found.discard(path.name)  # Remove root name
    return found


def test_python_dependencies() -> None:
    """Check if python dependencies are declared in `setup.py`."""
    found: set[str] = get_dependencies(package_root, fileType='*.py')
    missing = found - declared
    assert not missing, (
        f'Missing dependencies in package.xml: {sorted(missing)}\n'
        f'Found: {sorted(found)}\n'
        f'Declared: {sorted(declared)}'
    )
