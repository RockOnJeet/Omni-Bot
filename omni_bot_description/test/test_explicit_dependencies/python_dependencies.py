"""Helpers for discovering dependencies from Python files."""

import ast
import importlib.metadata
import logging
from pathlib import Path
import sys


# Python distribution names that do not directly match their rosdep keys.
PYTHON_DEPENDENCY_MAP: dict[str, str] = {
    'ament-index-python': 'ament_index_python',
    'launch-ros': 'launch_ros',
    'pytest': 'python3-pytest',
}


def get_python_dependency(module: str) -> str:
    """Convert a Python module name to its package.xml dependency name."""
    distributions = importlib.metadata.packages_distributions().get(module, [])

    if len(distributions) == 1:
        distribution = distributions[0]
        return PYTHON_DEPENDENCY_MAP.get(distribution, distribution)

    return module


def get_py_imports(path: Path) -> set[str]:
    """Extract external Python and ROS executable dependencies from a file."""
    dependencies: set[str] = set()
    try:
        tree = ast.parse(
            path.read_text(encoding='utf-8'), filename=str(path))
        for node in ast.walk(tree):
            match node:
                case ast.Import(names=names):
                    for alias in names:
                        module = alias.name.split('.')[0]
                        if module not in sys.stdlib_module_names:
                            dependencies.add(get_python_dependency(module))
                case ast.ImportFrom(module=module, level=0):
                    if module is not None:
                        module = module.split('.')[0]
                        if module not in sys.stdlib_module_names:
                            dependencies.add(get_python_dependency(module))
                case ast.Call(func=func, keywords=keywords):
                    if (isinstance(func, ast.Name) and func.id == 'Node') or (
                            isinstance(func, ast.Attribute) and
                            func.attr == 'Node'):
                        for keyword in keywords:
                            if keyword.arg == 'package':
                                if isinstance(keyword.value, ast.Constant) and isinstance(
                                        keyword.value.value, str):
                                    dependencies.add(keyword.value.value)
    except (OSError, SyntaxError, UnicodeError) as e:
        logging.warning(f'Failed to parse {path}: {e}')

    return dependencies
