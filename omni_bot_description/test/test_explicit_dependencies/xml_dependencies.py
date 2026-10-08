"""Helpers for discovering dependencies from XML, URDF, and Xacro files."""

import logging
from pathlib import Path
import xml.etree.ElementTree as ET


def get_package_xml_dependencies(package_xml: Path) -> set[str]:
    """Extract declared package dependencies from package.xml."""
    declared: set[str] = set()
    if not package_xml.exists():
        return declared

    try:
        tree = ET.parse(package_xml)
        root = tree.getroot()
        dep_tags = {'depend', 'exec_depend',
                    'build_depend', 'run_depend', 'test_depend'}
        for child in root:
            if child.tag in dep_tags and child.text:
                declared.add(child.text.strip())
    except ET.ParseError as e:
        logging.warning(f'Failed to parse package.xml: {e}')

    return declared


def get_xacro_dependencies(file_path: Path) -> set[str]:
    """Extract packages referenced by $(find pkg) in XML/Xacro attributes."""
    found: set[str] = set()
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()

        for elem in root.iter():
            for attr_val in elem.attrib.values():
                if '$(find' in attr_val:
                    pkg = attr_val.split('$(find')[1].split(')')[
                        0].replace('-pkg-share', '').strip()
                    found.add(pkg)

            if elem.tag == 'plugin' and elem.text:
                plugin = elem.text.strip()
                if '/' in plugin:
                    found.add(plugin.split('/', maxsplit=1)[0])
                elif '::' in plugin:
                    found.add(plugin.split('::', maxsplit=1)[0])
    except ET.ParseError as e:
        logging.warning(f'Failed to parse XML file {file_path}: {e}')

    return found
