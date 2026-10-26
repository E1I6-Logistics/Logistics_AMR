"""Package and install the Logitle experiment runner."""

from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "logitle_experiments"


def config_data_files():
    """Collect YAML configurations while retaining subdirectories."""
    files = []
    for path in glob("config/**/*.yaml", recursive=True):
        destination = Path("share") / package_name / Path(path).parent
        files.append((str(destination), [path]))
    return files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        *config_data_files(),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="woozoo",
    maintainer_email="woozoo.dev@gmail.com",
    description="Repeatable AMCL and Nav2 accuracy experiments for Logitle AMRs.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "navigation_sweep = logitle_experiments.runners.navigation_sweep:main",
            "amcl_validation = logitle_experiments.runners.amcl_validation:main",
            "analyze_run = logitle_experiments.analysis.report:main",
        ],
    },
)
