import io
import os
from setuptools import find_packages, setup

def read(*paths, **kwargs):
    content = ""
    with io.open(
        os.path.join(os.path.dirname(__file__), *paths),
        encoding=kwargs.get("encoding", "utf8"),
    ) as open_file:
        content = open_file.read().strip()
    return content

def read_requirements(path):
    return [
        line.strip()
        for line in read(path).split("\n")
        if not line.startswith(('"', "#", "-", "git+"))
    ]

setup(
    name="siyi_sdk",
    version="1.0.1",
    description="Python package implementation for Siyi SDK with extended functionalities for the SIYI ZR30 camera.",
    url="https://github.com/fbenti/siyi_zr30_sdk_ros2_submodule",
    long_description=read("README.md"),
    long_description_content_type="text/markdown",
    author="Filippo Bentivoglio",
    packages=find_packages(exclude=["*/test",".github"]),
    platforms=["Linux"],
    python_requires=">=3.9",
    # install_requires=read_requirements("requirements.txt"),
)