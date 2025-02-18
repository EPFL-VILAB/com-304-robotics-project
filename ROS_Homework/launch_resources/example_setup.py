import os
from glob import glob

from setuptools import find_packages, setup

package_name = "example"

# Iterate through all the files and subdirectories to build the data files array
# We need this to maintain the file structure of `models/` in the installed package
def generate_data_files(share_path, folder):
    data_files = []

    for path, _, files in os.walk(folder):
        list_entry = (
            os.path.join(share_path, path),
            [os.path.join(path, f) for f in files if not f.startswith(".")],
        )
        data_files.append(list_entry)

    return data_files

setup(
    ...,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*"))),
    ] + generate_data_files(os.path.join("share", package_name), "models"),
    ...
)