import os
import yaml

from pathlib import Path
import pandas as pd
import numpy as np
from os import path
import yaml

from ament_index_python.packages import get_package_share_directory


def try_load_mpc_config():
    try:
        mpc_pkg_path = get_package_share_directory("multi_purpose_mpc_ros")
        mpc_config_path = Path(mpc_pkg_path) / "config" / "config.yaml"
        with open(mpc_config_path, "r") as f:
            mpc_config = yaml.safe_load(f)
            map_yaml_path = Path(mpc_pkg_path) / Path(mpc_config["map"]["yaml_path"])
            reference_path_csv_path = Path(mpc_pkg_path) / Path(
                mpc_config["reference_path"]["csv_path"]
            )
    except:
        aic_tools_path = get_package_share_directory("aic_tools")
        map_yaml_path = (
            Path(aic_tools_path) / "resources" / "map" / "occupancy_grid_map.yaml"
        )
        reference_path_csv_path = ""

    return (map_yaml_path, reference_path_csv_path)
