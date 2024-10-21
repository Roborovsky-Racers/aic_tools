
from pathlib import Path
import pandas as pd
import numpy as np
from os import path
import yaml

import matplotlib.image as mpimg


def load_reference_path(reference_path_csv_path: str):
    reference_path_df = pd.read_csv(reference_path_csv_path)
    return reference_path_df


def load_occupancy_grid_map(map_yaml_path: str):
    base_path = path.dirname(map_yaml_path)

    with open(map_yaml_path, "r") as f:
        map_data = yaml.safe_load(f)

    pgm_file_path = path.join(base_path, map_data["image"])
    image = mpimg.imread(pgm_file_path)
    image_array = np.array(image)
    map_data["size"] = [image_array.shape[1], image_array.shape[0]]
    map_data["image_array"] = image_array

    return map_data