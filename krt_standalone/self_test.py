"""Exercise the native dependencies shipped in the standalone executable."""

from io import BytesIO
import json
from pathlib import Path
import platform
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rust_router'))

import grid_router
import numpy as np
import ortools
from ortools.sat.python import cp_model
import PIL
from PIL import Image, ImageFont
import scipy
from scipy.optimize import linear_sum_assignment
import shapely
from shapely.geometry import Polygon


def self_test():
    length, edges = grid_router.mst_l1(
        np.array([0., 3., 3.]), np.array([0., 0., 4.]),
    )
    if length != 7 or len(edges) != 2:
        raise RuntimeError('KRT_SELF_TEST_FAILED: Rust minimum spanning tree')

    rows, columns = linear_sum_assignment(np.array([[1, 3], [4, 2]]))
    if rows.tolist() != [0, 1] or columns.tolist() != [0, 1]:
        raise RuntimeError('KRT_SELF_TEST_FAILED: SciPy linear assignment')

    area = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)]).area
    if area != 4:
        raise RuntimeError('KRT_SELF_TEST_FAILED: Shapely polygon area')

    image_bytes = BytesIO()
    Image.new('RGB', (5, 5), (12, 34, 56)).save(image_bytes, format='PNG')
    image_bytes.seek(0)
    with Image.open(image_bytes) as image:
        image.load()
        if image.size != (5, 5) or image.getpixel((0, 0)) != (12, 34, 56):
            raise RuntimeError('KRT_SELF_TEST_FAILED: Pillow PNG round trip')
    font = ImageFont.load_default(size=12)
    if font.getbbox('KRT')[2] <= 0:
        raise RuntimeError('KRT_SELF_TEST_FAILED: Pillow font rendering')

    model = cp_model.CpModel()
    variable = model.new_int_var(0, 10, 'x')
    model.add(variable >= 4)
    model.minimize(variable)
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 5
    solver.parameters.log_search_progress = False
    status = solver.solve(model)
    if status != cp_model.OPTIMAL or solver.value(variable) != 4:
        raise RuntimeError('KRT_SELF_TEST_FAILED: OR-Tools CP-SAT optimization')

    return {
        'schema': 1,
        'status': 'passed',
        'python': platform.python_version(),
        'versions': {
            'grid_router': grid_router.__version__,
            'numpy': np.__version__,
            'scipy': scipy.__version__,
            'shapely': shapely.__version__,
            'pillow': PIL.__version__,
            'ortools': ortools.__version__,
        },
        'checks': {
            'rust_mst': {'length': length, 'edges': len(edges)},
            'scipy_assignment': {'rows': rows.tolist(), 'columns': columns.tolist()},
            'shapely_polygon': {'area': area},
            'pillow_png': {'size': [5, 5], 'pixel': [12, 34, 56]},
            'pillow_font': {'text': 'KRT', 'size': 12},
            'ortools_cp_sat': {'status': solver.status_name(status), 'x': solver.value(variable)},
        },
    }


if __name__ == '__main__':
    if len(sys.argv) != 1:
        raise SystemExit('self-test takes no arguments')
    print(json.dumps(self_test(), indent=2))
