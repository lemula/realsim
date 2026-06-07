import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import pylas


def convert_pcd_to_las(input_path: str, output_path: str) -> None:
    pcd = o3d.io.read_point_cloud(input_path)
    points = np.asarray(pcd.points)

    if points.size == 0:
        raise ValueError("输入的 PCD 文件不包含点数据。")

    las = pylas.create(point_format_id=3, file_version="1.2")
    las.header.point_data_format_id = 3
    las.header.point_data_record_length = 28 + 6 + 2

    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]

    if pcd.has_colors():
        colors = np.clip(np.asarray(pcd.colors) * 65535, 0, 65535).astype(np.uint16)
        las.red = colors[:, 0]
        las.green = colors[:, 1]
        las.blue = colors[:, 2]

    las.header.x_min = float(np.min(points[:, 0]))
    las.header.x_max = float(np.max(points[:, 0]))
    las.header.y_min = float(np.min(points[:, 1]))
    las.header.y_max = float(np.max(points[:, 1]))
    las.header.z_min = float(np.min(points[:, 2]))
    las.header.z_max = float(np.max(points[:, 2]))

    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    las.write(str(output_file))


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert PCD point cloud to LAS.")
    parser.add_argument("-i", "--input", required=True, help="输入 PCD 文件路径")
    parser.add_argument("-o", "--output", required=True, help="输出 LAS 文件路径")
    args = parser.parse_args()

    convert_pcd_to_las(args.input, args.output)


if __name__ == "__main__":
    main()