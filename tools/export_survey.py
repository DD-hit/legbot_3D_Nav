"""Fuse replayed FAST-LIO world scans into a binary XYZ PCD in the odom frame."""
import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import dtype_from_fields


def messages(directory, topic):
    files = sorted(Path(directory).glob('*.db3'), key=lambda file: int(file.stem.rsplit('_', 1)[1]))
    if not files:
        raise ValueError(f'No SQLite rosbag in {directory}')
    for file in files:
        connection = sqlite3.connect(f'{file.as_uri()}?mode=ro', uri=True)
        try:
            info = connection.execute('SELECT id,type FROM topics WHERE name=?', (topic,)).fetchone()
            if not info:
                raise ValueError(f'Missing {topic} in {file}')
            message_type = get_message(info[1])
            for stamp, blob in connection.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (info[0],)):
                yield stamp * 1e-9, deserialize_message(blob, message_type)
        finally:
            connection.close()


def sample(points, voxel):
    grid = np.floor(points / voxel).astype(np.int64) + 1048576
    if grid.min() < 0 or grid.max() >= 2097152:
        raise ValueError('Map extent exceeds voxel index range')
    keys = (grid[:, 0] << 42) | (grid[:, 1] << 21) | grid[:, 2]
    return points[np.unique(keys, return_index=True)[1]]


def write_cloud(file, points):
    header = f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
    with file.open('xb') as stream:
        stream.write(header.encode('ascii'))
        points.astype('<f4', copy=False).tofile(stream)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--topic', default='/fast_lio/cloud_registered')
    parser.add_argument('--voxel', type=float, default=.05)
    parser.add_argument('--from-time', type=float, default=-np.inf)
    parser.add_argument('--until-time', type=float, default=np.inf)
    args = parser.parse_args()
    if args.voxel <= 0 or args.from_time >= args.until_time:
        parser.error('Invalid voxel size or time interval')
    batch, pieces = [], []
    first = last = None
    count = 0
    for _, msg in messages(args.bag.resolve(), args.topic):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not args.from_time <= stamp < args.until_time:
            continue
        if msg.header.frame_id != 'odom' or msg.height != 1 or msg.is_bigendian:
            raise ValueError('Expected unorganized little-endian FAST-LIO scans in odom')
        data = np.frombuffer(msg.data, dtype=dtype_from_fields(msg.fields, point_step=msg.point_step), count=msg.width)
        xyz = np.column_stack([data[axis] for axis in 'xyz'])
        batch.append(xyz[np.isfinite(xyz).all(axis=1)])
        first = stamp if first is None else first
        last = stamp
        count += 1
        if len(batch) == 60:
            pieces.append(sample(np.concatenate(batch), args.voxel)); batch.clear()
            if len(pieces) == 16:
                pieces = [sample(np.concatenate(pieces), args.voxel)]
    if not count:
        raise ValueError('No registered scans in interval')
    if batch:
        pieces.append(sample(np.concatenate(batch), args.voxel))
    points = sample(np.concatenate(pieces), args.voxel)
    write_cloud(args.output, points)
    print(json.dumps(dict(scans=count, points=len(points), frame='odom', voxel_m=args.voxel, first=first, last=last)))


if __name__ == '__main__':
    main()
