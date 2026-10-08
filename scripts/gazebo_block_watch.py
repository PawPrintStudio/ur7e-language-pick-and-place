#!/usr/bin/env python3
"""Watch pick_object in Gazebo and record where it went; the rehearsal's ground truth.

The pick log only says what the software believes. This reads Gazebo's own
pose stream and writes, on SIGTERM/SIGINT, a JSON file with the block's
highest z and its final position, so the rehearsal can check that the block
really was lifted and really ended up where it should.

    python3 scripts/gazebo_block_watch.py OUT.json      # stop with Ctrl+C / kill
"""
import json
import re
import signal
import subprocess
import sys

TOPIC = '/world/pick_place_table/dynamic_pose/info'
PATTERN = re.compile(r'name: "pick_object".*?position \{\s*x: ([-\d.e]+)\s*'
                     r'y: ([-\d.e]+)\s*z: ([-\d.e]+)', re.S)


def main():
    out = sys.argv[1]
    record = {'samples': 0, 'max_z': None, 'final': None}

    def write(*_):
        with open(out, 'w') as stream:
            json.dump(record, stream)
        sys.exit(0)

    signal.signal(signal.SIGTERM, write)
    signal.signal(signal.SIGINT, write)
    stream = subprocess.Popen(['ign', 'topic', '-e', '-t', TOPIC], stdout=subprocess.PIPE,
                              text=True, bufsize=1)
    buffer = ''
    for line in stream.stdout:
        buffer += line
        if line.startswith('}') and 'pick_object' in buffer:   # end of one message
            match = PATTERN.search(buffer)
            if match:
                x, y, z = (float(v) for v in match.groups())
                record['samples'] += 1
                record['final'] = [x, y, z]
                record['max_z'] = z if record['max_z'] is None else max(record['max_z'], z)
            buffer = ''
        elif line.startswith('}'):
            buffer = ''
    write()


if __name__ == '__main__':
    main()
