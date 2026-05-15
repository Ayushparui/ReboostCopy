#!/usr/bin/env python3

# This is a utility script.
# DO NOT CALL IT DIRECTLY - unless you REALLY know what you are doing.
# It is called from enableRoboEireann.

import sys
import os
import shutil

script = os.path.basename(sys.argv[0])

mode, binary_path, backup_path, old_str, new_str = sys.argv[1:]

old = old_str.encode()
new = new_str.encode()

if len(new) > len(old):
    print(f"{script}: ERROR: replacement string must not be longer than original")
    sys.exit(1)

if mode == "restore":
    try:
        shutil.copy2(backup_path, binary_path)
        print(f"{script}: Restored {binary_path} from {backup_path}")
    except OSError as e:
        print(f"{script}: ERROR: failed to restore {binary_path} from {backup_path}: {e}")
        sys.exit(1)
    sys.exit(0)

# mode == "patch"
data = open(binary_path, "rb").read()

if old not in data:
    print(f"{script}: WARNING: {old!r} not found in {binary_path}, skipping")
    sys.exit(0)

try:
    shutil.copy2(binary_path, backup_path)
    print(f"{script}: Backup created: {backup_path}")
except OSError as e:
    print(f"{script}: ERROR: failed to create backup {backup_path}: {e}")
    sys.exit(1)

patched = data.replace(old, new.ljust(len(old), b"\x00"))
open(binary_path, "wb").write(patched)
print(f"{script}: Patched {binary_path}: {old!r} --> {new!r}")