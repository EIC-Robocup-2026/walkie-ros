#!/usr/bin/env python3
import sys
import os
import shutil
import glob

BACKUP_DIR = "/home/armmy2530/Project/walkie_ws/walkie-ros-test-pixi/tools/collision_mesh_backup"
DESC_DIR = "/home/armmy2530/Project/walkie_ws/walkie-ros-test-pixi/description"

def switch(target):
    src_dir = os.path.join(BACKUP_DIR, target)
    if not os.path.exists(src_dir):
        print(f"Error: {src_dir} does not exist!")
        sys.exit(1)
    
    files = glob.glob(f"{src_dir}/**/*.stl", recursive=True)
    if not files:
        print(f"Error: No STL files found in {src_dir}!")
        sys.exit(1)

    count = 0
    total_bytes = 0
    for f in files:
        rel = os.path.relpath(f, src_dir)
        dst = os.path.join(DESC_DIR, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(f, dst)
        count += 1
        total_bytes += os.path.getsize(dst)

    print(f"Successfully switched to '{target}' model: copied {count} STL files ({total_bytes / 1024:.1f} KB total).")

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("original", "optimized"):
        print("Usage: python3 switch_collision_model.py <original|optimized>")
        sys.exit(1)
    switch(sys.argv[1])
