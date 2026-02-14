#!/usr/bin/env python3
"""Test script to verify URDF loads correctly."""
import os
from ament_index_python.packages import get_package_share_directory

try:
    pkg_share_path = get_package_share_directory("me557_pen_description")
    urdf_path = os.path.join(pkg_share_path, "urdf", "Robot Assembly.SLDASM.urdf")
    
    print(f"Package share: {pkg_share_path}")
    print(f"URDF path: {urdf_path}")
    print(f"File exists: {os.path.exists(urdf_path)}")
    
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            content = f.read()
            print(f"URDF file size: {len(content)} bytes")
            print(f"First 200 chars: {content[:200]}")
            
            # Check for base_link
            if 'base_link' in content:
                print("✓ Found 'base_link' in URDF")
            else:
                print("✗ 'base_link' not found in URDF")
                
            # Check for mesh references
            mesh_count = content.count('package://me557_pen_description/meshes/')
            print(f"Found {mesh_count} mesh references")
            
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
