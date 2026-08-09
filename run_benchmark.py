#!/usr/bin/env python3
"""
Python benchmark harness for compiled C++ pf.cc library (libpf.so).
Compares official JS server log reference output against standalone compiled C++ pf.cc.
"""

import ctypes
import json
import sys
from pathlib import Path


class CSearchResult(ctypes.Structure):
    _fields_ = [
        ("ops", ctypes.c_uint32),
        ("cost", ctypes.c_uint32),
        ("incomplete", ctypes.c_uint32),
        ("path_len", ctypes.c_uint32),
        ("path_x", ctypes.c_uint8 * 1000),
        ("path_y", ctypes.c_uint8 * 1000),
    ]


def main():
    lib_path = Path(__file__).parent / "build" / "libpf.so"
    if not lib_path.exists():
        print(f"Error: Shared library '{lib_path}' not found. Run 'make' in pf-reference.", file=sys.stderr)
        sys.exit(1)

    lib = ctypes.CDLL(str(lib_path))

    lib.load_terrain_c.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
    lib.load_terrain_c.restype = None

    lib.search_path_c.argtypes = [
        ctypes.c_uint8,  # ox
        ctypes.c_uint8,  # oy
        ctypes.c_uint8,  # gx
        ctypes.c_uint8,  # gy
        ctypes.c_uint8,  # range
        ctypes.c_uint32, # flee
        ctypes.c_uint32, # max_ops
        ctypes.c_double  # heuristic_weight
    ]
    lib.search_path_c.restype = CSearchResult

    # Find benchmark JSON
    json_path = Path(__file__).parent.parent / "arena_api_mock" / "mock-screeps-arena" / "tests" / "data" / "path_tests_ssb5.json"
    if not json_path.exists():
        json_path = Path(__file__).parent.parent / "screeps_arena_sim" / "path_tests_ssb5-debug-paths.json"

    print(f"Loading benchmark JSON: {json_path}")
    data = json.load(open(json_path))

    terrain_str = data["terrain"]
    terrain_bytes = (ctypes.c_uint8 * 10000)()
    for i, c in enumerate(terrain_str):
        terrain_bytes[i] = int(c)

    lib.load_terrain_c(terrain_bytes)
    print("Loaded 10,000 terrain bytes into C++ pf.cc.")

    path_tests = data["path_tests"]
    print(f"Running {len(path_tests)} benchmark queries through C++ pf.cc...")

    matched = 0
    diffs = 0
    cost_mismatches = []
    inc_mismatches = []
    len_mismatches = []
    waypoint_diffs = 0

    for idx, test in enumerate(path_tests):
        ox = test["origin"]["x"]
        oy = test["origin"]["y"]
        gx = test["goal"]["x"]
        gy = test["goal"]["y"]
        r = test["range"]
        flee = 1 if test["flee"] else 0

        ref_ops = test["ops"]
        ref_cost = test["cost"]
        ref_inc = test["incomplete"]
        ref_path = test["path"]

        res = lib.search_path_c(ox, oy, gx, gy, r, flee, 50000, 1.2)

        cpp_inc = (res.incomplete != 0)
        cpp_cost = res.cost
        cpp_path_len = res.path_len

        exp_ref_cost = 0 if (ref_inc and ref_cost == 4294967295) else ref_cost
        exp_cpp_cost = 0 if (cpp_inc and cpp_cost == 4294967295) else cpp_cost

        # Compare path waypoints
        path_matches = True
        if cpp_path_len != len(ref_path):
            path_matches = False
        else:
            for i in range(cpp_path_len):
                if res.path_x[i] != ref_path[i]["x"] or res.path_y[i] != ref_path[i]["y"]:
                    path_matches = False
                    break

        if path_matches and cpp_inc == ref_inc and exp_cpp_cost == exp_ref_cost:
            matched += 1
        else:
            diffs += 1
            if cpp_inc != ref_inc:
                inc_mismatches.append((idx, ox, oy, gx, gy, r, flee, ref_inc, cpp_inc))
            elif exp_cpp_cost != exp_ref_cost:
                cost_mismatches.append((idx, ox, oy, gx, gy, r, flee, exp_ref_cost, exp_cpp_cost))
            elif cpp_path_len != len(ref_path):
                len_mismatches.append((idx, ox, oy, gx, gy, r, flee, len(ref_path), cpp_path_len))
            else:
                waypoint_diffs += 1

        if (idx + 1) % 500 == 0:
            print(f"Processed {idx + 1}/{len(path_tests)} queries...", flush=True)

    print(f"\n=== STANDALONE C++ pf.cc vs OFFICIAL SERVER LOGS (Out of {len(path_tests)} Queries) ===", flush=True)
    print(f"  100% Exact Match           : {matched} ({matched / len(path_tests) * 100.0:.1f}%)", flush=True)
    print(f"  Incomplete Flag Mismatches : {len(inc_mismatches)}", flush=True)
    print(f"  Cost Mismatches            : {len(cost_mismatches)}", flush=True)
    print(f"  Path Length Mismatches     : {len(len_mismatches)}", flush=True)
    print(f"  Same Cost/Len Waypoint Diff: {waypoint_diffs}", flush=True)

    if cost_mismatches:
        print("\n--- C++ pf.cc Cost Mismatches ---")
        for m in cost_mismatches[:15]:
            print(f"  Query #{m[0]}: origin=({m[1]},{m[2]}) goal=({m[3]},{m[4]}) range={m[5]} flee={m[6]} | REF cost={m[7]} | CPP cost={m[8]}")

if __name__ == "__main__":
    main()
