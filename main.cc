// Standalone benchmark runner for C++ pf.cc
#include "pf.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>

// Simple JSON parser for PATH_TEST records
struct PathTestRecord {
    uint32_t tick;
    uint32_t sample;
    uint8_t ox, oy;
    uint8_t gx, gy;
    uint8_t range;
    bool flee;
    uint32_t ops;
    uint32_t cost;
    bool incomplete;
    std::vector<std::pair<uint8_t, uint8_t>> path;
};

int main(int argc, char** argv) {
    std::string json_path = "../arena_api_mock/mock-screeps-arena/tests/data/path_tests_ssb5.json";
    if (argc > 1) {
        json_path = argv[1];
    }

    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open benchmark file: " << json_path << std::endl;
        return 1;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    size_t terr_key = content.find("\"terrain\"");
    if (terr_key == std::string::npos) {
        std::cerr << "Could not find terrain in JSON" << std::endl;
        return 1;
    }
    size_t terr_pos = content.find("\"", terr_key + 9);
    size_t terr_end = content.find("\"", terr_pos + 1);
    std::string terrain_str = content.substr(terr_pos + 1, terr_end - terr_pos - 1);

    if (terrain_str.length() < 10000) {
        std::cerr << "Invalid terrain length: " << terrain_str.length() << std::endl;
        return 1;
    }

    // Prepare 2,500 packed terrain bytes (2 bits per tile).
    // Terrain string is row-major: str[y*100+x] = terrain at (x,y).
    // pf.h look() reads column-major: packed[x*100+y] = terrain at (x,y).
    // Transpose here so the two align.
    std::vector<uint8_t> terrain_bytes(2500, 0);
    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 100; ++x) {
            char c = terrain_str[y * 100 + x];
            uint8_t val = (c == '1') ? 1 : ((c == '2') ? 2 : ((c == '3') ? 3 : 0));
            int col_idx = x * 100 + y;
            terrain_bytes[col_idx / 4] |= (val << ((col_idx % 4) * 2));
        }
    }

    // Load terrain into C++ pathfinder
    screeps::map_position_t room_pos(0, 0);
    screeps::path_finder_t::load_terrain({ {room_pos, terrain_bytes.data()} });

    auto pf = std::make_unique<screeps::path_finder_t>();

    std::cout << "Successfully loaded terrain. Parsing path_tests..." << std::endl;

    // Parse [PATH_TEST] queries using string search
    std::vector<PathTestRecord> tests;
    size_t test_pos = content.find("\"path_tests\":");
    if (test_pos == std::string::npos) test_pos = 0;

    while ((test_pos = content.find("\"origin\":", test_pos)) != std::string::npos) {
        // Find block start by looking backward for "tick" (always first key in each record),
        // then the { immediately before it. This avoids landing inside {"x":...} path waypoints
        // from the previous record when the whitespace-specific pattern doesn't match.
        size_t tick_pos = content.rfind("\"tick\"", test_pos);
        if (tick_pos == std::string::npos) break;
        size_t block_start = content.rfind("{", tick_pos);
        if (block_start == std::string::npos) break;

        // Find matching closing brace for this test block
        size_t block_end = block_start;
        int depth = 0;
        for (size_t p = block_start; p < content.length(); ++p) {
            if (content[p] == '{') depth++;
            else if (content[p] == '}') {
                depth--;
                if (depth == 0) { block_end = p; break; }
            }
        }

        std::string block = content.substr(block_start, block_end - block_start + 1);

        PathTestRecord r{};
        auto get_val = [&](const std::string& key) -> long {
            size_t k = block.find("\"" + key + "\":");
            if (k == std::string::npos) return 0;
            size_t v_start = block.find_first_of("0123456789truefalse", k + key.length() + 3);
            size_t v_end = block.find_first_of(",}\n", v_start);
            std::string s = block.substr(v_start, v_end - v_start);
            if (s == "true") return 1;
            if (s == "false") return 0;
            return std::stol(s);
        };

        auto get_nested_val = [&](const std::string& parent, const std::string& key) -> long {
            size_t p = block.find("\"" + parent + "\":");
            if (p == std::string::npos) return 0;
            size_t k = block.find("\"" + key + "\":", p);
            if (k == std::string::npos) return 0;
            size_t v_start = block.find_first_of("0123456789", k + key.length() + 3);
            size_t v_end = block.find_first_of(",}\n", v_start);
            return std::stol(block.substr(v_start, v_end - v_start));
        };

        r.ox = (uint8_t)get_nested_val("origin", "x");
        r.oy = (uint8_t)get_nested_val("origin", "y");
        r.gx = (uint8_t)get_nested_val("goal", "x");
        r.gy = (uint8_t)get_nested_val("goal", "y");
        r.range = (uint8_t)get_val("range");
        r.flee = (bool)get_val("flee");
        r.ops = (uint32_t)get_val("ops");
        r.cost = (uint32_t)get_val("cost");
        r.incomplete = (bool)get_val("incomplete");

        // Parse path array: find "[" after "path":, then scan to matching "]",
        // collecting {x,y} pairs along the way.
        size_t path_k = block.find("\"path\":");
        if (path_k != std::string::npos) {
            size_t arr_start = block.find("[", path_k);
            if (arr_start != std::string::npos) {
                // Find matching "]" using bracket depth
                size_t arr_end = arr_start;
                int adepth = 0;
                for (size_t p = arr_start; p < block.size(); ++p) {
                    if (block[p] == '[') adepth++;
                    else if (block[p] == ']') { if (--adepth == 0) { arr_end = p; break; } }
                }
                // Parse each {"x":N,"y":M} waypoint
                size_t scan = arr_start;
                while (scan < arr_end) {
                    size_t xk = block.find("\"x\":", scan);
                    if (xk == std::string::npos || xk >= arr_end) break;
                    size_t xv = block.find_first_of("0123456789", xk + 4);
                    size_t xve = block.find_first_of(",}", xv);
                    size_t yk = block.find("\"y\":", xk);
                    if (yk == std::string::npos || yk >= arr_end) break;
                    size_t yv = block.find_first_of("0123456789", yk + 4);
                    size_t yve = block.find_first_of(",}", yv);
                    uint8_t px = (uint8_t)std::stol(block.substr(xv, xve - xv));
                    uint8_t py = (uint8_t)std::stol(block.substr(yv, yve - yv));
                    r.path.emplace_back(px, py);
                    scan = yve + 1;
                }
            }
        }

        tests.push_back(r);
        test_pos = block_end + 1;
    }


    std::cout << "Parsed " << tests.size() << " benchmark queries. Running C++ pf.cc..." << std::endl;

    uint32_t matched = 0;
    uint32_t diffs = 0;
    struct CostDiffRecord {
        size_t idx;
        uint8_t ox, oy, gx, gy, range;
        bool flee;
        uint32_t ref_cost, cpp_cost;
    };
    std::vector<CostDiffRecord> cost_mismatches;
    uint32_t inc_mismatches = 0;
    uint32_t len_mismatches = 0;
    uint32_t waypoint_diffs = 0;

    for (size_t i = 0; i < tests.size(); ++i) {
        const auto& t = tests[i];
        screeps::world_position_t origin(t.ox, t.oy);
        screeps::world_position_t goal_pos(t.gx, t.gy);
        screeps::goal_t goal(t.range, goal_pos);

        auto res = pf->search(origin, {goal}, nullptr, 2, 10, 1, 10000, 0xffffffff, t.flee, 1.2);

        bool cpp_inc = res.incomplete;
        uint32_t cpp_cost = res.cost;

        uint32_t exp_ref_cost = (t.incomplete && t.cost == 4294967295) ? 0 : t.cost;
        uint32_t exp_cpp_cost = (cpp_inc && cpp_cost == 4294967295) ? 0 : cpp_cost;

        bool path_matches = (res.path.size() == t.path.size());
        if (path_matches) {
            for (size_t k = 0; k < res.path.size(); ++k) {
                if (res.path[k].first != t.path[k].first || res.path[k].second != t.path[k].second) {
                    path_matches = false;
                    break;
                }
            }
        }

        if (path_matches && cpp_inc == t.incomplete && exp_cpp_cost == exp_ref_cost) {
            matched++;
        } else {
            diffs++;
            if (cpp_inc != t.incomplete) {
                inc_mismatches++;
            } else if (exp_cpp_cost != exp_ref_cost) {
                cost_mismatches.push_back({i, t.ox, t.oy, t.gx, t.gy, t.range, t.flee, exp_ref_cost, exp_cpp_cost});
            } else if (res.path.size() != t.path.size()) {
                len_mismatches++;
                if (len_mismatches <= 8) {
                    std::cout << "  LenMismatch #" << i << ": origin=(" << (int)t.ox << "," << (int)t.oy
                              << ") goal=(" << (int)t.gx << "," << (int)t.gy << ") range=" << (int)t.range
                              << " flee=" << t.flee << " cost=" << exp_ref_cost
                              << " | REF path=" << t.path.size() << " CPP path=" << res.path.size() << std::endl;
                }
            } else {
                waypoint_diffs++;
            }
        }
    }

    std::cout << "\n=== STANDALONE C++ pf.cc vs OFFICIAL SERVER LOGS (Out of " << tests.size() << " Queries) ===" << std::endl;
    std::cout << "  100% Exact Match           : " << matched << " (" << (matched * 100.0 / tests.size()) << "%)" << std::endl;
    std::cout << "  Incomplete Flag Mismatches : " << inc_mismatches << std::endl;
    std::cout << "  Cost Mismatches            : " << cost_mismatches.size() << std::endl;
    std::cout << "  Path Length Mismatches     : " << len_mismatches << std::endl;
    std::cout << "  Same Cost/Len Waypoint Diff: " << waypoint_diffs << std::endl;

    if (!cost_mismatches.empty()) {
        std::cout << "\n--- C++ pf.cc Cost Mismatches ---" << std::endl;
        size_t limit = std::min<size_t>(15, cost_mismatches.size());
        for (size_t k = 0; k < limit; ++k) {
            const auto& m = cost_mismatches[k];
            std::cout << "  Query #" << m.idx << ": origin=(" << (int)m.ox << "," << (int)m.oy
                      << ") goal=(" << (int)m.gx << "," << (int)m.gy << ") range=" << (int)m.range
                      << " flee=" << m.flee << " | REF cost=" << m.ref_cost << " | CPP cost=" << m.cpp_cost << std::endl;
        }
    }

    return 0;
}
