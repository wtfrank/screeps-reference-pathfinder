// Standalone benchmark runner for C++ pf.cc
#include "pf.h"
#include "bench_json.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>

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

    std::string terrain_str = parse_terrain(content);
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
    std::vector<PathTestRecord> tests = parse_path_tests(content);

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
