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

    // Prepare 10,000 terrain bytes
    std::vector<uint8_t> terrain_bytes(10000);
    for (size_t i = 0; i < 10000; ++i) {
        char c = terrain_str[i];
        if (c == '1') terrain_bytes[i] = 1;
        else if (c == '2') terrain_bytes[i] = 2;
        else if (c == '3') terrain_bytes[i] = 3;
        else terrain_bytes[i] = 0;
    }

    // Load terrain into C++ pathfinder
    screeps::map_position_t room_pos(0, 0);
    screeps::path_finder_t::load_terrain({ {room_pos, terrain_bytes.data()} });

    screeps::path_finder_t pf;

    std::cout << "Successfully loaded terrain. Parsing path_tests..." << std::endl;

    // Parse [PATH_TEST] queries using string search
    std::vector<PathTestRecord> tests;
    size_t test_pos = 0;
    while ((test_pos = content.find("\"ox\":", test_pos)) != std::string::npos) {
        size_t block_start = content.rfind("{", test_pos);
        size_t block_end = content.find("}", test_pos);
        if (block_start == std::string::npos || block_end == std::string::npos) break;

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

        r.ox = (uint8_t)get_val("ox");
        r.oy = (uint8_t)get_val("oy");
        r.gx = (uint8_t)get_val("gx");
        r.gy = (uint8_t)get_val("gy");
        r.range = (uint8_t)get_val("range");
        r.flee = (bool)get_val("flee");
        r.ops = (uint32_t)get_val("ops");
        r.cost = (uint32_t)get_val("cost");
        r.incomplete = (bool)get_val("inc");

        // Parse path array length
        size_t path_k = block.find("\"path\":");
        if (path_k != std::string::npos) {
            size_t p_start = block.find("[", path_k);
            size_t p_end = block.find("]", p_start);
            std::string p_str = block.substr(p_start, p_end - p_start + 1);
            size_t count = 0;
            size_t pos = 0;
            while ((pos = p_str.find("{\"x\":", pos)) != std::string::npos) {
                count++;
                pos++;
            }
            r.path.resize(count);
        }

        tests.push_back(r);
        test_pos = block_end + 1;
    }

    std::cout << "Parsed " << tests.size() << " benchmark queries. Running C++ pf.cc..." << std::endl;

    uint32_t matched = 0;
    uint32_t diffs = 0;
    uint32_t cost_diffs = 0;
    uint32_t inc_diffs = 0;

    for (size_t i = 0; i < tests.size(); ++i) {
        const auto& t = tests[i];
        screeps::world_position_t origin(t.ox, t.oy);
        screeps::world_position_t goal_pos(t.gx, t.gy);
        screeps::goal_t goal(t.range, goal_pos);

        auto res = pf.search(origin, {goal}, nullptr, 2, 10, 1, 50000, 0xffffffff, t.flee, 1.2);

        uint32_t exp_cost = (t.incomplete && t.cost == 4294967295) ? 0 : t.cost;
        uint32_t res_cost = (res.incomplete && res.cost == 4294967295) ? 0 : res.cost;

        if (res.incomplete == t.incomplete && res_cost == exp_cost && res.path.size() == t.path.size()) {
            matched++;
        } else {
            diffs++;
            if (res.incomplete != t.incomplete) inc_diffs++;
            if (res_cost != exp_cost) {
                cost_diffs++;
                if (cost_diffs <= 10) {
                    std::cout << "C++ COST DIFF #" << cost_diffs << ": ox=" << (int)t.ox << " oy=" << (int)t.oy
                              << " gx=" << (int)t.gx << " gy=" << (int)t.gy << " range=" << (int)t.range
                              << " flee=" << t.flee << " | REF cost=" << exp_cost << " inc=" << t.incomplete
                              << " | CPP cost=" << res_cost << " inc=" << res.incomplete << std::endl;
                }
            }
        }
    }

    std::cout << "\n=== C++ pf.cc BENCHMARK RESULTS (Out of " << tests.size() << " Queries) ===" << std::endl;
    std::cout << "  100% Exact Match           : " << matched << " (" << (matched * 100.0 / tests.size()) << "%)" << std::endl;
    std::cout << "  Incomplete Mismatches      : " << inc_diffs << std::endl;
    std::cout << "  Cost Mismatches            : " << cost_diffs << std::endl;
    std::cout << "  Total Diffs                : " << diffs << std::endl;

    return 0;
}
