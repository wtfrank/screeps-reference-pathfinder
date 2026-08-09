#include "pf.h"
#include <cstring>
#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>

extern "C" {

struct CSearchResult {
    uint32_t ops;
    uint32_t cost;
    uint32_t incomplete;
    uint32_t path_len;
    uint8_t path_x[1000];
    uint8_t path_y[1000];
};

static uint8_t g_bitpacked_terrain[2500];
static std::unique_ptr<screeps::path_finder_t> g_pf;

void load_terrain_c(const uint8_t* terrain_10000) {
    if (!g_pf) {
        g_pf = std::make_unique<screeps::path_finder_t>();
    }
    std::memset(g_bitpacked_terrain, 0, 2500);
    for (size_t i = 0; i < 10000; ++i) {
        uint8_t val = terrain_10000[i] & 0x03;
        g_bitpacked_terrain[i / 4] |= (val << ((i % 4) * 2));
    }
    screeps::map_position_t room_pos(0, 0);
    screeps::path_finder_t::load_terrain({ {room_pos, g_bitpacked_terrain} });
}

CSearchResult search_path_c(
    uint8_t ox, uint8_t oy,
    uint8_t gx, uint8_t gy, uint8_t range,
    uint32_t flee,
    uint32_t max_ops,
    double heuristic_weight
) {
    CSearchResult out{};
    try {
        if (!g_pf) {
            g_pf = std::make_unique<screeps::path_finder_t>();
        }
        screeps::map_position_t room_pos(0, 0);
        screeps::world_position_t origin(ox, oy);
        screeps::world_position_t goal_pos(gx, gy);
        screeps::goal_t goal(range, goal_pos);

        auto res = g_pf->search(origin, {goal}, nullptr, 2, 10, 1, max_ops, 0xffffffff, flee != 0, heuristic_weight);

        out.ops = res.ops;
        out.cost = res.cost;
        out.incomplete = res.incomplete ? 1 : 0;
        out.path_len = std::min<size_t>(res.path.size(), 1000);

        for (size_t i = 0; i < out.path_len; ++i) {
            out.path_x[i] = res.path[i].first;
            out.path_y[i] = res.path[i].second;
        }
    } catch (const std::exception& e) {
        std::cerr << "C++ Exception in search_path_c: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown C++ Exception in search_path_c" << std::endl;
    }

    return out;
}

}
