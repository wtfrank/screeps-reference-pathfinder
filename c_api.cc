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

static uint8_t g_terrain_bytes[2500];
static std::unique_ptr<screeps::path_finder_t> g_pf;

void load_terrain_c(const uint8_t* terrain_10000) {
    if (!g_pf) {
        g_pf = std::make_unique<screeps::path_finder_t>();
    }
    std::memset(g_terrain_bytes, 0, sizeof(g_terrain_bytes));
    // Input is row-major: terrain_10000[y*100+x] = terrain at (x,y).
    // pf.h look() reads column-major: packed[x*100+y] = terrain at (x,y).
    // Transpose here so the two align.
    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 100; ++x) {
            uint8_t val = terrain_10000[y * 100 + x] & 0x03;
            int col_idx = x * 100 + y;
            g_terrain_bytes[col_idx / 4] |= (val << ((col_idx % 4) * 2));
        }
    }
    screeps::map_position_t room_pos(0, 0);
    screeps::path_finder_t::load_terrain({ {room_pos, g_terrain_bytes} });
}

// Mirrors the JS searchPath() wrapper in game.path-finder.search-path.js:
//   - origin bounds check
//   - parameter clamping matching JS Math.min/max defaults
//   - path reversed to origin→goal order (JS does ret.path.reverse())
CSearchResult search_path_c(
    uint8_t ox, uint8_t oy,
    uint8_t gx, uint8_t gy, uint8_t range,
    uint32_t plain_cost,
    uint32_t swamp_cost,
    uint32_t flee,
    uint32_t max_ops,
    uint32_t max_cost,
    double heuristic_weight
) {
    CSearchResult out{};

    // JS: origin out-of-bounds → {path:[], ops:0, cost:0, incomplete:false}
    // arenaSize for 100x100 is 99 (0-based).
    const uint8_t arena_size = 99;
    if (ox > arena_size || oy > arena_size) {
        return out;
    }

    // Clamp options to match JS Math.min/max behaviour:
    //   plainCost:       clamp [1, 254], default 2
    //   swampCost:       clamp [1, 254], default 10
    //   heuristicWeight: clamp [1, 9],   default 1.2
    //   maxOps:          clamp [1, ∞),   default 10000
    //   maxCost:         clamp [1, ∞),   default 0xffffffff
    uint32_t pc = std::min(254u, std::max(1u, plain_cost  ? plain_cost  : 2u));
    uint32_t sc = std::min(254u, std::max(1u, swamp_cost  ? swamp_cost  : 10u));
    double   hw = std::min(9.0,  std::max(1.0, heuristic_weight != 0.0 ? heuristic_weight : 1.2));
    uint32_t mo = std::max(1u,   max_ops  ? max_ops  : 10000u);
    uint32_t mc = std::max(1u,   max_cost ? max_cost : 0xffffffffu);

    try {
        if (!g_pf) {
            g_pf = std::make_unique<screeps::path_finder_t>();
        }
        screeps::world_position_t origin(ox, oy);
        screeps::world_position_t goal_pos(gx, gy);
        screeps::goal_t goal(range, goal_pos);

        auto res = g_pf->search(origin, {goal}, nullptr, pc, sc, 1, mo, mc, flee != 0, hw);

        out.ops        = res.ops;
        out.cost       = res.cost;
        out.incomplete = res.incomplete ? 1 : 0;
        out.path_len   = std::min<size_t>(res.path.size(), 1000);

        // JS: ret.path.reverse() — native pf.cc builds path goal→origin,
        // JS reverses it to origin→goal before returning.
        for (size_t i = 0; i < out.path_len; ++i) {
            size_t rev = out.path_len - 1 - i;
            out.path_x[i] = res.path[rev].first;
            out.path_y[i] = res.path[rev].second;
        }
    } catch (const std::exception& e) {
        std::cerr << "C++ Exception in search_path_c: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown C++ Exception in search_path_c" << std::endl;
    }

    return out;
}

}
