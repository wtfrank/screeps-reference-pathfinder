#include "bench_json.h"
#include <string>
#include <vector>

std::string parse_terrain(const std::string& content) {
    size_t key = content.find("\"terrain\"");
    if (key == std::string::npos) return {};
    size_t q1 = content.find('"', key + 9);
    size_t q2 = content.find('"', q1 + 1);
    return content.substr(q1 + 1, q2 - q1 - 1);
}

std::vector<PathTestRecord> parse_path_tests(const std::string& content) {
    std::vector<PathTestRecord> tests;

    size_t test_pos = content.find("\"path_tests\":");
    if (test_pos == std::string::npos) test_pos = 0;

    while ((test_pos = content.find("\"origin\":", test_pos)) != std::string::npos) {
        // Find block start: search backward for "tick" (always the first key in each record),
        // then the '{' immediately before it. This avoids landing inside {"x":...} path
        // waypoints from the previous record when a whitespace-specific pattern doesn't match.
        size_t tick_pos = content.rfind("\"tick\"", test_pos);
        if (tick_pos == std::string::npos) break;
        size_t block_start = content.rfind('{', tick_pos);
        if (block_start == std::string::npos) break;

        // Find matching '}' for the record block.
        size_t block_end = block_start;
        int depth = 0;
        for (size_t p = block_start; p < content.size(); ++p) {
            if (content[p] == '{') ++depth;
            else if (content[p] == '}') { if (--depth == 0) { block_end = p; break; } }
        }

        const std::string block = content.substr(block_start, block_end - block_start + 1);

        // Read a scalar value by key name (handles integers, true/false).
        auto get_val = [&](const std::string& key) -> long {
            size_t k = block.find('"' + key + "\":");
            if (k == std::string::npos) return 0;
            size_t vs = block.find_first_of("0123456789truefalse", k + key.size() + 3);
            size_t ve = block.find_first_of(",}\n", vs);
            std::string s = block.substr(vs, ve - vs);
            if (s == "true")  return 1;
            if (s == "false") return 0;
            return std::stol(s);
        };

        // Read a numeric value nested inside a named object (e.g. "origin": {"x": N}).
        auto get_nested_val = [&](const std::string& parent, const std::string& key) -> long {
            size_t p = block.find('"' + parent + "\":");
            if (p == std::string::npos) return 0;
            size_t k = block.find('"' + key + "\":", p);
            if (k == std::string::npos) return 0;
            size_t vs = block.find_first_of("0123456789", k + key.size() + 3);
            size_t ve = block.find_first_of(",}\n", vs);
            return std::stol(block.substr(vs, ve - vs));
        };

        PathTestRecord r{};
        r.ox       = (uint8_t) get_nested_val("origin", "x");
        r.oy       = (uint8_t) get_nested_val("origin", "y");
        r.gx       = (uint8_t) get_nested_val("goal",   "x");
        r.gy       = (uint8_t) get_nested_val("goal",   "y");
        r.range    = (uint8_t) get_val("range");
        r.flee     = (bool)    get_val("flee");
        r.ops      = (uint32_t)get_val("ops");
        r.cost     = (uint32_t)get_val("cost");
        r.incomplete = (bool)  get_val("incomplete");

        // Parse the path array, collecting {x,y} waypoints.
        size_t path_k = block.find("\"path\":");
        if (path_k != std::string::npos) {
            size_t arr_start = block.find('[', path_k);
            if (arr_start != std::string::npos) {
                // Find matching ']' with bracket depth.
                size_t arr_end = arr_start;
                int adepth = 0;
                for (size_t p = arr_start; p < block.size(); ++p) {
                    if (block[p] == '[') ++adepth;
                    else if (block[p] == ']') { if (--adepth == 0) { arr_end = p; break; } }
                }
                // Parse each {"x":N,"y":M} entry.
                size_t scan = arr_start;
                while (scan < arr_end) {
                    size_t xk = block.find("\"x\":", scan);
                    if (xk == std::string::npos || xk >= arr_end) break;
                    size_t xv  = block.find_first_of("0123456789", xk + 4);
                    size_t xve = block.find_first_of(",}", xv);
                    size_t yk  = block.find("\"y\":", xk);
                    if (yk == std::string::npos || yk >= arr_end) break;
                    size_t yv  = block.find_first_of("0123456789", yk + 4);
                    size_t yve = block.find_first_of(",}", yv);
                    r.path.emplace_back(
                        (uint8_t)std::stol(block.substr(xv, xve - xv)),
                        (uint8_t)std::stol(block.substr(yv, yve - yv))
                    );
                    scan = yve + 1;
                }
            }
        }

        tests.push_back(std::move(r));
        test_pos = block_end + 1;
    }

    return tests;
}
