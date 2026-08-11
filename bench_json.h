#pragma once
#include <cstdint>
#include <string>
#include <vector>

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

// Parse the terrain string (10000 chars) from the JSON content.
// Returns the raw terrain string, or empty if not found.
std::string parse_terrain(const std::string& content);

// Parse all path_test records from the JSON content.
std::vector<PathTestRecord> parse_path_tests(const std::string& content);
