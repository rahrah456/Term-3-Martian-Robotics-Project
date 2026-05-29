#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include "Config.h"

// ============================================================
// DYNAMIC ARENA MAP
//
// Coordinate system is assigned dynamically on competition day.
//
// Server coordinates are:
//   x = 1..9
//   y = 1..9
//
// Internal coordinates remain:
//   row = 0..8
//   col = 0..8
//
// The first RFID tag detected is assigned the server-provided
// coordinate, and the whole grid rotates consistently.
// ============================================================

struct PointI {
    int16_t x;
    int16_t y;
};

class ArenaMap {

public:

    // ========================================================
    // Arena Geometry
    // ========================================================

    static constexpr float ARENA_WIDTH_MM  = 2400.0f;
    static constexpr float ARENA_DEPTH_MM = 2400.0f;

    static constexpr float HOLE_SPACING_MM = 250.0f;

    static constexpr int GRID_HOLES = 9;

    // ========================================================
    // Dynamic coordinate origin
    // ========================================================

    int8_t originRow = 0;
    int8_t originCol = 0;

    bool coordinateSystemInitialised = false;

    // ========================================================
    // Convert server coordinates (1..9) to internal (0..8)
    // ========================================================

    void serverToInternal(
        uint8_t serverY,
        uint8_t serverX,
        uint8_t& row,
        uint8_t& col
    ) const {

        row = serverY - 1;
        col = serverX - 1;
    }

    // ========================================================
    // Configure coordinate system from first detected RFID
    // ========================================================

    void setCoordinateOrigin(
        uint8_t detectedRow,
        uint8_t detectedCol
    ) {

        originRow = detectedRow;
        originCol = detectedCol;

        coordinateSystemInitialised = true;
    }

    // ========================================================
    // Convert logical grid coordinate -> physical grid index
    // ========================================================

    void logicalToPhysical(
        uint8_t logicalRow,
        uint8_t logicalCol,
        uint8_t& physicalRow,
        uint8_t& physicalCol
    ) const {

        physicalRow =
            (logicalRow - originRow + GRID_HOLES)
            % GRID_HOLES;

        physicalCol =
            (logicalCol - originCol + GRID_HOLES)
            % GRID_HOLES;
    }

    // ========================================================
    // Hole Centre
    // ========================================================

    PointI holeCentre(
        uint8_t logicalRow,
        uint8_t logicalCol
    ) const {

        uint8_t row;
        uint8_t col;

        logicalToPhysical(
            logicalRow,
            logicalCol,
            row,
            col
        );

        PointI p;

        // First hole centre = 250,250

        p.x = 250 + col * HOLE_SPACING_MM;
        p.y = 250 + row * HOLE_SPACING_MM;

        return p;
    }

    // ========================================================
    // Inside arena test
    // ========================================================

    bool isInsideArena(
        int16_t x,
        int16_t y
    ) const {

        return
            x >= 0 &&
            x <= ARENA_WIDTH_MM &&
            y >= 0 &&
            y <= ARENA_DEPTH_MM;
    }

    // ========================================================
    // RFID -> hole mapping
    // ========================================================

    bool rfidToHole(
        const char* tag,
        uint8_t& row,
        uint8_t& col
    ) const {

        // TODO:
        // Replace with real RFID mappings

        return false;
    }

    // ========================================================
    // Nearest hole
    // ========================================================

    int nearestHole(
        int16_t x,
        int16_t y
    ) const {

        float bestDist = 1e9;
        int bestIdx = -1;

        for (uint8_t r = 0; r < GRID_HOLES; r++) {

            for (uint8_t c = 0; c < GRID_HOLES; c++) {

                PointI p = holeCentre(r, c);

                float dx = p.x - x;
                float dy = p.y - y;

                float d2 = dx * dx + dy * dy;

                if (d2 < bestDist) {

                    bestDist = d2;
                    bestIdx = r * GRID_HOLES + c;
                }
            }
        }

        return bestIdx;
    }
};

// ============================================================
// Compatibility defines
// ============================================================

#define GRID_HOLES ArenaMap::GRID_HOLES
#define HOLE_SPACING_MM ArenaMap::HOLE_SPACING_MM

#endif
