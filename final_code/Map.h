#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include "Config.h"

// ============================================================
//  MAP  —  hardcoded arena layout
//  Coordinate system: mm from base origin.
//  +X = right, +Y = forward (into arena).
// ============================================================

struct PointI { int16_t x, y; };

// ── Hole ────────────────────────────────────────────────────
struct Hole {
  uint8_t col, row;    // 0..8
  PointI centre;        // world coordinates (mm)
  bool lineNorth, lineSouth, lineEast, lineWest;  // connected by a line?
};

// ── Wall ────────────────────────────────────────────────────
struct WallSegment {
  PointI a, b;
};

// ── Tunnel ──────────────────────────────────────────────────
struct Tunnel {
  PointI arenaEntry;    // where you enter from arena side
  PointI baseExit;      // where you exit into base
  // TODO: add airlock door state, RFID tag ID for entry/exit
};

// ── Line Segment (grid line on floor) ───────────────────────
// Connects two adjacent hole centres.
struct GridLine {
  uint8_t aRow, aCol;  // hole A (0..8)
  uint8_t bRow, bCol;  // hole B — must be orthogonally adjacent
};

// ── Arena Grid ──────────────────────────────────────────────
// Hole (col, row) has centre at:
//   x = BASE_TO_ARENA_X + HOLE_SPACING_MM/2 + col * HOLE_SPACING_MM
//   y = BASE_TO_ARENA_Y + HOLE_SPACING_MM/2 + row * HOLE_SPACING_MM
// With HOLE_SPACING_MM = 250, the grid spans 2000mm of the 2500mm arena,
// leaving 250mm margin all round.

class ArenaMap {
public:
  Hole holes[GRID_HOLES][GRID_HOLES];

  ArenaMap() {
    const float ox = BASE_TO_ARENA_X + HOLE_SPACING_MM / 2.0;
    const float oy = BASE_TO_ARENA_Y + HOLE_SPACING_MM / 2.0;

    for (uint8_t r = 0; r < GRID_HOLES; r++) {
      for (uint8_t c = 0; c < GRID_HOLES; c++) {
        holes[r][c].col = c;
        holes[r][c].row = r;
        holes[r][c].centre.x = (int16_t)(ox + c * HOLE_SPACING_MM);
        holes[r][c].centre.y = (int16_t)(oy + r * HOLE_SPACING_MM);
        holes[r][c].lineNorth = false;
        holes[r][c].lineSouth = false;
        holes[r][c].lineEast  = false;
        holes[r][c].lineWest  = false;
      }
    }

    // ── Grid lines: repeatable pattern connecting half the holes ──
    // TODO: confirm exact arena pattern. This example connects all
    // holes in odd rows horizontally and all holes in odd columns
    // vertically, forming 2×2 blocks.
    for (uint8_t r = 0; r < GRID_HOLES; r++) {
      for (uint8_t c = 0; c < GRID_HOLES; c++) {
        if (c < GRID_HOLES - 1 && (r % 2 == 1)) {
          holes[r][c].lineEast = true;
          holes[r][c + 1].lineWest = true;
        }
        if (r < GRID_HOLES - 1 && (c % 2 == 1)) {
          holes[r][c].lineSouth = true;
          holes[r + 1][c].lineNorth = true;
        }
      }
    }
  }

  // ── Convert RFID tag data to hole coordinates ──────────────
  // TODO: determine actual tag format once arena is available.
  // Options: "row,col" string, hex UID lookup, arena mm coords.
  // Returns true on success, fills (r, c).
  bool rfidToHole(const char* tagData, uint8_t& outRow, uint8_t& outCol) const {
    // Attempt to parse "row,col" or "row col"
    int r = -1, c = -1;
    if (sscanf(tagData, "%d,%d", &r, &c) >= 2 ||
        sscanf(tagData, "%d %d", &r, &c) >= 2) {
      if (r >= 0 && r < GRID_HOLES && c >= 0 && c < GRID_HOLES) {
        outRow = (uint8_t)r;
        outCol = (uint8_t)c;
        return true;
      }
    }
    return false;
  }

  // ── Hole world coordinate from grid index ──────────────────
  PointI holeCentre(uint8_t row, uint8_t col) const {
    if (row >= GRID_HOLES || col >= GRID_HOLES)
      return {0, 0};
    return holes[row][col].centre;
  }

  // ── Find nearest hole centre to a given pose ───────────────
  // Returns hole index, or -1 if further than HOLE_SPACING_MM.
  int nearestHole(int16_t x, int16_t y) const {
    int best = -1;
    int32_t bestDist2 = (int32_t)HOLE_SPACING_MM * HOLE_SPACING_MM;
    for (uint8_t r = 0; r < GRID_HOLES; r++) {
      for (uint8_t c = 0; c < GRID_HOLES; c++) {
        int32_t dx = x - holes[r][c].centre.x;
        int32_t dy = y - holes[r][c].centre.y;
        int32_t d2 = dx * dx + dy * dy;
        if (d2 < bestDist2) {
          bestDist2 = d2;
          best = r * GRID_HOLES + c;
        }
      }
    }
    return best;
  }
};

// ── Pre-defined Wall Segments ───────────────────────────────
// TODO: measure and add arena walls, tunnel walls, base walls
// const WallSegment ARENA_WALLS[] = { ... };
// const int ARENA_WALL_COUNT = 0;

// ── Tunnels ─────────────────────────────────────────────────
// TODO: measure tunnel entry/exit positions once arena is built
// const Tunnel TUNNELS[] = { ... };
// const int TUNNEL_COUNT = 0;

#endif
