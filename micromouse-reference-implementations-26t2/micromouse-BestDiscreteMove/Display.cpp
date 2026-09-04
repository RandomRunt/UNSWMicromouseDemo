// AI-ASSISTED FILE: Framebuffer-free Task 4.3 renderer generated with OpenAI
// Codex (2026-08-11). AI-authored sections are identified inline.
#include "Display.hpp"

#include <Wire.h>
#include <avr/pgmspace.h>

namespace mtrn3100 {

namespace {

constexpr uint8_t DISPLAY_ADDRESS = 0x3C;
constexpr uint8_t DISPLAY_WIDTH = 128;
constexpr uint8_t DISPLAY_PAGES = 8;
constexpr uint8_t MAP_PITCH = 7;
constexpr uint8_t MAP_EXTENT = MAZE_COLUMNS * MAP_PITCH;
constexpr uint8_t FIRST_TEXT_PAGE = 1;
constexpr uint8_t GLYPH_COLUMNS = 5;
constexpr uint8_t GLYPH_SCALE = 2;
constexpr uint8_t GLYPH_SPACING = 2;
constexpr uint8_t TEXT_CHARACTERS = 5;
constexpr uint8_t TEXT_WIDTH = TEXT_CHARACTERS
                             * (GLYPH_COLUMNS * GLYPH_SCALE + GLYPH_SPACING);
constexpr uint8_t TEXT_COLUMN = (DISPLAY_WIDTH - TEXT_WIDTH) / 2;
constexpr uint8_t DATA_CHUNK_BYTES = 24;

enum Glyph : uint8_t {
    GLYPH_0,
    GLYPH_1,
    GLYPH_2,
    GLYPH_3,
    GLYPH_4,
    GLYPH_5,
    GLYPH_6,
    GLYPH_7,
    GLYPH_8,
    GLYPH_9,
    GLYPH_F,
    GLYPH_L,
    GLYPH_R,
    GLYPH_COLON,
    GLYPH_DASH,
    GLYPH_E,
    GLYPH_S,
    GLYPH_D,
    GLYPH_X,
    GLYPH_PERCENT,
    GLYPH_SPACE,
};

// Five-column, seven-row glyphs live in flash rather than scarce SRAM.
const uint8_t FONT[][GLYPH_COLUMNS] PROGMEM = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
    {0x7F, 0x09, 0x09, 0x09, 0x01},  // F
    {0x7F, 0x40, 0x40, 0x40, 0x40},  // L
    {0x7F, 0x09, 0x19, 0x29, 0x46},  // R
    {0x00, 0x36, 0x36, 0x00, 0x00},  // :
    {0x08, 0x08, 0x08, 0x08, 0x08},  // -
    {0x7F, 0x49, 0x49, 0x49, 0x41},  // E
    {0x46, 0x49, 0x49, 0x49, 0x31},  // S
    {0x7F, 0x41, 0x41, 0x22, 0x1C},  // D
    {0x63, 0x14, 0x08, 0x14, 0x63},  // X
    {0x63, 0x13, 0x08, 0x64, 0x63},  // %
    {0x00, 0x00, 0x00, 0x00, 0x00},  // space
};

const char SPECIAL_CHARACTERS[] PROGMEM = {
    'F', 'L', 'R', ':', '-', 'E', 'S', 'D', 'X', '%'};

const uint8_t DISPLAY_SETUP[] PROGMEM = {
    0xAE,        // display off
    0xD5, 0x80,  // clock divider
    0xA8, 0x3F,  // multiplex ratio: 64 rows
    0xD3, 0x00,  // display offset
    0x40,        // start line 0
    0x8D, 0x14,  // charge pump on
    0x20, 0x02,  // page addressing mode
    0xA1,        // segment remap
    0xC8,        // COM scan direction
    0xDA, 0x12,  // COM pin configuration
    0x81, 0x7F,  // contrast
    0xD9, 0xF1,  // pre-charge
    0xDB, 0x40,  // VCOM detect
    0xA4,        // display follows RAM
    0xA6,        // normal display
    0xAF,        // display on
};

uint8_t glyphFor(char character) {
    if (character >= '0' && character <= '9') {
        return static_cast<uint8_t>(GLYPH_0 + character - '0');
    }
    for (uint8_t index = 0; index < sizeof(SPECIAL_CHARACTERS); ++index) {
        if (character == static_cast<char>(
                pgm_read_byte(&SPECIAL_CHARACTERS[index]))) {
            return static_cast<uint8_t>(GLYPH_F + index);
        }
    }
    return GLYPH_SPACE;
}

void renderTextPage(const char* text, uint8_t half, uint8_t* output) {
    uint8_t outputIndex = 0;
    for (uint8_t character = 0; character < TEXT_CHARACTERS; ++character) {
        const uint8_t glyph = glyphFor(text[character]);
        for (uint8_t column = 0; column < GLYPH_COLUMNS; ++column) {
            const uint8_t source = pgm_read_byte(&FONT[glyph][column]);
            uint16_t scaled = 0;
            for (uint8_t row = 0; row < 7; ++row) {
                if (source & (1U << row)) scaled |= 0x03U << (row * 2);
            }
            const uint8_t pageByte = static_cast<uint8_t>(scaled >> (half * 8));
            output[outputIndex++] = pageByte;
            output[outputIndex++] = pageByte;
        }
        output[outputIndex++] = 0;
        output[outputIndex++] = 0;
    }
}

void setPixel(
    uint8_t* pagePixels,
    uint8_t page,
    uint8_t x,
    uint8_t y) {
    if (x >= DISPLAY_WIDTH || y >= DISPLAY_PAGES * 8U || y / 8U != page) {
        return;
    }
    pagePixels[x] |= static_cast<uint8_t>(1U << (y & 0x07U));
}

void clearPixel(
    uint8_t* pagePixels,
    uint8_t page,
    uint8_t x,
    uint8_t y) {
    if (x >= DISPLAY_WIDTH || y >= DISPLAY_PAGES * 8U || y / 8U != page) {
        return;
    }
    pagePixels[x] &= static_cast<uint8_t>(~(1U << (y & 0x07U)));
}

void drawHorizontal(
    uint8_t* pagePixels,
    uint8_t page,
    uint8_t x0,
    uint8_t x1,
    uint8_t y) {
    for (uint8_t x = x0; x <= x1; ++x) setPixel(pagePixels, page, x, y);
}

void drawVertical(
    uint8_t* pagePixels,
    uint8_t page,
    uint8_t x,
    uint8_t y0,
    uint8_t y1) {
    for (uint8_t y = y0; y <= y1; ++y) setPixel(pagePixels, page, x, y);
}

void drawGlyph(
    uint8_t* pagePixels,
    uint8_t page,
    uint8_t x,
    uint8_t y,
    char character,
    uint8_t scale) {
    const uint8_t glyph = glyphFor(character);
    for (uint8_t column = 0; column < GLYPH_COLUMNS; ++column) {
        const uint8_t source = pgm_read_byte(&FONT[glyph][column]);
        for (uint8_t row = 0; row < 7; ++row) {
            if ((source & (1U << row)) == 0) continue;
            for (uint8_t dx = 0; dx < scale; ++dx) {
                for (uint8_t dy = 0; dy < scale; ++dy) {
                    setPixel(
                        pagePixels,
                        page,
                        static_cast<uint8_t>(x + column * scale + dx),
                        static_cast<uint8_t>(y + row * scale + dy));
                }
            }
        }
    }
}

char phaseGlyph(MappingPhase phase) {
    switch (phase) {
        case MappingPhase::Explore: return 'E';
        case MappingPhase::ReturnToStart: return 'R';
        case MappingPhase::ShortestRun: return 'S';
        case MappingPhase::Done: return 'D';
        case MappingPhase::Fault: return 'X';
    }
    return 'X';
}

void drawMazeEdge(
    uint8_t* pagePixels,
    uint8_t page,
    uint8_t row,
    uint8_t column,
    Direction direction,
    EdgeState state) {
    const uint8_t x0 = column * MAP_PITCH;
    const uint8_t y0 = row * MAP_PITCH;
    const uint8_t x1 = x0 + MAP_PITCH;
    const uint8_t y1 = y0 + MAP_PITCH;

    // A thin lattice first makes all 81 grid positions visible; the twelve
    // excluded positions are crossed out later. Open edges cut a four-pixel
    // door through it; walls add a second parallel line; Unknown remains thin;
    // Conflict receives a centre tick.
    if (state == EdgeState::Open) {
        for (uint8_t offset = 2; offset <= 5; ++offset) {
            if (direction == Direction::North) {
                clearPixel(pagePixels, page, x0 + offset, y0);
            } else if (direction == Direction::East) {
                clearPixel(pagePixels, page, x1, y0 + offset);
            } else if (direction == Direction::South) {
                clearPixel(pagePixels, page, x0 + offset, y1);
            } else {
                clearPixel(pagePixels, page, x0, y0 + offset);
            }
        }
        return;
    }

    if (state == EdgeState::Wall) {
        if (direction == Direction::North) {
            drawHorizontal(pagePixels, page, x0, x1, y0 + 1U);
        } else if (direction == Direction::East) {
            drawVertical(pagePixels, page, x1 - 1U, y0, y1);
        } else if (direction == Direction::South) {
            drawHorizontal(pagePixels, page, x0, x1, y1 - 1U);
        } else {
            drawVertical(pagePixels, page, x0 + 1U, y0, y1);
        }
        return;
    }

    if (state != EdgeState::Conflict) return;
    const uint8_t middleX = x0 + MAP_PITCH / 2U;
    const uint8_t middleY = y0 + MAP_PITCH / 2U;
    if (direction == Direction::North || direction == Direction::South) {
        const uint8_t edgeY = direction == Direction::North ? y0 : y1;
        const uint8_t tickStart = edgeY == 0 ? 0 : edgeY - 1U;
        const uint8_t tickEnd = edgeY >= MAP_EXTENT
                              ? MAP_EXTENT : edgeY + 1U;
        drawVertical(pagePixels, page, middleX, tickStart, tickEnd);
    } else {
        const uint8_t edgeX = direction == Direction::West ? x0 : x1;
        const uint8_t tickStart = edgeX == 0 ? 0 : edgeX - 1U;
        const uint8_t tickEnd = edgeX >= MAP_EXTENT
                              ? MAP_EXTENT : edgeX + 1U;
        drawHorizontal(pagePixels, page, tickStart, tickEnd, middleY);
    }
}

void drawCellMarkers(
    uint8_t* pagePixels,
    uint8_t page,
    const Maze& maze,
    uint8_t row,
    uint8_t column,
    const Pose& start,
    uint8_t goalRow,
    uint8_t goalColumn) {
    const uint8_t x0 = column * MAP_PITCH;
    const uint8_t y0 = row * MAP_PITCH;

    // Cross out non-playable cells without allocating a separate display or
    // maze mask. The lattice and thick logical walls remain visible around
    // each excluded corner cell.
    if (!Maze::isMappable(row, column)) {
        for (uint8_t offset = 2U; offset <= 5U; ++offset) {
            setPixel(pagePixels, page, x0 + offset, y0 + offset);
            setPixel(
                pagePixels,
                page,
                x0 + offset,
                static_cast<uint8_t>(y0 + MAP_PITCH - offset));
        }
        return;
    }

    const uint8_t centreX = x0 + 3U;
    const uint8_t centreY = y0 + 3U;

    if (maze.isVisited(row, column)) {
        setPixel(pagePixels, page, centreX, centreY);
        setPixel(pagePixels, page, centreX + 1U, centreY);
        setPixel(pagePixels, page, centreX, centreY + 1U);
        setPixel(pagePixels, page, centreX + 1U, centreY + 1U);
    }

    if (row == start.row && column == start.column) {
        drawHorizontal(pagePixels, page, centreX - 1U, centreX + 1U, centreY);
        drawVertical(pagePixels, page, centreX, centreY - 1U, centreY + 1U);
    }

    if (row == goalRow && column == goalColumn) {
        drawHorizontal(pagePixels, page, x0 + 2U, x0 + 5U, y0 + 2U);
        drawHorizontal(pagePixels, page, x0 + 2U, x0 + 5U, y0 + 5U);
        drawVertical(pagePixels, page, x0 + 2U, y0 + 2U, y0 + 5U);
        drawVertical(pagePixels, page, x0 + 5U, y0 + 2U, y0 + 5U);
    }
}

void drawRobot(
    uint8_t* pagePixels,
    uint8_t page,
    const Pose& pose) {
    if (!Maze::isMappable(pose.row, pose.column)
        || !isValidDirection(pose.heading)) {
        return;
    }
    const uint8_t x0 = pose.column * MAP_PITCH;
    const uint8_t y0 = pose.row * MAP_PITCH;
    const uint8_t centreX = x0 + 3U;
    const uint8_t centreY = y0 + 3U;

    // Clear the central marker area so the arrow is legible. Keep the first
    // and last interior rows/columns intact because thick walls occupy them.
    for (uint8_t x = x0 + 2U; x <= x0 + 5U; ++x) {
        for (uint8_t y = y0 + 2U; y <= y0 + 5U; ++y) {
            clearPixel(pagePixels, page, x, y);
        }
    }

    if (pose.heading == Direction::North
        || pose.heading == Direction::South) {
        drawVertical(pagePixels, page, centreX, y0 + 1U, y0 + 6U);
        const uint8_t wingY = pose.heading == Direction::North
                            ? y0 + 2U : y0 + 5U;
        setPixel(pagePixels, page, centreX - 1U, wingY);
        setPixel(pagePixels, page, centreX + 1U, wingY);
    } else {
        drawHorizontal(pagePixels, page, x0 + 1U, x0 + 6U, centreY);
        const uint8_t wingX = pose.heading == Direction::West
                            ? x0 + 2U : x0 + 5U;
        setPixel(pagePixels, page, wingX, centreY - 1U);
        setPixel(pagePixels, page, wingX, centreY + 1U);
    }
}

void renderMapPage(
    uint8_t* pagePixels,
    uint8_t page,
    const Maze& maze,
    const Pose& pose,
    const Pose& start,
    uint8_t goalRow,
    uint8_t goalColumn,
    MappingPhase phase) {
    for (uint8_t column = 0; column < DISPLAY_WIDTH; ++column) {
        pagePixels[column] = 0;
    }

    // Full 10x10 lattice: nine rows by nine columns of the largest square
    // cells that fit on a 64-pixel-high display. The twelve non-playable
    // corner cells are crossed out below.
    for (uint8_t gridLine = 0; gridLine <= MAZE_COLUMNS; ++gridLine) {
        const uint8_t coordinate = gridLine * MAP_PITCH;
        drawHorizontal(pagePixels, page, 0, MAP_EXTENT, coordinate);
        drawVertical(pagePixels, page, coordinate, 0, MAP_EXTENT);
    }

    for (uint8_t row = 0; row < MAZE_ROWS; ++row) {
        for (uint8_t column = 0; column < MAZE_COLUMNS; ++column) {
            // Render each shared edge once. East and South cover all interior
            // edges; North and West are needed only for the outer perimeter.
            if (row == 0) {
                drawMazeEdge(
                    pagePixels,
                    page,
                    row,
                    column,
                    Direction::North,
                    maze.edge(row, column, Direction::North));
            }
            if (column == 0) {
                drawMazeEdge(
                    pagePixels,
                    page,
                    row,
                    column,
                    Direction::West,
                    maze.edge(row, column, Direction::West));
            }
            drawMazeEdge(
                pagePixels,
                page,
                row,
                column,
                Direction::East,
                maze.edge(row, column, Direction::East));
            drawMazeEdge(
                pagePixels,
                page,
                row,
                column,
                Direction::South,
                maze.edge(row, column, Direction::South));
            drawCellMarkers(
                pagePixels,
                page,
                maze,
                row,
                column,
                start,
                goalRow,
                goalColumn);
        }
    }
    drawRobot(pagePixels, page, pose);

    const uint8_t percent = static_cast<uint8_t>(
        static_cast<uint16_t>(maze.visitedCount()) * 100U
        / MAZE_MAPPABLE_CELL_COUNT);
    char completion[4] = {
        percent >= 100U ? '1' : ' ',
        percent >= 100U ? '0' : static_cast<char>('0' + (percent / 10U)),
        percent >= 100U ? '0' : static_cast<char>('0' + (percent % 10U)),
        '%',
    };
    if (percent < 10U) completion[1] = ' ';
    for (uint8_t index = 0; index < 4; ++index) {
        drawGlyph(
            pagePixels,
            page,
            static_cast<uint8_t>(70U + index * 12U),
            4,
            completion[index],
            2);
    }
    drawGlyph(pagePixels, page, 89, 38, phaseGlyph(phase), 3);
}

}  // namespace

bool MicromouseDisplay::initialise() {
    mReady = true;

    // SSD1306 128x64, internal charge pump, page addressing mode. Commands are
    // sent in one transaction and stored in flash to preserve Nano SRAM.
    if (!writeFlashCommands(DISPLAY_SETUP, sizeof(DISPLAY_SETUP)) || !clear()) {
        disable();
        return false;
    }

    showReading(0, 0, false);
    showReading(1, 0, false);
    showReading(2, 0, false);
    return mReady;
}

bool MicromouseDisplay::isReady() const {
    return mReady;
}

void MicromouseDisplay::showReading(
    uint8_t sensorIndex,
    uint16_t distanceMm,
    bool valid) {
    if (!mReady || sensorIndex > 2) return;

    char label = 'R';
    if (sensorIndex == 0) {
        label = 'F';
    } else if (sensorIndex == 1) {
        label = 'L';
    }
    char text[TEXT_CHARACTERS] = {label, ':', '-', '-', '-'};
    if (valid) {
        if (distanceMm > 999) distanceMm = 999;
        text[2] = static_cast<char>('0' + distanceMm / 100);
        text[3] = static_cast<char>('0' + (distanceMm / 10) % 10);
        text[4] = static_cast<char>('0' + distanceMm % 10);
    }

    uint8_t pixels[TEXT_WIDTH];
    const uint8_t firstPage = FIRST_TEXT_PAGE + sensorIndex * 2;
    for (uint8_t half = 0; half < 2; ++half) {
        renderTextPage(text, half, pixels);
        if (!setPageAndColumn(firstPage + half, TEXT_COLUMN)
            || !writeData(pixels, sizeof(pixels))) {
            disable();
            return;
        }
    }
}

void MicromouseDisplay::showMap(
    const Maze& maze,
    const Pose& pose,
    const Pose& start,
    uint8_t goalRow,
    uint8_t goalColumn,
    MappingPhase phase) {
    if (!mReady) return;

    // One 128-byte page is the only framebuffer. The full 1 KB image is never
    // resident in Nano SRAM, and this method is called only with motors stopped.
    uint8_t pixels[DISPLAY_WIDTH];
    for (uint8_t page = 0; page < DISPLAY_PAGES; ++page) {
        renderMapPage(
            pixels,
            page,
            maze,
            pose,
            start,
            goalRow,
            goalColumn,
            phase);
        if (!setPageAndColumn(page, 0) || !writeData(pixels, sizeof(pixels))) {
            disable();
            return;
        }
    }
}

bool MicromouseDisplay::writeCommands(const uint8_t* commands, uint8_t count) {
    if (!mReady) return false;

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    for (uint8_t index = 0; index < count; ++index) {
        Wire.write(commands[index]);
    }
    const uint8_t status = Wire.endTransmission();
    const bool success = status == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    return success;
}

bool MicromouseDisplay::writeFlashCommands(
    const uint8_t* commands,
    uint8_t count) {
    if (!mReady) return false;

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    for (uint8_t index = 0; index < count; ++index) {
        Wire.write(pgm_read_byte(&commands[index]));
    }
    const uint8_t status = Wire.endTransmission();
    const bool success = status == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    return success;
}

bool MicromouseDisplay::writeData(const uint8_t* data, uint8_t count) {
    if (!mReady) return false;

    uint8_t sent = 0;
    while (sent < count) {
        const uint8_t remaining = count - sent;
        const uint8_t chunk = remaining < DATA_CHUNK_BYTES
                            ? remaining
                            : DATA_CHUNK_BYTES;
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(DISPLAY_ADDRESS);
        Wire.write(static_cast<uint8_t>(0x40));
        for (uint8_t index = 0; index < chunk; ++index) {
            Wire.write(data[sent + index]);
        }
        const uint8_t status = Wire.endTransmission();
        const bool success = status == 0 && !Wire.getWireTimeoutFlag();
        Wire.clearWireTimeoutFlag();
        if (!success) return false;
        sent += chunk;
    }
    return true;
}

bool MicromouseDisplay::setPageAndColumn(uint8_t page, uint8_t column) {
    const uint8_t commands[] = {
        static_cast<uint8_t>(0xB0 | page),
        static_cast<uint8_t>(column & 0x0F),
        static_cast<uint8_t>(0x10 | (column >> 4)),
    };
    return writeCommands(commands, sizeof(commands));
}

bool MicromouseDisplay::clear() {
    const uint8_t blank[DATA_CHUNK_BYTES] = {};
    for (uint8_t page = 0; page < DISPLAY_PAGES; ++page) {
        if (!setPageAndColumn(page, 0)) return false;
        uint8_t remaining = DISPLAY_WIDTH;
        while (remaining > 0) {
            const uint8_t count = remaining < sizeof(blank)
                                ? remaining
                                : sizeof(blank);
            if (!writeData(blank, count)) return false;
            remaining -= count;
        }
    }
    return true;
}

void MicromouseDisplay::disable() {
    mReady = false;
    Wire.clearWireTimeoutFlag();
}

}  // namespace mtrn3100
