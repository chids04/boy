#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build/debug"
EXECUTABLE="$BUILD_DIR/boy"

# Default to debug, allow override
PRESET="${1:-debug}"
TEST_ROM="${2:-}"

echo "🔨 Building with preset: $PRESET"
cmake --preset "$PRESET" -S "$PROJECT_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR"

if [ ! -f "$EXECUTABLE" ]; then
    echo "❌ Build failed: executable not found at $EXECUTABLE"
    exit 1
fi

echo "✅ Build successful"

if [ -z "$TEST_ROM" ]; then
    echo "⚠️  No test ROM specified"
    echo "Usage: $0 [preset] [test_rom_path]"
    echo "Example: $0 debug test/gb_doctor/cpu_instrs.gb"
    exit 0
fi

if [ ! -f "$TEST_ROM" ]; then
    echo "❌ Test ROM not found: $TEST_ROM"
    exit 1
fi

echo "🧪 Running test: $TEST_ROM"
"$EXECUTABLE" "$TEST_ROM"
