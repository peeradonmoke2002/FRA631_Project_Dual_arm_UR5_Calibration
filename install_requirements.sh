#!/bin/bash

# Read env mode from argument
ENV_MODE=$1

# Default to dev if not provided
if [ -z "$ENV_MODE" ]; then
  ENV_MODE="dev"
fi

echo "=========================================="
echo "Start installing requirements for: $ENV_MODE"
echo "=========================================="

if [ "$ENV_MODE" = "robot" ]; then
    REQUIRE_FILE="robot-requirements.txt"
elif [ "$ENV_MODE" = "dev" ]; then
    REQUIRE_FILE="dev-requirements.txt"
else
    echo "Unknown ENV_MODE: $ENV_MODE"
    echo "Usage: ./install_requirements.sh [dev|robot]"
    exit 1
fi

# Check file exists
if [ ! -f "$REQUIRE_FILE" ]; then
    echo "Error: $REQUIRE_FILE not found!"
    exit 1
fi

# Show which file used
echo "Using requirements file: $REQUIRE_FILE"
echo ""

# Install with output
pip install -r "$REQUIRE_FILE"

echo ""
echo "=========================================="
echo "Done installing for: $ENV_MODE"
echo "=========================================="
