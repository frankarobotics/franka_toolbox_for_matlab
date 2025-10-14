#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the script directory
cd "$SCRIPT_DIR"

# Create build directory if it doesn't exist
mkdir -p build/interface

# Define the Cap'n Proto file
CAPNP_FILE="interface/rpc.capnp"

# Compile the Cap'n Proto file
if ! capnp compile -oc++:./build/interface "$CAPNP_FILE" --src-prefix=interface -I.; then
    echo "Error: Failed to compile Cap'n Proto file"
    exit 1
fi

echo "Cap'n Proto compilation successful"
