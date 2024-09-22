#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Change to the script directory.
cd $SCRIPT_DIR

# Clean the build directory.
make clean

# Build the documentation.
make html

# Deploy the documentation to GitHub Pages.
ghp-import -n -p -f _build/html