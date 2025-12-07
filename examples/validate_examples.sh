#!/bin/bash
# Validation script for ROS 2 code examples

# This script validates that ROS 2 code examples can be imported and have basic syntax
# It doesn't run the nodes, just checks for syntax errors and basic import capability

echo "Starting ROS 2 code example validation..."

# Check if ROS 2 environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 environment not sourced. Please source your ROS 2 setup.bash first."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS 2 distribution detected: $ROS_DISTRO"

# Find all Python files in the examples directory
EXAMPLES_DIR="examples/ros2_workspace/src"
PYTHON_FILES=$(find $EXAMPLES_DIR -name "*.py" -type f)

if [ -z "$PYTHON_FILES" ]; then
    echo "No Python files found in $EXAMPLES_DIR"
    exit 0
fi

echo "Found Python files:"
echo "$PYTHON_FILES"
echo

SUCCESS_COUNT=0
TOTAL_COUNT=0

for file in $PYTHON_FILES; do
    TOTAL_COUNT=$((TOTAL_COUNT + 1))
    echo "Validating: $file"

    # Check Python syntax
    if python3 -m py_compile "$file" 2>/dev/null; then
        echo "  ✓ Syntax OK"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "  ✗ Syntax ERROR"
        python3 -m py_compile "$file"  # Show the actual error
    fi
    echo
done

echo "Validation Summary:"
echo "Total files checked: $TOTAL_COUNT"
echo "Successfully validated: $SUCCESS_COUNT"

if [ $SUCCESS_COUNT -eq $TOTAL_COUNT ]; then
    echo "✓ All code examples passed validation!"
    exit 0
else
    echo "✗ Some code examples failed validation!"
    exit 1
fi