#!/bin/bash
PATHS=(
  "./src/main/deploy/pathplanner/paths/1L.path"
  "./src/main/deploy/pathplanner/paths/1L1.path"
  "./src/main/deploy/pathplanner/paths/LeftStation.path"
  "./src/main/deploy/pathplanner/paths/ThreePieceLeftA.path"
  "./src/main/deploy/pathplanner/paths/ThreePieceLeftB.path"
  "./src/main/deploy/pathplanner/paths/ThreePieceLeftC.path"
  "./src/main/deploy/pathplanner/paths/ThreePieceLeftD.path"
  "./src/main/deploy/pathplanner/paths/ThreePieceLeftE.path"
  "./src/main/deploy/pathplanner/paths/ThreePieceLeftStart.path"
)

# Default arguments
REEF="blue"
TAG_FILE="./tools/pathy/apriltag/2025-reefscape-welded.json"
TAG_ID1=18
TAG_ID2=21
OUTPUT_DIR="./src/main/deploy/pathplanner/paths"
# =========================================

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
MAIN_SCRIPT="$SCRIPT_DIR/main.py"

# Common arguments for all paths
COMMON_ARGS="--reef $REEF --tag-file $TAG_FILE --tag-id1 $TAG_ID1 --tag-id2 $TAG_ID2"

# Add output directory if specified
if [ -n "$OUTPUT_DIR" ]; then
  COMMON_ARGS="$COMMON_ARGS --output-dir $OUTPUT_DIR"
  
  # Create output directory if it doesn't exist
  if [ ! -d "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    echo "Created output directory: $OUTPUT_DIR"
  fi
fi

echo "Processing specific paths with arguments: $COMMON_ARGS"
echo "-------------------------------------------"

# Process each path file
for path in "${PATHS[@]}"; do
  if [ ! -f "$path" ]; then
    echo "Error: Path file not found: $path"
    continue
  fi
  
  filename=$(basename "$path")
  
  # Check if filename contains "Left" or "Right" to apply --only-reflect
  if [[ "$filename" =~ (Left|Right) ]]; then
    echo "Processing (reflect only): $path"
    python "$MAIN_SCRIPT" "$path" $COMMON_ARGS --only-reflect
  else
    echo "Processing (all transformations): $path"
    python "$MAIN_SCRIPT" "$path" $COMMON_ARGS
  fi
  
  echo
done

echo "All specified paths processed!"
