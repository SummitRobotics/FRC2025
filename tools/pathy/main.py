import os
import sys
import argparse
import copy
import re
from path_transformer import (
    load_json_file, 
    save_json_file, 
    calculate_reef_center, 
    transform_coordinates
)

def main():
    parser = argparse.ArgumentParser(description="FRC 2025 Reefscape Path Transformer")
    parser.add_argument("path_file", help="Path to process (e.g., pathplanner/paths/1L.path)")
    parser.add_argument("--tag-id1", type=int, default=18,
                        help="First AprilTag ID on reef (default: 18)")
    parser.add_argument("--tag-id2", type=int, default=21,
                        help="Second AprilTag ID on reef, opposite to the first (default: 21)")
    parser.add_argument("--tag-file", default="2025-reefscape-welded.json", 
                        help="Path to AprilTag field layout file (default: 2025-reefscape-welded.json)")
    parser.add_argument("--reef", choices=["blue", "red"], default="blue",
                       help="Which reef to use for transformations (default: blue)")
    parser.add_argument("--only-reflect", action="store_true",
                       help="Only generate the reflected path (no rotations)")
    parser.add_argument("--output-dir", 
                       help="Directory where transformed paths should be saved (default: same as input)")
    args = parser.parse_args()

    # Check if the required file exists
    tag_file = args.tag_file
    if not os.path.exists(tag_file):
        print(f"Error: Tag file '{tag_file}' not found.")
        print("Please download this file from the FRC game documentation.")
        return

    print("FRC 2025 Reefscape Path Transformer")
    print("-----------------------------------")
    print(f"Reef: {args.reef.upper()}")
    
    # Get path file
    path_file = args.path_file
    if not os.path.exists(path_file):
        print(f"Error: Path file '{path_file}' not found.")
        return
    
    # Get AprilTag IDs
    tag_id1 = args.tag_id1
    tag_id2 = args.tag_id2
    
    # Process the files
    try:
        # Load data
        tag_data = load_json_file(tag_file)
        path_data = load_json_file(path_file)
        
        # Calculate center
        center_x, center_y = calculate_reef_center(tag_data, tag_id1, tag_id2)
        print(f"\nReef center calculated at: ({center_x:.4f}, {center_y:.4f})")
        
        # Get output directory and base filename
        path_dir = os.path.dirname(path_file)
        path_basename = os.path.basename(path_file)
        
        # Use specified output directory if provided
        output_dir = args.output_dir if args.output_dir else path_dir
        
        # Create output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"Created output directory: {output_dir}")
        
        # First, save a copy of the input file to the output directory if different
        if output_dir != path_dir:
            input_copy_path = os.path.join(output_dir, path_basename)
            save_json_file(path_data, input_copy_path)
            print(f"Saved copy of input file to output directory: {input_copy_path}")
        
        # Parse the filename to determine side and orientation (L/R)
        filename_pattern = r"^(\d)([LR])(.*)\.path$"
        match = re.match(filename_pattern, path_basename)
        
        filename_stem = os.path.splitext(path_basename)[0]
        file_extension = os.path.splitext(path_basename)[1]
        
        # Check if "Left" or "Right" appears in the filename
        has_left_right = False
        if "Left" in filename_stem:
            orientation_text = "Left"
            opposite_text = "Right"
            has_left_right = True
        elif "Right" in filename_stem:
            orientation_text = "Right"
            opposite_text = "Left"
            has_left_right = True
        
        if not match and not has_left_right:
            print(f"Warning: Path filename '{path_basename}' doesn't match expected patterns")
            print("Defaulting to side 1, orientation L")
            input_side = 1
            orientation = "L"
            remaining_part = filename_stem
        elif has_left_right:
            # Use the Left/Right from filename
            print(f"Detected orientation from filename: {orientation_text}")
            input_side = 1  # Default to side 1
            orientation = orientation_text[0]  # Use first letter (L or R)
            remaining_part = ""  # Will use the whole filename differently
        else:
            input_side = int(match.group(1))
            orientation = match.group(2)
            remaining_part = match.group(3)
            
            # Validate side
            if input_side < 1 or input_side > 6:
                print(f"Warning: Invalid side number {input_side}. Side must be 1-6. Defaulting to 1.")
                input_side = 1
        
        print(f"Detected: Side {input_side}, Orientation {orientation}")
        
        # If only reflecting, handle that case separately
        if args.only_reflect:
            print("\nGenerating only reflected path...")
            
            # Set reflection angle based on side 1 orientation (0°)
            reflection_angle = 0
            
            # For red reef, adjust the reflection angle
            if args.reef == "red":
                reflection_angle = 180
                print(f"Note: Using reflection angle {reflection_angle}° for red reef")
            
            # Generate the reflected path
            reflected_path = transform_coordinates(
                path_data, center_x, center_y, "reflect", 
                reflection_angle=reflection_angle
            )
            
            # Create the output filename
            if has_left_right:
                # Replace "Left" with "Right" or vice versa
                output_filename = filename_stem.replace(orientation_text, opposite_text) + file_extension
            else:
                # Swap L/R in the filename
                opposite_orientation = "R" if orientation == "L" else "L"
                output_filename = f"{input_side}{opposite_orientation}{remaining_part}.path"
            
            output_file = os.path.join(output_dir, output_filename)
            save_json_file(reflected_path, output_file)
            print(f"  Created reflected path: {output_file}")
            
            print("\nReflection completed successfully!")
            return
        
        # Calculate rotation needed to align with side 1
        # For input side N, we need to rotate by -(N-1)*60 to get to side 1 position
        align_rotation = -(input_side - 1) * 60
        
        # If we're processing red reef and not already on side 1, 
        # we need to invert the rotation direction
        if args.reef == "red" and input_side != 1:
            align_rotation = -align_rotation
            print(f"Note: Adjusting alignment rotation for red reef: {align_rotation}°")
        
        # Align the path to side 1 position
        aligned_path = path_data
        if input_side != 1:
            aligned_path = transform_coordinates(path_data, center_x, center_y, "rotate", align_rotation)
            print(f"Aligned path from side {input_side} to side 1 with {align_rotation}° rotation")
        
        # Set rotation angles (counterclockwise) for the 6 sides
        rotation_angles = [0, 60, 120, 180, 240, 300]
        
        # For red reef, invert the rotation directions
        if args.reef == "red":
            rotation_angles = [0] + [-angle for angle in rotation_angles[1:]]
            print("Note: Using inverted rotation for red reef")
        
        # Generate 6 rotated paths (one for each side)
        print("\nGenerating paths for all sides...")
        
        generated_paths = []
        
        for i, angle in enumerate(rotation_angles):
            side_number = i + 1
            
            # Calculate the actual rotation needed
            if input_side == 1:
                # If we started at side 1, just use the rotation angle
                actual_angle = angle
            else:
                # If we aligned from another side, combine the rotations
                actual_angle = (align_rotation + angle) % 360
            
            # Generate path for this side
            if angle == 0:
                # For side 1, use the aligned path
                rotated_path = aligned_path
            else:
                # For other sides, rotate from the aligned path
                rotated_path = transform_coordinates(aligned_path, center_x, center_y, "rotate", angle)
            
            # Store for later reflection
            generated_paths.append(rotated_path)
            
            # Create the filename with the correct side number (without reef color)
            new_filename = f"{side_number}{orientation}{remaining_part}.path"
            output_file = os.path.join(output_dir, new_filename)
            
            save_json_file(rotated_path, output_file)
            
            if side_number == input_side:
                print(f"  Saved original side path: {output_file}")
            else:
                print(f"  Created side {side_number} path: {output_file}")
        
        # Generate reflections (swap L to R or R to L)
        print("\nGenerating reflected paths...")
        
        # Determine the opposite orientation
        opposite_orientation = "R" if orientation == "L" else "L"
        
        for i, angle in enumerate(rotation_angles):
            side_number = i + 1
            path_to_reflect = generated_paths[i]
            
            # Reflect the path across its side's axis
            reflected_path = transform_coordinates(
                path_to_reflect, center_x, center_y, "reflect", 
                reflection_angle=angle
            )
            
            # Create the filename with the opposite orientation (without reef color)
            new_filename = f"{side_number}{opposite_orientation}{remaining_part}.path"
            output_file = os.path.join(output_dir, new_filename)
            
            save_json_file(reflected_path, output_file)
            print(f"  Created reflected path for side {side_number}: {output_file}")
        
        print("\nAll paths created successfully!")
        print(f"Generated 12 paths: 6 sides with both L and R orientations for {args.reef} reef")
        
    except Exception as e:
        print(f"Error: {e}")
        return

if __name__ == "__main__":
    main()
