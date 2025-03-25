import json
import math
import os
import copy

def load_json_file(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

def save_json_file(data, file_path):
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=2)

def calculate_reef_center(tag_data, tag_id1, tag_id2):
    # Find the tag positions from their IDs
    tag1_pos = None
    tag2_pos = None
    
    for tag in tag_data["tags"]:
        if tag["ID"] == tag_id1:
            tag1_pos = tag["pose"]["translation"]
        elif tag["ID"] == tag_id2:
            tag2_pos = tag["pose"]["translation"]
    
    if not tag1_pos or not tag2_pos:
        raise ValueError(f"Could not find tags with IDs {tag_id1} and {tag_id2}")
    
    # Calculate midpoint
    center_x = (tag1_pos["x"] + tag2_pos["x"]) / 2
    center_y = (tag1_pos["y"] + tag2_pos["y"]) / 2
    
    return center_x, center_y

def rotate_point(x, y, center_x, center_y, angle_deg):
    # Convert angle to radians
    angle_rad = math.radians(angle_deg)
    
    # Translate point to origin
    translated_x = x - center_x
    translated_y = y - center_y
    
    # Rotate
    rotated_x = translated_x * math.cos(angle_rad) - translated_y * math.sin(angle_rad)
    rotated_y = translated_x * math.sin(angle_rad) + translated_y * math.cos(angle_rad)
    
    # Translate back
    new_x = rotated_x + center_x
    new_y = rotated_y + center_y
    
    return new_x, new_y

def reflect_point(x, y, center_x, center_y, reflection_angle=0):
    """
    Reflect a point across a line that passes through the center of the reef 
    at the specified angle.
    
    reflection_angle: angle in degrees from the positive x-axis (counterclockwise)
    representing the direction of the side of the reef
    """
    # Convert reflection angle to radians
    angle_rad = math.radians(reflection_angle)
    
    # Calculate the perpendicular reflection line (90 degrees offset from the side)
    perpendicular_angle_rad = angle_rad + math.pi/2
    
    # Translate point to be relative to center
    dx = x - center_x
    dy = y - center_y
    
    # Calculate the dot product with the perpendicular vector
    # This gives us the component of the point's position along the perpendicular
    perp_x = math.cos(perpendicular_angle_rad)
    perp_y = math.sin(perpendicular_angle_rad)
    dot_product = dx * perp_x + dy * perp_y
    
    # Reflect by subtracting twice the perpendicular component
    reflected_x = x - 2 * dot_product * perp_x
    reflected_y = y - 2 * dot_product * perp_y
    
    return reflected_x, reflected_y

def transform_coordinates(path_data, center_x, center_y, operation, angle=0, reflection_angle=0):
    """
    Transform all coordinates in a path according to the specified operation.
    
    operation: "rotate" or "reflect"
    angle: rotation angle in degrees (for "rotate" operation)
    reflection_angle: reflection axis angle in degrees (for "reflect" operation)
    """
    new_path = copy.deepcopy(path_data)
    
    # Helper function to transform a point based on the operation
    def transform_point(point):
        if not point:
            return point
            
        if operation == "rotate":
            point["x"], point["y"] = rotate_point(
                point["x"], point["y"], center_x, center_y, angle
            )
        elif operation == "reflect":
            point["x"], point["y"] = reflect_point(
                point["x"], point["y"], center_x, center_y, reflection_angle
            )
        return point
    
    # Helper function to transform an angle
    def transform_angle(original_angle):
        if operation == "rotate":
            return (original_angle + angle) % 360
        elif operation == "reflect":
            # For reflection: first make angle relative to reflection axis,
            # then negate (reflecting), then convert back to global reference
            relative_angle = original_angle - reflection_angle
            reflected_relative_angle = -relative_angle
            reflected_angle = (reflection_angle + reflected_relative_angle) % 360
            return reflected_angle
        return original_angle
    
    # Transform all waypoints
    for waypoint in new_path["waypoints"]:
        transform_point(waypoint["anchor"])
        transform_point(waypoint["prevControl"])
        transform_point(waypoint["nextControl"])
    
    # Transform rotation targets - fix the property name
    if "rotationTargets" in new_path and new_path["rotationTargets"]:
        for target in new_path["rotationTargets"]:
            if "rotationDegrees" in target:
                target["rotationDegrees"] = transform_angle(target["rotationDegrees"])
    
    # Transform goalEndState rotation
    if "goalEndState" in new_path and "rotation" in new_path["goalEndState"]:
        new_path["goalEndState"]["rotation"] = transform_angle(new_path["goalEndState"]["rotation"])
    
    # Transform idealStartingState rotation
    if "idealStartingState" in new_path and "rotation" in new_path["idealStartingState"]:
        new_path["idealStartingState"]["rotation"] = transform_angle(new_path["idealStartingState"]["rotation"])
    
    return new_path
