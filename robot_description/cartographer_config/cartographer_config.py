def cartographer_config(robot_name, original_file, new_file):
    with open(original_file, 'r') as file:
        lines = file.readlines()

    with open(new_file, 'w') as file:
        for line in lines:
            if 'map_frame = "submap"' in line:
                file.write(f'  map_frame = "{robot_name}_submap",\n')
            elif 'tracking_frame = "base_footprint"' in line:
                file.write(f'  tracking_frame = "{robot_name}_base_footprint",\n')
            elif 'published_frame = "odom"' in line:
                file.write(f'  published_frame = "{robot_name}_odom",\n')
            elif 'odom_frame = "odom"' in line:
                file.write(f'  odom_frame = "{robot_name}_odom",\n')
            else:
                file.write(line)

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 4:
        print("Usage: python3 cartographer_config.py <robot_name> <original_file> <new_file>")
    else:
        robot_name = sys.argv[1]
        original_file = sys.argv[2]
        new_file = sys.argv[3]
        cartographer_config(robot_name, original_file, new_file)