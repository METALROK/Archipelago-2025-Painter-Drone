def convert_trajectory(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        prev_command = None
        for line in infile:
            line = line.strip()
            if not line:
                continue

            parts = line.split()
            command = parts[0]
            x = parts[1]
            z = parts[2]
            if command != prev_command:
                if command == 'MOVE':
                    outfile.write("pi.set_servo_pulsewidth(21, 2000)\n")#2000 - assumed NOT drawing
                elif command == 'DRAW':
                    outfile.write("pi.set_servo_pulsewidth(21, 1000)\n")#1000 - assumed drawing
                prev_command = command

            outfile.write(f"set_position(x={x}, y=0, z={z})\n")


# Пример использования
convert_trajectory('trajectory.txt', 'converted_trajectory.txt')