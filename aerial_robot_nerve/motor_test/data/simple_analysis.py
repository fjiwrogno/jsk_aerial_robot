#!/usr/bin/env python3
import sys

def analyze_data():
    # File paths
    file1 = "/home/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/u=25.20v_motor_test_1706621604.txt"
    file2 = "/home/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/motor_test_aerialWithunit1753271753.txt"
    
    print("Reading data files...")
    
    # Read file 1
    try:
        with open(file1, 'r') as f:
            lines1 = f.readlines()
        print(f"File 1: {len(lines1)} lines")
        if lines1:
            print(f"First line: {lines1[0].strip()}")
            print(f"Last line: {lines1[-1].strip()}")
    except Exception as e:
        print(f"Error reading file 1: {e}")
        return
    
    # Read file 2
    try:
        with open(file2, 'r') as f:
            lines2 = f.readlines()
        print(f"File 2: {len(lines2)} lines")
        if lines2:
            print(f"First line: {lines2[0].strip()}")
            print(f"Last line: {lines2[-1].strip()}")
    except Exception as e:
        print(f"Error reading file 2: {e}")
        return
    
    # Parse and filter data
    data1_filtered = []
    data2_filtered = []
    
    print("\nFiltering data for PWM range 1100-1500...")
    
    # Process file 1
    for line in lines1:
        try:
            values = line.strip().split()
            if len(values) >= 12:
                pwm = float(values[0])
                if 1100 <= pwm <= 1500:
                    data1_filtered.append([float(v) for v in values])
        except:
            continue
    
    # Process file 2
    for line in lines2:
        try:
            values = line.strip().split()
            if len(values) >= 12:
                pwm = float(values[0])
                if 1550 <= pwm <= 1750:
                    data2_filtered.append([float(v) for v in values])
        except:
            continue
    
    print(f"File 1 filtered data points: {len(data1_filtered)}")
    print(f"File 2 filtered data points: {len(data2_filtered)}")
    
    if not data1_filtered or not data2_filtered:
        print("No data found in the specified PWM range!")
        return
    
    # Create gnuplot scripts
    create_gnuplot_files(data1_filtered, data2_filtered)

def create_gnuplot_files(data1, data2):
    # Write filtered data to files
    with open('rotor_data.txt', 'w') as f:
        for row in data1:
            f.write(f"{row[0]} {row[3]} {row[8]} {row[9]}\n")  # PWM, Force_Z, Current, RPM
    
    with open('rotor_with_unit_data.txt', 'w') as f:
        for row in data2:
            f.write(f"{row[0]} {row[3]} {row[8]} {row[9]}\n")  # PWM, Force_Z, Current, RPM
    
    # Create gnuplot script
    gnuplot_script = """
set terminal png size 1800,600
set output 'motor_comparison.png'

set multiplot layout 1,3 title 'Motor Test Data Comparison: Rotor vs Rotor with unit'

# Plot 1: Force Z
set title 'Force Z vs PWM (1100-1500)'
set xlabel 'PWM Value'
set ylabel 'Force Z (N)'
set grid
plot 'rotor_data.txt' using 1:2 with points pt 7 ps 0.5 lc rgb '#2E86AB' title 'Rotor', \\
     'rotor_with_unit_data.txt' using 1:2 with points pt 7 ps 0.5 lc rgb '#A23B72' title 'Rotor with unit'

# Plot 2: Current
set title 'Current vs PWM (1100-1500)'
set xlabel 'PWM Value'
set ylabel 'Current (A)'
set grid
plot 'rotor_data.txt' using 1:3 with points pt 7 ps 0.5 lc rgb '#2E86AB' title 'Rotor', \\
     'rotor_with_unit_data.txt' using 1:3 with points pt 7 ps 0.5 lc rgb '#A23B72' title 'Rotor with unit'

# Plot 3: RPM
set title 'RPM vs PWM (1100-1500)'
set xlabel 'PWM Value'
set ylabel 'RPM'
set grid
plot 'rotor_data.txt' using 1:4 with points pt 7 ps 0.5 lc rgb '#2E86AB' title 'Rotor', \\
     'rotor_with_unit_data.txt' using 1:4 with points pt 7 ps 0.5 lc rgb '#A23B72' title 'Rotor with unit'

unset multiplot
"""
    
    with open('plot_comparison.gp', 'w') as f:
        f.write(gnuplot_script)
    
    print("Created gnuplot files:")
    print("- rotor_data.txt")
    print("- rotor_with_unit_data.txt") 
    print("- plot_comparison.gp")
    print("\nTo generate the plot, run: gnuplot plot_comparison.gp")
    
    # Show statistics
    print("\n=== Data Statistics ===")
    print("Rotor (File 1):")
    force_z1 = [row[3] for row in data1]
    current1 = [row[8] for row in data1]
    rpm1 = [row[9] for row in data1]
    print(f"  Data points: {len(data1)}")
    print(f"  Force Z range: {min(force_z1):.2f} to {max(force_z1):.2f} N")
    print(f"  Current range: {min(current1):.2f} to {max(current1):.2f} A")
    print(f"  RPM range: {min(rpm1):.0f} to {max(rpm1):.0f}")
    
    print("\nRotor with unit (File 2):")
    force_z2 = [row[3] for row in data2]
    current2 = [row[8] for row in data2]
    rpm2 = [row[9] for row in data2]
    print(f"  Data points: {len(data2)}")
    print(f"  Force Z range: {min(force_z2):.2f} to {max(force_z2):.2f} N")
    print(f"  Current range: {min(current2):.2f} to {max(current2):.2f} A")
    print(f"  RPM range: {min(rpm2):.0f} to {max(rpm2):.0f}")

if __name__ == "__main__":
    analyze_data()
