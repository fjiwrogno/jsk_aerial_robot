#!/usr/bin/env python3

def analyze_valid_data():
    # File paths
    file1 = "/home/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/u=25.20v_motor_test_1706621604.txt"
    file2 = "/    print(f"  RPM range: {min(rpm2):.0f} to {max(rpm2):.0f} (filtered > 14000)")
    print(f"  RPM average: {sum(rpm2)/len(rpm2):.0f}")
    if rpm2_capped > 0:
        print(f"  ⚠️  {rpm2_capped} RPM values > 14000 were capped to 14000")/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/motor_test_aerialWithunit1753271753.txt"
    
    print("Processing motor test data...")
    print("File 1: PWM range 1100-1500, status='valid'")
    print("File 2: PWM range 1550-1750, status='valid'")
    
    # Parse and filter data
    data1_filtered = []
    data2_filtered = []
    
    # Process file 1
    print("\nProcessing file 1...")
    try:
        with open(file1, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if line and line != "done":
                    values = line.split()
                    if len(values) >= 13:  # Ensure we have status column
                        try:
                            pwm = float(values[0])
                            status = values[12]  # Status is the last column
                            if status == "valid" and 1100 <= pwm <= 1500:
                                # Store: PWM, Force_X, Force_Y, Force_Z, Force_norm, Torque_X, Torque_Y, Torque_Z, Current, RPM, Temp, Voltage
                                data1_filtered.append([float(v) for v in values[:12]])
                        except ValueError:
                            continue
        print(f"Found {len(data1_filtered)} valid data points in PWM range 1100-1500")
    except Exception as e:
        print(f"Error reading file 1: {e}")
        return
    
    # Process file 2
    print("Processing file 2...")
    try:
        with open(file2, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if line and line != "done":
                    values = line.split()
                    if len(values) >= 13:  # Ensure we have status column
                        try:
                            pwm = float(values[0])
                            status = values[12]  # Status is the last column
                            if status == "valid" and 1550 <= pwm <= 1750:
                                # Store: PWM, Force_X, Force_Y, Force_Z, Force_norm, Torque_X, Torque_Y, Torque_Z, Current, RPM, Temp, Voltage
                                data2_filtered.append([float(v) for v in values[:12]])
                        except ValueError:
                            continue
        print(f"Found {len(data2_filtered)} valid data points in PWM range 1550-1750")
    except Exception as e:
        print(f"Error reading file 2: {e}")
        return
    
    if not data1_filtered:
        print("No valid data found in file 1 for PWM range 1100-1500!")
        return
    
    if not data2_filtered:
        print("No valid data found in file 2 for PWM range 1550-1750!")
        return
    
    # Create data files for plotting
    create_plot_data(data1_filtered, data2_filtered)
    
    # Show statistics
    show_statistics(data1_filtered, data2_filtered)

def create_plot_data(data1, data2):
    print("\nCreating plot data files...")
    
    # Write filtered data to files for plotting
    with open('rotor_data.txt', 'w') as f:
        f.write("# PWM Force_Z Current RPM\n")
        for row in data1:
            # PWM, Force_Z (col 3), Current (col 8), RPM (col 9)
            # Filter out RPM > 14000 (abnormal values)
            rpm_value = row[9] if row[9] <= 14000 else 14000
            f.write(f"{row[0]:.1f} {row[3]:.4f} {row[8]:.4f} {rpm_value:.0f}\n")
    
    with open('rotor_with_unit_data.txt', 'w') as f:
        f.write("# PWM_mapped Force_Z Current RPM\n")
        for row in data2:
            # Map PWM from 1550-1750 to 1100-1400 (1550->1100, 1750->1400)
            # Linear mapping: new_pwm = 1100 + (old_pwm - 1550) * (1400-1100)/(1750-1550)
            original_pwm = row[0]
            mapped_pwm = 1100 + (original_pwm - 1550) * (1400-1100) / (1750-1550)
            # Multiply Force Z by -1 to correct the sign
            corrected_force_z = -row[3]
            # Filter out RPM > 14000 (abnormal values)
            rpm_value = row[9] if row[9] <= 14000 else 14000
            f.write(f"{mapped_pwm:.1f} {corrected_force_z:.4f} {row[8]:.4f} {rpm_value:.0f}\n")
    
    # Create gnuplot script with enhanced visibility for sparse data
    gnuplot_script = """
set terminal png size 2200,800 font "Arial,12"
set output 'motor_comparison.png'

set multiplot layout 1,3 title 'Motor Test Data Comparison: Rotor (9 PWM points) vs Rotor with unit (3 PWM points)' font "Arial,16"

# Plot 1: Force Z with curve fitting
set title 'Force Z vs PWM\\n(Rotor: continuous, Rotor+unit: 3 discrete points)' font "Arial,13"
set xlabel 'PWM Value' font "Arial,12"
set ylabel 'Force Z (N)' font "Arial,12"
set grid
set key top left font "Arial,10"

# Define fitting functions (polynomial)
f1(x) = a1*x*x + b1*x + c1
f2(x) = a2*x*x + b2*x + c2

# Fit curves
fit f1(x) 'rotor_data.txt' using 1:2 via a1,b1,c1
fit f2(x) 'rotor_with_unit_data.txt' using 1:2 via a2,b2,c2

plot 'rotor_data.txt' using 1:2 with points pt 7 ps 0.6 lc rgb '#0066CC' title 'Rotor (9 PWM levels)', \\
     f1(x) with lines lw 3 lc rgb '#004499' title 'Rotor (fitted)', \\
     'rotor_with_unit_data.txt' using 1:2 with points pt 9 ps 1.5 lc rgb '#FF3300' title 'Rotor+unit (3 PWM levels)', \\
     f2(x) with lines lw 3 lc rgb '#CC1100' title 'Rotor+unit (fitted)'

# Plot 2: Current with enhanced visibility for discrete points
set title 'Current vs PWM\\n(Note: Rotor+unit tested only at 3 PWM values)' font "Arial,13"
set xlabel 'PWM Value' font "Arial,12"
set ylabel 'Current (A)' font "Arial,12"
set grid
set key top left font "Arial,10"

# Define fitting functions for current
g1(x) = d1*x*x + e1*x + f1_c
g2(x) = d2*x*x + e2*x + f2_c

# Fit curves
fit g1(x) 'rotor_data.txt' using 1:3 via d1,e1,f1_c
fit g2(x) 'rotor_with_unit_data.txt' using 1:3 via d2,e2,f2_c

# Enhanced visibility - larger points for sparse data
plot 'rotor_data.txt' using 1:3 with points pt 6 ps 0.8 lc rgb '#0066CC' title 'Rotor (continuous)', \\
     g1(x) with lines lw 3 lc rgb '#004499' title 'Rotor (fitted)', \\
     'rotor_with_unit_data.txt' using 1:3 with points pt 8 ps 2.0 lc rgb '#FF3300' title 'Rotor+unit (3 points)', \\
     g2(x) with lines lw 3 lc rgb '#CC1100' title 'Rotor+unit (fitted)'

# Plot 3: RPM with enhanced visibility for discrete points
set title 'RPM vs PWM\\n(Rotor+unit: limited PWM test range)' font "Arial,13"
set xlabel 'PWM Value' font "Arial,12"
set ylabel 'RPM' font "Arial,12"
set grid
set key top left font "Arial,10"

# Define fitting functions for RPM
h1(x) = g1_r*x*x + h1_r*x + i1
h2(x) = g2_r*x*x + h2_r*x + i2

# Fit curves
fit h1(x) 'rotor_data.txt' using 1:4 via g1_r,h1_r,i1
fit h2(x) 'rotor_with_unit_data.txt' using 1:4 via g2_r,h2_r,i2

# Enhanced visibility - different point styles
plot 'rotor_data.txt' using 1:4 with points pt 5 ps 0.8 lc rgb '#0066CC' title 'Rotor (9 levels)', \\
     h1(x) with lines lw 3 lc rgb '#004499' title 'Rotor (fitted)', \\
     'rotor_with_unit_data.txt' using 1:4 with points pt 11 ps 2.0 lc rgb '#FF3300' title 'Rotor+unit (3 levels)', \\
     h2(x) with lines lw 3 lc rgb '#CC1100' title 'Rotor+unit (fitted)'

unset multiplot
"""
    
    with open('plot_comparison.gp', 'w') as f:
        f.write(gnuplot_script)
    
    print("Created files:")
    print("- rotor_data.txt (Rotor data, PWM 1100-1500)")
    print("- rotor_with_unit_data.txt (Rotor with unit data, PWM mapped to 1100-1400)")
    print("- plot_comparison.gp (gnuplot script with curve fitting)")
    print("\nTo generate the plot, run: gnuplot plot_comparison.gp")

def show_statistics(data1, data2):
    print("\n" + "="*50)
    print("DATA STATISTICS")
    print("="*50)
    
    # Extract columns
    force_z1 = [row[3] for row in data1]  # Force Z
    current1 = [row[8] for row in data1]  # Current
    rpm1 = [min(row[9], 14000) for row in data1]      # RPM (filtered > 14000)
    pwm1 = [row[0] for row in data1]      # PWM
    
    force_z2 = [row[3] for row in data2]  # Force Z
    current2 = [row[8] for row in data2]  # Current
    rpm2 = [min(row[9], 14000) for row in data2]      # RPM (filtered > 14000)
    pwm2 = [row[0] for row in data2]      # PWM
    
    # Count how many RPM values were capped
    rpm1_original = [row[9] for row in data1]
    rpm2_original = [row[9] for row in data2]
    rpm1_capped = sum(1 for rpm in rpm1_original if rpm > 14000)
    rpm2_capped = sum(1 for rpm in rpm2_original if rpm > 14000)
    
    print("ROTOR (File 1):")
    print(f"  Data points: {len(data1)}")
    print(f"  PWM range: {min(pwm1):.0f} to {max(pwm1):.0f}")
    print(f"  Force Z range: {min(force_z1):.4f} to {max(force_z1):.4f} N")
    print(f"  Force Z average: {sum(force_z1)/len(force_z1):.4f} N")
    print(f"  Current range: {min(current1):.4f} to {max(current1):.4f} A")
    print(f"  Current average: {sum(current1)/len(current1):.4f} A")
    print(f"  RPM range: {min(rpm1):.0f} to {max(rpm1):.0f} (filtered > 14000)")
    print(f"  RPM average: {sum(rpm1)/len(rpm1):.0f}")
    if rpm1_capped > 0:
        print(f"  ⚠️  {rpm1_capped} RPM values > 14000 were capped to 14000")
    
    print("\nROTOR WITH UNIT (File 2):")
    print(f"  Data points: {len(data2)}")
    print(f"  Original PWM range: {min(pwm2):.0f} to {max(pwm2):.0f}")
    # Calculate mapped PWM range
    mapped_pwm_min = 1100 + (min(pwm2) - 1550) * (1400-1100) / (1750-1550)
    mapped_pwm_max = 1100 + (max(pwm2) - 1550) * (1400-1100) / (1750-1550)
    print(f"  Mapped PWM range: {mapped_pwm_min:.0f} to {mapped_pwm_max:.0f} (aligned to Rotor)")
    # Apply -1 correction to Force Z for statistics
    corrected_force_z2 = [-fz for fz in force_z2]
    print(f"  Force Z range (original): {min(force_z2):.4f} to {max(force_z2):.4f} N")
    print(f"  Force Z range (corrected): {min(corrected_force_z2):.4f} to {max(corrected_force_z2):.4f} N")
    print(f"  Force Z average (corrected): {sum(corrected_force_z2)/len(corrected_force_z2):.4f} N")
    print(f"  Current range: {min(current2):.4f} to {max(current2):.4f} A")
    print(f"  Current average: {sum(current2)/len(current2):.4f} A")
    print(f"  RPM range: {min(rpm2):.0f} to {max(rpm2):.0f} (filtered > 15000)")
    print(f"  RPM average: {sum(rpm2)/len(rpm2):.0f}")
    if rpm2_capped > 0:
        print(f"  ⚠️  {rpm2_capped} RPM values > 15000 were capped to 15000")
    
    # Show sample data and PWM distribution analysis
    print("\nSAMPLE DATA:")
    print("Rotor (first 5 points):")
    print("PWM\tForce_Z\tCurrent\tRPM")
    for i in range(min(5, len(data1))):
        filtered_rpm = min(data1[i][9], 15000)
        print(f"{data1[i][0]:.0f}\t{data1[i][3]:.4f}\t{data1[i][8]:.4f}\t{filtered_rpm:.0f}")
    
    print("\nRotor with unit (first 5 points, with mapped PWM and corrected Force Z):")
    print("Orig_PWM\tMapped_PWM\tForce_Z_orig\tForce_Z_corr\tCurrent\tRPM")
    for i in range(min(5, len(data2))):
        original_pwm = data2[i][0]
        mapped_pwm = 1100 + (original_pwm - 1550) * (1400-1100) / (1750-1550)
        original_force_z = data2[i][3]
        corrected_force_z = -original_force_z
        filtered_rpm = min(data2[i][9], 15000)
        print(f"{original_pwm:.0f}\t\t{mapped_pwm:.0f}\t\t{original_force_z:.4f}\t\t{corrected_force_z:.4f}\t\t{data2[i][8]:.4f}\t{filtered_rpm:.0f}")

    # PWM Distribution Analysis
    print("\n" + "="*60)
    print("PWM DISTRIBUTION ANALYSIS")
    print("="*60)
    
    # Count PWM values for each dataset
    import collections
    pwm_counts1 = collections.Counter([row[0] for row in data1])
    pwm_counts2 = collections.Counter([row[0] for row in data2])
    
    print("ROTOR PWM Distribution:")
    for pwm in sorted(pwm_counts1.keys()):
        print(f"  PWM {pwm:.0f}: {pwm_counts1[pwm]} data points")
    
    print("\nROTOR WITH UNIT PWM Distribution (Original):")
    for pwm in sorted(pwm_counts2.keys()):
        mapped_pwm = 1100 + (pwm - 1550) * (1400-1100) / (1750-1550)
        print(f"  PWM {pwm:.0f} (mapped to {mapped_pwm:.0f}): {pwm_counts2[pwm]} data points")
    
    print(f"\n🔍 DATA DENSITY COMPARISON:")
    print(f"  Rotor: {len(pwm_counts1)} different PWM values")
    print(f"  Rotor with unit: {len(pwm_counts2)} different PWM values")
    print(f"  Rotor with unit has {len(pwm_counts1) - len(pwm_counts2)} fewer PWM test points!")
    
    if len(pwm_counts2) < len(pwm_counts1):
        print(f"\n⚠️  WARNING: This explains why Rotor with unit appears as discrete points")
        print(f"     while Rotor shows continuous curves in the plots!")
        print(f"     Rotor with unit was only tested at {len(pwm_counts2)} PWM values,")
        print(f"     while Rotor was tested at {len(pwm_counts1)} PWM values.")

    # Calculate current difference at PWM 1500 (Rotor) vs PWM 1750 (Rotor with unit)
    print("\n" + "="*70)
    print("CURRENT DIFFERENCE ANALYSIS AT PWM 1500 vs PWM 1750")
    print("="*70)
    print("Comparing equivalent power settings:")
    print("  - Rotor: PWM 1500 (maximum in range 1100-1500)")
    print("  - Rotor with unit: PWM 1750 (maximum in range 1550-1750)")
    print("  - Both represent maximum power output for each configuration")
    
    # Find data points at PWM 1500 for Rotor (exact match or within 1 PWM unit)
    rotor_1500_data = [row for row in data1 if abs(row[0] - 1500) < 1.0]
    
    # Find data points at PWM 1750 for Rotor with unit (exact match or within 1 PWM unit)
    rotor_with_unit_1750_data = [row for row in data2 if abs(row[0] - 1750) < 1.0]
    
    if rotor_1500_data and rotor_with_unit_1750_data:
        # Calculate statistics for Rotor at PWM 1500
        rotor_1500_currents = [row[8] for row in rotor_1500_data]
        rotor_1500_avg_current = sum(rotor_1500_currents) / len(rotor_1500_currents)
        rotor_1500_min_current = min(rotor_1500_currents)
        rotor_1500_max_current = max(rotor_1500_currents)
        
        # Calculate statistics for Rotor with unit at PWM 1750
        rotor_with_unit_1750_currents = [row[8] for row in rotor_with_unit_1750_data]
        rotor_with_unit_1750_avg_current = sum(rotor_with_unit_1750_currents) / len(rotor_with_unit_1750_currents)
        rotor_with_unit_1750_min_current = min(rotor_with_unit_1750_currents)
        rotor_with_unit_1750_max_current = max(rotor_with_unit_1750_currents)
        
        # Calculate difference
        current_difference = rotor_with_unit_1750_avg_current - rotor_1500_avg_current
        percentage_difference = (current_difference / rotor_1500_avg_current) * 100
        
        print(f"\n📊 ROTOR at PWM 1500:")
        print(f"  Data points: {len(rotor_1500_data)}")
        print(f"  Average current: {rotor_1500_avg_current:.4f} A")
        print(f"  Current range: {rotor_1500_min_current:.4f} to {rotor_1500_max_current:.4f} A")
        print(f"  Standard deviation: {(sum([(i - rotor_1500_avg_current)**2 for i in rotor_1500_currents]) / len(rotor_1500_currents))**0.5:.4f} A")
        
        print(f"\n📊 ROTOR WITH UNIT at PWM 1750:")
        print(f"  Data points: {len(rotor_with_unit_1750_data)}")
        print(f"  Average current: {rotor_with_unit_1750_avg_current:.4f} A")
        print(f"  Current range: {rotor_with_unit_1750_min_current:.4f} to {rotor_with_unit_1750_max_current:.4f} A")
        print(f"  Standard deviation: {(sum([(i - rotor_with_unit_1750_avg_current)**2 for i in rotor_with_unit_1750_currents]) / len(rotor_with_unit_1750_currents))**0.5:.4f} A")
        
        print(f"\n🔥 CURRENT DIFFERENCE ANALYSIS:")
        print(f"  Absolute difference: {current_difference:.4f} A")
        print(f"  Percentage difference: {percentage_difference:.2f}%")
        print(f"  (Rotor_with_unit - Rotor) / Rotor * 100%")
        
        if current_difference > 0:
            print(f"  ⚡ Rotor with unit consumes {abs(current_difference):.4f} A MORE than Rotor")
            print(f"     This represents a {percentage_difference:.2f}% INCREASE in current consumption")
        else:
            print(f"  💡 Rotor with unit consumes {abs(current_difference):.4f} A LESS than Rotor")
            print(f"     This represents a {abs(percentage_difference):.2f}% DECREASE in current consumption")
            
        # Also show Force Z and RPM at these PWM values for context
        rotor_1500_force_z = [row[3] for row in rotor_1500_data]
        rotor_1500_rpm = [row[9] for row in rotor_1500_data]
        rotor_with_unit_1750_force_z = [-row[3] for row in rotor_with_unit_1750_data]  # Apply correction
        rotor_with_unit_1750_rpm = [row[9] for row in rotor_with_unit_1750_data]
        
        print(f"\n📈 PERFORMANCE COMPARISON AT MAXIMUM PWM:")
        print(f"  Rotor (PWM 1500):")
        print(f"    Average Force Z: {sum(rotor_1500_force_z)/len(rotor_1500_force_z):.4f} N")
        print(f"    Average RPM: {sum(rotor_1500_rpm)/len(rotor_1500_rpm):.0f}")
        print(f"  Rotor with unit (PWM 1750):")
        print(f"    Average Force Z: {sum(rotor_with_unit_1750_force_z)/len(rotor_with_unit_1750_force_z):.4f} N")
        print(f"    Average RPM: {sum(rotor_with_unit_1750_rpm)/len(rotor_with_unit_1750_rpm):.0f}")
        
    else:
        print("❌ No data found at specified PWM values for comparison")
        print(f"Rotor data points at PWM 1500: {len(rotor_1500_data) if 'rotor_1500_data' in locals() else 0}")
        print(f"Rotor with unit data points at PWM 1750: {len(rotor_with_unit_1750_data) if 'rotor_with_unit_1750_data' in locals() else 0}")

if __name__ == "__main__":
    analyze_valid_data()
