#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

# Try to use a scientific plotting style
try:
    plt.style.use('seaborn-v0_8')
except:
    try:
        plt.style.use('seaborn')
    except:
        pass  # Use default style

def load_data(file_path):
    """
    Load motor test data from file
    Columns: pwm_value, force.x, force.y, force.z, force_norm, torque.x, torque.y, torque.z, current, rpm, temperature, voltage, status
    """
    if not os.path.exists(file_path):
        print(f"Warning: File {file_path} not found!")
        return None
    
    try:
        # Read file line by line to handle mixed data types
        valid_data = []
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and line != "done":
                    parts = line.split()
                    if len(parts) >= 13:  # Ensure we have all columns including status
                        try:
                            # Extract numeric values (first 12 columns)
                            numeric_values = [float(x) for x in parts[:12]]
                            status = parts[12]  # Status is the last column
                            
                            # Only keep valid data
                            if status == "valid":
                                valid_data.append(numeric_values)
                        except ValueError:
                            continue  # Skip lines with invalid numeric data
        
        if valid_data:
            return np.array(valid_data)
        else:
            print(f"No valid data found in {file_path}")
            return None
            
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        return None

def filter_pwm_range(data, pwm_min, pwm_max):
    """Filter data by PWM range"""
    if data is None:
        return None
    pwm_col = data[:, 0]  # PWM is first column
    mask = (pwm_col >= pwm_min) & (pwm_col <= pwm_max)
    return data[mask]

def plot_comparison():
    # File paths
    file1 = "/home/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/u=25.20v_motor_test_1706621604.txt"
    file2 = "/home/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/motor_test_aerialWithunit1753271753.txt"
    
    # Load data
    print("Loading data files...")
    data1 = load_data(file1)
    data2 = load_data(file2)
    
    if data1 is None or data2 is None:
        print("Failed to load one or both data files!")
        return
    
    print(f"Data1 valid points: {data1.shape[0]}")
    print(f"Data2 valid points: {data2.shape[0]}")
    
    # Filter data for different PWM ranges
    print("Filtering data...")
    print("File 1: PWM range 1100-1500")
    print("File 2: PWM range 1550-1750")
    data1_filtered = filter_pwm_range(data1, 1100, 1500)
    data2_filtered = filter_pwm_range(data2, 1550, 1750)
    
    if data1_filtered is None or data2_filtered is None or len(data1_filtered) == 0 or len(data2_filtered) == 0:
        print("No data in the specified PWM ranges!")
        print(f"Data1 filtered points: {len(data1_filtered) if data1_filtered is not None else 0}")
        print(f"Data2 filtered points: {len(data2_filtered) if data2_filtered is not None else 0}")
        return

    print(f"Data1 filtered points: {len(data1_filtered)}")
    print(f"Data2 filtered points: {len(data2_filtered)}")
    
    # Create figure with 3 horizontal subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Motor Test Data Comparison: Rotor vs Rotor_with_unit', fontsize=16, fontweight='bold')
    
    # Colors for the plots
    color1 = '#2E86AB'  # Blue
    color2 = '#A23B72'  # Red
    
    # Plot 1: Force Z comparison (PWM 1100-1500)
    print("Creating Force Z comparison plot...")
    pwm1 = data1_filtered[:, 0]  # PWM values
    force_z1 = data1_filtered[:, 3]  # Force Z (column 3)
    pwm2 = data2_filtered[:, 0]
    force_z2 = data2_filtered[:, 3]
    
    axes[0].scatter(pwm1, force_z1, label='Rotor', alpha=0.7, color=color1, s=20)
    axes[0].scatter(pwm2, force_z2, label='Rotor_with_unit', alpha=0.7, color=color2, s=20)
    axes[0].set_xlabel('PWM Value')
    axes[0].set_ylabel('Force Z (N)')
    axes[0].set_title('Force Z vs PWM (1100-1500)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Current comparison (PWM 1100-1500)
    print("Creating Current comparison plot...")
    current1 = data1_filtered[:, 8]  # Current (column 8)
    current2 = data2_filtered[:, 8]
    
    axes[1].scatter(pwm1, current1, label='Rotor', alpha=0.7, color=color1, s=20)
    axes[1].scatter(pwm2, current2, label='Rotor_with_unit', alpha=0.7, color=color2, s=20)
    axes[1].set_xlabel('PWM Value')
    axes[1].set_ylabel('Current (A)')
    axes[1].set_title('Current vs PWM (1100-1500)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: RPM comparison (PWM 1100-1500)
    print("Creating RPM comparison plot...")
    rpm1 = data1_filtered[:, 9]  # RPM (column 9)
    rpm2 = data2_filtered[:, 9]
    
    axes[2].scatter(pwm1, rpm1, label='Rotor', alpha=0.7, color=color1, s=20)
    axes[2].scatter(pwm2, rpm2, label='Rotor_with_unit', alpha=0.7, color=color2, s=20)
    axes[2].set_xlabel('PWM Value')
    axes[2].set_ylabel('RPM')
    axes[2].set_title('RPM vs PWM (1100-1500)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # Adjust layout
    plt.tight_layout()
    
    # Save the plot
    output_path = "/home/chen/Research/jsk_aerial_robot/src/jsk_aerial_robot_dev/aerial_robot_nerve/motor_test/data/motor_comparison.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    
    # Show statistics
    print("\n=== Data Statistics ===")
    print("Rotor (File 1):")
    print(f"  Data points in range: {len(data1_filtered)}")
    print(f"  Force Z range: {force_z1.min():.2f} to {force_z1.max():.2f} N")
    print(f"  Current range: {current1.min():.2f} to {current1.max():.2f} A")
    print(f"  RPM range: {rpm1.min():.0f} to {rpm1.max():.0f}")
    
    print("\nRotor_with_unit (File 2):")
    print(f"  Data points in range: {len(data2_filtered)}")
    print(f"  Force Z range: {force_z2.min():.2f} to {force_z2.max():.2f} N")
    print(f"  Current range: {current2.min():.2f} to {current2.max():.2f} A")
    print(f"  RPM range: {rpm2.min():.0f} to {rpm2.max():.0f}")
    
    # Show the plot
    plt.show()

if __name__ == "__main__":
    plot_comparison()
