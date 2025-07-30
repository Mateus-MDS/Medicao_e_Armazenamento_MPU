import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV file from microcontroller
filename = 'mpu_data.csv'

try:
    # Read CSV data
    data = pd.read_csv(filename)
    print(f"Data loaded: {len(data)} samples")
    
    # Calculate time (10Hz = 0.1s per sample)
    time = data['Sample'] * 0.1
    
    # Create 4 subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('MPU6050 Data Analysis', fontsize=16, fontweight='bold')
    
    # Plot 1: Acceleration
    axes[0, 0].plot(time, data['AccelX'], 'r-', label='AccelX', linewidth=1.5)
    axes[0, 0].plot(time, data['AccelY'], 'g-', label='AccelY', linewidth=1.5)
    axes[0, 0].plot(time, data['AccelZ'], 'b-', label='AccelZ', linewidth=1.5)
    axes[0, 0].set_title('Acceleration (g)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Acceleration (g)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Gyroscope
    axes[0, 1].plot(time, data['GyroX'], 'r-', label='GyroX', linewidth=1.5)
    axes[0, 1].plot(time, data['GyroY'], 'g-', label='GyroY', linewidth=1.5)
    axes[0, 1].plot(time, data['GyroZ'], 'b-', label='GyroZ', linewidth=1.5)
    axes[0, 1].set_title('Gyroscope (deg/s)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Angular Velocity (deg/s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Roll and Pitch angles
    axes[1, 0].plot(time, data['Roll'], 'purple', label='Roll', linewidth=2)
    axes[1, 0].plot(time, data['Pitch'], 'orange', label='Pitch', linewidth=2)
    axes[1, 0].set_title('Orientation Angles (deg)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Angle (deg)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 4: Acceleration magnitude
    accel_magnitude = np.sqrt(data['AccelX']**2 + data['AccelY']**2 + data['AccelZ']**2)
    axes[1, 1].plot(time, accel_magnitude, 'black', linewidth=2, label='Magnitude')
    axes[1, 1].axhline(y=1.0, color='red', linestyle='--', alpha=0.7, label='1g reference')
    axes[1, 1].set_title('Acceleration Magnitude')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Magnitude (g)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Show statistics
    print(f"Duration: {time.iloc[-1]:.1f} seconds")
    print(f"Sample rate: {len(data)/time.iloc[-1]:.1f} Hz")
    
    print("\nAcceleration statistics (g):")
    print(f"  X: min={data['AccelX'].min():.3f}, max={data['AccelX'].max():.3f}, avg={data['AccelX'].mean():.3f}")
    print(f"  Y: min={data['AccelY'].min():.3f}, max={data['AccelY'].max():.3f}, avg={data['AccelY'].mean():.3f}")
    print(f"  Z: min={data['AccelZ'].min():.3f}, max={data['AccelZ'].max():.3f}, avg={data['AccelZ'].mean():.3f}")
    
    print("\nGyroscope statistics (deg/s):")
    print(f"  X: min={data['GyroX'].min():.1f}, max={data['GyroX'].max():.1f}, avg={data['GyroX'].mean():.1f}")
    print(f"  Y: min={data['GyroY'].min():.1f}, max={data['GyroY'].max():.1f}, avg={data['GyroY'].mean():.1f}")
    print(f"  Z: min={data['GyroZ'].min():.1f}, max={data['GyroZ'].max():.1f}, avg={data['GyroZ'].mean():.1f}")
    
    print("\nAngle statistics (deg):")
    print(f"  Roll:  min={data['Roll'].min():.1f}, max={data['Roll'].max():.1f}, avg={data['Roll'].mean():.1f}")
    print(f"  Pitch: min={data['Pitch'].min():.1f}, max={data['Pitch'].max():.1f}, avg={data['Pitch'].mean():.1f}")
    
    # Show plot
    plt.show()
    
    # Save figure
    plt.savefig('mpu6050_analysis.png', dpi=300, bbox_inches='tight')
    print("\nPlot saved as: mpu6050_analysis.png")

except FileNotFoundError:
    print(f"ERROR: File '{filename}' not found!")
    print("\nMake sure to:")
    print("1. Transfer CSV file from microcontroller using 't' command")
    print("2. Copy data between markers and save as 'mpu_data.csv'")
    print("3. Put file in same folder as this script")
    
except KeyError as e:
    print(f"ERROR: Column not found: {e}")
    print("Check if CSV has correct header:")
    print("Sample,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Roll,Pitch")
    
except Exception as e:
    print(f"ERROR: {e}")

print("\nUSAGE INSTRUCTIONS:")
print("1. On microcontroller: press 't' -> type 'mpu_data.csv'")
print("2. Copy data between markers")
print("3. Paste in text file and save as 'mpu_data.csv'")
print("4. Run: python PlotaDados.py")