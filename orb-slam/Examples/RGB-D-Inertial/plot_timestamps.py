import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def analyze_rgbd_inertial_timestamps(csv_file='timestamp_log.csv'):
    """
    Analyze timestamps from RGBD camera and inertial sensors
    Expected CSV format: timestamp, type
    Types: 0=color/rgb, 1=depth, 2=accel, 3=gyro (or adjust based on your data)
    """
    
    # Read the CSV file
    try:
        df = pd.read_csv(csv_file)
        print(f"Successfully loaded {len(df)} records from {csv_file}")
        print("CSV columns:", df.columns.tolist())
        print("First few rows:")
        print(df.head())
        print("\nData type distribution:")
        print(df['type'].value_counts().sort_index())
    except FileNotFoundError:
        print(f"Error: Could not find {csv_file}")
        return
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return
    
    # Convert timestamp to numeric, handling any string issues
    df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
    
    # Check for any failed conversions
    nan_count = df['timestamp'].isna().sum()
    if nan_count > 0:
        print(f"Warning: {nan_count} timestamps could not be converted to numbers")
        df = df.dropna(subset=['timestamp'])  # Remove rows with invalid timestamps
    
    # Separate data by type - adjust these mappings based on your data format
    color_data = df[df['type'] == 0]['timestamp'].reset_index(drop=True)
    depth_data = df[df['type'] == 1]['timestamp'].reset_index(drop=True)
    accel_data = df[df['type'] == 2]['timestamp'].reset_index(drop=True)
    gyro_data = df[df['type'] == 3]['timestamp'].reset_index(drop=True)
    
    # If you have different type mappings, uncomment and adjust:
    # color_data = df[df['type'] == 'rgb']['timestamp'].reset_index(drop=True)
    # depth_data = df[df['type'] == 'depth']['timestamp'].reset_index(drop=True)
    # accel_data = df[df['type'] == 'accel']['timestamp'].reset_index(drop=True)
    # gyro_data = df[df['type'] == 'gyro']['timestamp'].reset_index(drop=True)
    
    print(f"\nData summary:")
    print(f"Color frames: {len(color_data)}")
    print(f"Depth frames: {len(depth_data)}")
    print(f"Accel samples: {len(accel_data)}")
    print(f"Gyro samples: {len(gyro_data)}")
    
    if len(color_data) == 0 and len(depth_data) == 0 and len(accel_data) == 0 and len(gyro_data) == 0:
        print("No data found! Check your CSV file format.")
        return
    
    # Create the main visualization
    plt.figure(figsize=(15, 12))
    
    # Plot 1: Raw timestamps over time - separate scales for visibility
    plt.subplot(3, 2, 1)
    
    # Plot camera data first (lower frequency, more visible)
    if not color_data.empty:
        plt.plot(range(len(color_data)), color_data, 'bo-', label='Color', linewidth=2, markersize=4, alpha=0.8)
    if not depth_data.empty:
        plt.plot(range(len(depth_data)), depth_data, 'rs-', label='Depth', linewidth=2, markersize=4, alpha=0.8)
    
    # Sample IMU data for visibility (every 50th point)
    if not accel_data.empty:
        accel_sample = accel_data[::50]  # Every 50th point
        plt.plot(range(0, len(accel_data), 50), accel_sample, 'g^', label='IMU Accel (sampled)', markersize=3, alpha=0.7)
    if not gyro_data.empty:
        gyro_sample = gyro_data[::50]  # Every 50th point
        plt.plot(range(0, len(gyro_data), 50), gyro_sample, 'mv', label='IMU Gyro (sampled)', markersize=3, alpha=0.7)
    
    plt.xlabel('Sample Number')
    plt.ylabel('Timestamp (seconds)')
    plt.title('RGBD + Inertial Raw Timestamps')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 2: Frame rates over time - fix the missing IMU rates
    plt.subplot(3, 2, 2)
    if len(color_data) > 1:
        color_diff = color_data.diff().dropna()
        color_rates = 1.0 / color_diff
        # Remove outliers (rates > 200 Hz or < 1 Hz)
        color_rates = color_rates[(color_rates >= 1) & (color_rates <= 200)]
        plt.plot(range(len(color_rates)), color_rates, 'b-', label=f'Color (avg: {np.mean(color_rates):.1f} Hz)', linewidth=2)
    
    if len(depth_data) > 1:
        depth_diff = depth_data.diff().dropna()
        depth_rates = 1.0 / depth_diff
        depth_rates = depth_rates[(depth_rates >= 1) & (depth_rates <= 200)]
        plt.plot(range(len(depth_rates)), depth_rates, 'r-', label=f'Depth (avg: {np.mean(depth_rates):.1f} Hz)', linewidth=2)
    
    # Sample IMU rates for visibility
    if len(accel_data) > 1:
        accel_diff = accel_data.diff().dropna()
        accel_rates = 1.0 / accel_diff
        accel_rates = accel_rates[(accel_rates >= 1) & (accel_rates <= 2000)]
        # Sample every 100th point for visibility
        accel_sample = accel_rates[::100]
        plt.plot(range(0, len(accel_rates), 100), accel_sample, 'g^', label=f'Accel (avg: {np.mean(accel_rates):.0f} Hz)', markersize=4, alpha=0.7)
    
    if len(gyro_data) > 1:
        gyro_diff = gyro_data.diff().dropna()
        gyro_rates = 1.0 / gyro_diff
        gyro_rates = gyro_rates[(gyro_rates >= 1) & (gyro_rates <= 2000)]
        # Sample every 100th point for visibility
        gyro_sample = gyro_rates[::100]
        plt.plot(range(0, len(gyro_rates), 100), gyro_sample, 'mv', label=f'Gyro (avg: {np.mean(gyro_rates):.0f} Hz)', markersize=4, alpha=0.7)
    
    plt.xlabel('Sample Number')
    plt.ylabel('Rate (Hz)')
    plt.title('Sampling Rates Over Time')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim(0, 200)  # Focus on reasonable frame rates
    
    # Plot 3: Time synchronization analysis - fix the weird offset calculation
    plt.subplot(3, 2, 3)
    if not color_data.empty and not depth_data.empty:
        # Find overlapping time windows for proper sync analysis
        color_start, color_end = color_data.min(), color_data.max()
        depth_start, depth_end = depth_data.min(), depth_data.max()
        
        # Use the overlapping time window
        sync_start = max(color_start, depth_start)
        sync_end = min(color_end, depth_end)
        
        # Filter data to overlapping window
        color_sync = color_data[(color_data >= sync_start) & (color_data <= sync_end)]
        depth_sync = depth_data[(depth_data >= sync_start) & (depth_data <= sync_end)]
        
        if len(color_sync) > 10 and len(depth_sync) > 10:
            # Find closest timestamps for sync analysis
            min_len = min(len(color_sync), len(depth_sync))
            if min_len > 1:
                sync_diff = (color_sync.iloc[:min_len].values - depth_sync.iloc[:min_len].values) * 1000
                plt.plot(range(min_len), sync_diff, 'purple', label=f'Color-Depth Sync (mean: {np.mean(sync_diff):.1f}ms)')
                plt.axhline(y=0, color='black', linestyle='--', alpha=0.5)
                plt.ylabel('Time Difference (ms)')
                plt.title('RGBD Synchronization (Overlapping Window)')
            else:
                plt.text(0.5, 0.5, 'Insufficient overlapping data\nfor sync analysis', 
                        transform=plt.gca().transAxes, ha='center', va='center')
        else:
            plt.text(0.5, 0.5, f'No overlapping time window\nColor: {color_start:.1f}-{color_end:.1f}s\nDepth: {depth_start:.1f}-{depth_end:.1f}s', 
                    transform=plt.gca().transAxes, ha='center', va='center')
    else:
        plt.text(0.5, 0.5, 'Missing color or depth data', 
                transform=plt.gca().transAxes, ha='center', va='center')
    
    # Plot 4: Timestamp intervals histogram - separate camera and IMU scales
    plt.subplot(3, 2, 4)
    
    # Camera intervals on reasonable scale
    if len(color_data) > 1:
        color_intervals = color_data.diff().dropna() * 1000  # Convert to ms
        color_intervals = color_intervals[(color_intervals > 0) & (color_intervals < 100)]  # Remove outliers
        plt.hist(color_intervals, bins=30, alpha=0.6, label=f'Color (μ={np.mean(color_intervals):.1f}ms)', color='blue', density=True)
    
    if len(depth_data) > 1:
        depth_intervals = depth_data.diff().dropna() * 1000
        depth_intervals = depth_intervals[(depth_intervals > 0) & (depth_intervals < 100)]
        plt.hist(depth_intervals, bins=30, alpha=0.6, label=f'Depth (μ={np.mean(depth_intervals):.1f}ms)', color='red', density=True)
    
    plt.xlabel('Frame Interval (ms)')
    plt.ylabel('Normalized Count')
    plt.title('Camera Frame Interval Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlim(0, 50)  # Focus on reasonable camera intervals
    
    # Plot 5: IMU data rate analysis - separate plot with proper scaling
    plt.subplot(3, 2, 5)
    if len(accel_data) > 1:
        accel_intervals = accel_data.diff().dropna() * 1000
        accel_intervals = accel_intervals[(accel_intervals > 0) & (accel_intervals < 10)]  # Remove outliers, focus on 0-10ms
        plt.hist(accel_intervals, bins=50, alpha=0.6, label=f'Accel (μ={np.mean(accel_intervals):.2f}ms)', color='green', density=True)
    
    if len(gyro_data) > 1:
        gyro_intervals = gyro_data.diff().dropna() * 1000
        gyro_intervals = gyro_intervals[(gyro_intervals > 0) & (gyro_intervals < 10)]
        plt.hist(gyro_intervals, bins=50, alpha=0.6, label=f'Gyro (μ={np.mean(gyro_intervals):.2f}ms)', color='magenta', density=True)
    
    plt.xlabel('Sample Interval (ms)')
    plt.ylabel('Normalized Count')
    plt.title('IMU Sample Interval Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlim(0, 5)  # Focus on expected IMU intervals (0-5ms for 200-1000Hz)
    
    # Plot 6: Timeline comparison - improved with proper legend and sampling
    plt.subplot(3, 2, 6)
    all_data = []
    
    # Add camera data
    if not color_data.empty:
        for i, ts in enumerate(color_data):
            all_data.append((ts, 'Color', i))
    if not depth_data.empty:
        for i, ts in enumerate(depth_data):
            all_data.append((ts, 'Depth', i))
    
    # Sample IMU data for visibility (every 100th point)
    if not accel_data.empty:
        for i in range(0, len(accel_data), 100):  # Every 100th point
            all_data.append((accel_data.iloc[i], 'Accel', i))
    if not gyro_data.empty:
        for i in range(0, len(gyro_data), 100):
            all_data.append((gyro_data.iloc[i], 'Gyro', i))
    
    if all_data:
        all_data.sort(key=lambda x: x[0])  # Sort by timestamp
        colors = {'Color': 'blue', 'Depth': 'red', 'Accel': 'green', 'Gyro': 'magenta'}
        y_positions = {'Color': 3, 'Depth': 2, 'Accel': 1, 'Gyro': 0}
        
        # Plot first 500 points for visibility
        for ts, data_type, idx in all_data[:500]:
            plt.scatter(ts, y_positions[data_type], c=colors[data_type], s=15, alpha=0.7)
    
    plt.xlabel('Timestamp (seconds)')
    plt.ylabel('Data Type')
    plt.yticks(range(4), ['Gyro', 'Accel', 'Depth', 'Color'])
    plt.title('Data Timeline (sampled for visibility)')
    plt.grid(True, alpha=0.3)
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor='blue', label='Color'),
                      Patch(facecolor='red', label='Depth'),
                      Patch(facecolor='green', label='Accel (sampled)'),
                      Patch(facecolor='magenta', label='Gyro (sampled)')]
    plt.legend(handles=legend_elements, loc='upper left')
    
    plt.tight_layout()
    
    # Save the plot
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'rgbd_inertial_analysis_{timestamp_str}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved as: {filename}")
    plt.show()
    
    # Print detailed statistics
    print("\n=== DETAILED TIMESTAMP ANALYSIS ===")
    
    if not color_data.empty:
        print(f"\nColor Camera:")
        print(f"  Samples: {len(color_data)}")
        print(f"  Time range: {color_data.min():.3f} - {color_data.max():.3f} seconds")
        print(f"  Duration: {color_data.max() - color_data.min():.3f} seconds")
        if len(color_data) > 1:
            avg_rate = 1.0 / np.mean(color_data.diff().dropna())
            std_interval = np.std(color_data.diff().dropna()) * 1000
            print(f"  Average rate: {avg_rate:.1f} Hz")
            print(f"  Interval std dev: {std_interval:.2f} ms")
    
    if not depth_data.empty:
        print(f"\nDepth Camera:")
        print(f"  Samples: {len(depth_data)}")
        print(f"  Time range: {depth_data.min():.3f} - {depth_data.max():.3f} seconds")
        print(f"  Duration: {depth_data.max() - depth_data.min():.3f} seconds")
        if len(depth_data) > 1:
            avg_rate = 1.0 / np.mean(depth_data.diff().dropna())
            std_interval = np.std(depth_data.diff().dropna()) * 1000
            print(f"  Average rate: {avg_rate:.1f} Hz")
            print(f"  Interval std dev: {std_interval:.2f} ms")
    
    if not accel_data.empty:
        print(f"\nAccelerometer:")
        print(f"  Samples: {len(accel_data)}")
        print(f"  Time range: {accel_data.min():.3f} - {accel_data.max():.3f} seconds")
        print(f"  Duration: {accel_data.max() - accel_data.min():.3f} seconds")
        if len(accel_data) > 1:
            avg_rate = 1.0 / np.mean(accel_data.diff().dropna())
            std_interval = np.std(accel_data.diff().dropna()) * 1000
            print(f"  Average rate: {avg_rate:.1f} Hz")
            print(f"  Interval std dev: {std_interval:.2f} ms")
    
    if not gyro_data.empty:
        print(f"\nGyroscope:")
        print(f"  Samples: {len(gyro_data)}")
        print(f"  Time range: {gyro_data.min():.3f} - {gyro_data.max():.3f} seconds")
        print(f"  Duration: {gyro_data.max() - gyro_data.min():.3f} seconds")
        if len(gyro_data) > 1:
            avg_rate = 1.0 / np.mean(gyro_data.diff().dropna())
            std_interval = np.std(gyro_data.diff().dropna()) * 1000
            print(f"  Average rate: {avg_rate:.1f} Hz")
            print(f"  Interval std dev: {std_interval:.2f} ms")
    
    # Synchronization analysis
    if not color_data.empty and not depth_data.empty:
        min_len = min(len(color_data), len(depth_data))
        if min_len > 1:
            sync_diff = color_data[:min_len] - depth_data[:min_len]
            print(f"\nRGBD Synchronization:")
            print(f"  Mean offset: {np.mean(sync_diff)*1000:.2f} ms")
            print(f"  Std deviation: {np.std(sync_diff)*1000:.2f} ms")
            print(f"  Max offset: {np.max(np.abs(sync_diff))*1000:.2f} ms")

# Run the analysis
if __name__ == "__main__":
    # You can change the CSV filename here
    analyze_rgbd_inertial_timestamps('timestamp_log.csv')