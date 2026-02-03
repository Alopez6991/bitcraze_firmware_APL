#!/usr/bin/env python3
"""
Plot script for USD log data.
Shows EKF measured vs predicted values and gyro data to identify timing issues.
"""

import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
import sys
import os

def plot_ekf_timing(csv_file: str, df: pd.DataFrame, time_s: pd.Series):
    """Plot EKF timing analysis (measured vs predicted, innovation, gyro)."""
    
    # Create figure with 4 subplots
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    
    # Plot 1: EKF Measured vs Predicted - X axis
    ax1 = axes[0]
    ax1.plot(time_s, df['kalman_pred.measNX'], label='Measured NX', color='tab:blue', alpha=0.8)
    ax1.plot(time_s, df['kalman_pred.predNX'], label='Predicted NX', color='tab:red', alpha=0.8, linestyle='--')
    ax1.set_ylabel('Flow X (normalized)')
    ax1.set_title('EKF Flow X: Measured vs Predicted')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: EKF Measured vs Predicted - Y axis
    ax2 = axes[1]
    ax2.plot(time_s, df['kalman_pred.measNY'], label='Measured NY', color='tab:blue', alpha=0.8)
    ax2.plot(time_s, df['kalman_pred.predNY'], label='Predicted NY', color='tab:red', alpha=0.8, linestyle='--')
    ax2.set_ylabel('Flow Y (normalized)')
    ax2.set_title('EKF Flow Y: Measured vs Predicted')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Innovation (residual) - difference between measured and predicted
    ax3 = axes[2]
    innovation_x = df['kalman_pred.measNX'] - df['kalman_pred.predNX']
    innovation_y = df['kalman_pred.measNY'] - df['kalman_pred.predNY']
    ax3.plot(time_s, innovation_x, label='Innovation X', color='tab:blue', alpha=0.8)
    ax3.plot(time_s, innovation_y, label='Innovation Y', color='tab:orange', alpha=0.8)
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax3.set_ylabel('Innovation (meas - pred)')
    ax3.set_title('EKF Innovation (Residual) - Timing issues show as systematic bias')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Gyro data (X, Y, Z)
    ax4 = axes[3]
    ax4.plot(time_s, df['gyro.x'], label='Gyro X', color='tab:blue', alpha=0.7)
    ax4.plot(time_s, df['gyro.y'], label='Gyro Y', color='tab:orange', alpha=0.7)
    ax4.plot(time_s, df['gyro.z'], label='Gyro Z', color='tab:green', alpha=0.7)
    ax4.set_ylabel('Angular rate (deg/s)')
    ax4.set_xlabel('Time (s)')
    ax4.set_title('Gyroscope Data - Compare with innovation for timing correlation')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = os.path.splitext(csv_file)[0] + '_ekf_timing.png'
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to: {output_path}")


def plot_ekf_velocity(csv_file: str, df: pd.DataFrame, time_s: pd.Series):
    """Plot EKF measured vs predicted alongside velocity estimates and yaw rate."""
    
    # Create figure with 5 subplots
    fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
    
    # Plot 1: EKF Measured vs Predicted - X axis
    ax1 = axes[0]
    ax1.plot(time_s, df['kalman_pred.measNX'], label='Measured NX', color='tab:blue', alpha=0.8)
    ax1.plot(time_s, df['kalman_pred.predNX'], label='Predicted NX', color='tab:red', alpha=0.8, linestyle='--')
    ax1.set_ylabel('Flow X (normalized)')
    ax1.set_title('EKF Flow X: Measured vs Predicted')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: EKF Measured vs Predicted - Y axis
    ax2 = axes[1]
    ax2.plot(time_s, df['kalman_pred.measNY'], label='Measured NY', color='tab:blue', alpha=0.8)
    ax2.plot(time_s, df['kalman_pred.predNY'], label='Predicted NY', color='tab:red', alpha=0.8, linestyle='--')
    ax2.set_ylabel('Flow Y (normalized)')
    ax2.set_title('EKF Flow Y: Measured vs Predicted')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Estimated Velocity X
    ax3 = axes[2]
    ax3.plot(time_s, df['kalman.statePX'], label='Velocity X (statePX)', color='tab:blue', alpha=0.8)
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax3.set_ylabel('Velocity X (m/s)')
    ax3.set_title('Estimated Velocity X (Kalman)')
    ax3.legend(loc='upper right')
    ax3.set_ylim([-0.5, 0.5])
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Estimated Velocity Y
    ax4 = axes[3]
    ax4.plot(time_s, df['kalman.statePY'], label='Velocity Y (statePY)', color='tab:orange', alpha=0.8)
    ax4.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax4.set_ylabel('Velocity Y (m/s)')
    ax4.set_title('Estimated Velocity Y (Kalman)')
    ax4.legend(loc='upper right')
    ax4.set_ylim([-0.5, 0.5])
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Gyro Z (yaw rate) - filtered
    ax5 = axes[4]
    gyro_z_filtered = uniform_filter1d(df['gyro.z'].values, size=15)  # Moving average filter
    ax5.plot(time_s, df['gyro.z'], label='Gyro Z (raw)', color='tab:green', alpha=0.3)
    ax5.plot(time_s, gyro_z_filtered, label='Gyro Z (filtered)', color='tab:green', alpha=0.9, linewidth=1.5)
    ax5.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax5.set_ylabel('Yaw rate (deg/s)')
    ax5.set_xlabel('Time (s)')
    ax5.set_title('Gyroscope Z - Yaw Rate (filtered)')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = os.path.splitext(csv_file)[0] + '_ekf_velocity.png'
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to: {output_path}")


def plot_log_data(csv_file: str):
    """Load CSV and create multi-panel plots of sensor data."""
    
    # Load the data
    df = pd.read_csv(csv_file)
    
    # Convert timestamp from ms to seconds for better readability
    time_s = (df['timestamp'] - df['timestamp'].iloc[0]) / 1000.0
    
    # Generate both plots
    plot_ekf_timing(csv_file, df, time_s)
    plot_ekf_velocity(csv_file, df, time_s)
    
    plt.show()


if __name__ == '__main__':
    # Default to log01.csv in the same directory
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file = os.path.join(script_dir, 'log01.csv')
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    print(f"Loading: {csv_file}")
    plot_log_data(csv_file)
