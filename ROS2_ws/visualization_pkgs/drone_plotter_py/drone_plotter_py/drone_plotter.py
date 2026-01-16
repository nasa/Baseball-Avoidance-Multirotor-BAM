#!/usr/bin/env python3
"""
eVTOL Trajectory Visualization Node
----------------------------------
This node subscribes to the '/pub_pose' topic and provides an interactive 3D
visualization of the eVTOL's trajectory and its current orientation.

It uses rclpy for ROS2 integration and matplotlib for plotting. The orientation
is computed from the quaternion (assuming the eVTOL's forward direction is along
the x-axis in its body frame), and is visualized as an arrow at the latest position.

Authors:
    Newton Campbell
    Tenavi Nakamura-Zimmerer
History:
    2025-03-23: Created (NC)
    2025-07-21: Corrected y and z positions to east and altitude
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import matplotlib
# Use TkAgg backend for cross-platform compatibility
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from threading import Lock
import time


class DronePlotter(Node):
    """
    A ROS2 node for visualizing eVTOL trajectories and orientations.

    This class subscribes to the '/pub_pose' topic to receive PoseStamped messages,
    which contain the eVTOL's position and orientation information. It then uses
    matplotlib to create a 3D plot of the eVTOL's trajectory and current orientation.

    The plot is updated in real-time as new pose information is received.

    Attributes:
        subscription (rclpy.subscription.Subscription): Subscription to the 'pub_pose' topic.
        positions (list): List of (x, y, z) tuples representing the eVTOL's trajectory in north-east-down frame.
        current_orientation (tuple): The eVTOL's current orientation as a quaternion (x, y, z, w).
        fig (matplotlib.figure.Figure): The main figure for the plot.
        ax (mpl_toolkits.mplot3d.axes3d.Axes3D): The 3D axes for the plot.
        plot_lock (threading.Lock): Lock for thread-safe plot updates.
        last_update_time (float): Time of the last plot update.
    """

    def __init__(self):
        """
        Initialize the DronePlotter node.

        This constructor initializes the DronePlotter node by calling the parent
        class constructor with the node name 'drone_plotter'. It sets up the
        node's basic configuration and prepares it for further initialization
        steps that will be defined in the class.
        """
        super().__init__('drone_plotter')

        # Create subscription to pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            'pub_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Subscribed to pub_pose topic')
        
        # Initialize data storage
        self.positions = []
        self.current_orientation = None
        self.plot_lock = Lock()
        self.last_update_time = time.time()
        self.update_rate = 0.1  # Update plot at most every 0.1 seconds
        
        # Initialize the plot
        try:
            plt.ion()  # Interactive mode on
            self.fig = plt.figure(figsize=(10, 8))
            self.fig.canvas.manager.set_window_title("eVTOL Visualization")
            self.ax = self.fig.add_subplot(111, projection='3d')
            
            # Set up plot closing handler
            self.fig.canvas.mpl_connect('close_event', self.on_close)
            
            self.get_logger().info('Visualization initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize visualization: {e}')
            raise

        # Create a timer for periodic plot updates rather than relying on callbacks
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Periodic timer callback to update the plot"""
        if hasattr(self, 'fig') and plt.fignum_exists(self.fig.number):
            self.update_plot()

    def pose_callback(self, msg):
        """
        Callback function for received pose messages.
        
        Args:
            msg (PoseStamped): The received pose message containing position and orientation.
        """
        with self.plot_lock:
            pos = msg.pose.position
            # Reverse z position to get altitude
            self.positions.append((pos.x, pos.y, -pos.z))
            self.current_orientation = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )

    def update_plot(self):
        """
        Update the 3D visualization plot with the latest trajectory and orientation data.
        This method is designed to be thread-safe and rate-limited to prevent excessive updates.
        """
        current_time = time.time()
        if current_time - self.last_update_time < self.update_rate:
            return
            
        with self.plot_lock:
            if not self.positions:
                return

            # Reset the last update time
            self.last_update_time = current_time
            
            # Extract position data
            xs, ys, zs = zip(*self.positions)
            
            try:
                self.ax.clear()
                # Plot trajectory
                self.ax.plot(ys, xs, zs, marker='.', linestyle='-', label="Trajectory")
                
                # Current position
                x, y, z = self.positions[-1]
                self.ax.scatter(y, x, z, color='r', s=100, label="Current Position")
                
                # Orientation vector (reverse z direction to get altitude)
                if self.current_orientation:
                    forward = self.compute_forward_vector(self.current_orientation)
                    self.ax.quiver(
                        y, x, z,
                        forward[1], forward[0], -forward[2],
                        length=10., color='g', arrow_length_ratio=0.2,
                        label="Orientation"
                    )
                
                # Set axis labels and title
                self.ax.set_xlabel("east (m)")
                self.ax.set_ylabel("north (m)")
                self.ax.set_zlabel("altitude (m)")
                self.ax.set_title("eVTOL Trajectory and Orientation")
                self.ax.legend(loc='upper left')
                
                # Make axes equal for better visualization
                x_limits = self.ax.get_xlim3d()
                y_limits = self.ax.get_ylim3d()
                z_limits = self.ax.get_zlim3d()
                
                x_range = abs(x_limits[1] - x_limits[0])
                x_middle = np.mean(x_limits)
                y_range = abs(y_limits[1] - y_limits[0])
                y_middle = np.mean(y_limits)
                z_range = abs(z_limits[1] - z_limits[0])
                z_middle = np.mean(z_limits)
                
                # Set the same scale for all axes
                max_range = 0.5 * max([x_range, y_range, z_range])
                self.ax.set_xlim3d([x_middle - max_range, x_middle + max_range])
                self.ax.set_ylim3d([y_middle - max_range, y_middle + max_range])
                self.ax.set_zlim3d([z_middle - max_range, z_middle + max_range])
                
                # Draw the plot
                self.fig.canvas.draw_idle()
                plt.pause(0.001)
            except Exception as e:
                self.get_logger().error(f'Error updating plot: {e}')

    @staticmethod
    def compute_forward_vector(quat):
        """
        Compute the forward vector from a quaternion.

        This method takes a quaternion representing the orientation of the drone
        and calculates the corresponding forward vector in the global frame.

        Args:
            quat (tuple): A tuple containing the quaternion components (qx, qy, qz, qw).

        Returns:
            numpy.ndarray: A 3D vector representing the forward direction in the global frame.

        Note:
            This method assumes that the drone's forward direction aligns with the x-axis
            in its body frame.
        """
        qx, qy, qz, qw = quat
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-10:  # Avoid division by near-zero values
            norm = 1.0
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        
        r11 = 1 - 2*(qy*qy + qz*qz)
        r12 = 2*(qx*qy - qz*qw)
        r13 = 2*(qx*qz + qy*qw)
        return np.array([r11, r12, r13])
    
    def on_close(self, event):
        """Handle window close event"""
        self.get_logger().info('Plot window closed. Press Ctrl+C to terminate the node.')


def main(args=None):
    """Main entry point for the drone plotter node."""
    rclpy.init(args=args)
    
    try:
        node = DronePlotter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Drone Plotter: {e}")
    finally:
        # Cleanup
        if 'node' in locals():
            node.get_logger().info("Shutting down Drone Plotter Visualization.")
            node.destroy_node()
        plt.close('all')  # Close all matplotlib figures
        rclpy.shutdown()


if __name__ == '__main__':
    main()
