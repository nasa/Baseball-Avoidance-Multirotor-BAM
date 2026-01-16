#!/usr/bin/env python3
"""
Phase-Space Warping (PSW) Analysis Node for eVTOL Dynamics
---------------------------------------------------------
This node implements a rigorous Phase-Space Warping analysis framework for
detecting parameter drift and anomalies in eVTOL flight dynamics. It publishes
results to the /phase_space_warping_analyzer

The implementation follows the theoretical framework established by Chelidze & Cusumano (2006)
while addressing the practical challenges of reference model construction in operational
flight scenarios. The approach employs adaptive local linear modeling with
statistical robustness measures to ensure reliable drift detection.

Author: Newton Campbell
Date: 04/08/2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np
from scipy import linalg
from scipy.spatial import distance
from scipy.stats import chi2
from collections import deque
from sklearn.decomposition import PCA
from sklearn.neighbors import NearestNeighbors
from sklearn.covariance import MinCovDet


class PhaseSpaceWarpingAnalyzer(Node):
    """
    A ROS2 node for performing Phase-Space Warping (PSW) analysis on eVTOL flight dynamics.

    This class implements the PSW algorithm to detect parameter drift and anomalies
    in the system dynamics. It subscribes to pose data, performs the PSW analysis,
    and publishes the results for further processing or visualization.

    The PSW analysis is based on the theoretical framework by Chelidze & Cusumano (2006),
    adapted for real-time analysis of flight data. It uses techniques such as
    delay-coordinate embedding, local linear modeling, and statistical measures
    to quantify deviations from a reference dynamics model.

    Key features:
    - Constructs a phase space embedding using Takens' theorem
    - Establishes a reference manifold from initial flight data
    - Calculates a tracking function to measure deviations from reference dynamics
    - Provides confidence and significance metrics for the detected anomalies
    - Estimates drift rates to track gradual changes in system behavior

    The node is configurable via ROS parameters, allowing for adjustment of
    analysis parameters such as embedding dimension, time delay, and confidence thresholds.
    """
    def __init__(self):
        """
        Initialize the PhaseSpaceWarpingAnalyzer node.

        This constructor sets up the ROS2 node, configures QoS profiles,
        declares and retrieves parameters, initializes data structures,
        and sets up subscribers and publishers. It also logs the initial
        configuration information.

        The node is responsible for:
        - Subscribing to pose data
        - Performing Phase-Space Warping (PSW) analysis
        - Publishing PSW analysis results

        Parameters are configurable via ROS parameters and include:
        - Buffer size
        - Analysis interval
        - Embedding dimension
        - Time delay
        - Reference percentage
        - Number of neighbors for local modeling
        - SVD threshold for dimensionality reduction
        - Confidence threshold for analysis

        The node initializes in a state where it collects initial data
        to establish a reference manifold before beginning analysis.
        """
        super().__init__('phase_space_warping_analyzer')
        
        # Configure QoS profile for reliable data transmission
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Analysis parameters - configurable via ROS parameters
        self.declare_parameter('buffer_size', 100)
        self.declare_parameter('analysis_interval_steps', 15)  # Number of timesteps between analyses (after the buffer fills up for the first time)
        self.declare_parameter('embedding_dimension', 4)
        self.declare_parameter('time_delay', 4)
        self.declare_parameter('reference_percentage', 0.25)
        self.declare_parameter('k_neighbors', 12)
        self.declare_parameter('svd_threshold', 0.95)
        self.declare_parameter('confidence_threshold', 0.75)
        
        # Retrieve parameters
        self.buffer_size = self.get_parameter('buffer_size').value
        self.analysis_interval_steps = self.get_parameter('analysis_interval_steps').value
        self.embedding_dimension = self.get_parameter('embedding_dimension').value
        self.time_delay = self.get_parameter('time_delay').value
        self.reference_percentage = self.get_parameter('reference_percentage').value
        self.k_neighbors = self.get_parameter('k_neighbors').value
        self.svd_threshold = self.get_parameter('svd_threshold').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Data structures
        self.pose_buffer = deque(maxlen=self.buffer_size)
        self.reference_data = None
        self.reference_established = False
        self.psw_history = []
        self.step_counter = 0  # Counter for simulation timesteps
        self.analysis_ready = False  # Flag to indicate when analysis should be performed
        
        # Statistical measures
        self.mean_tracking_norm = None
        self.cov_tracking = None
        
        # Subscribers and publishers
        self.subscription = self.create_subscription(
            PoseStamped,
            'pub_pose',
            self.pose_callback,
            qos_profile=qos_profile
        )
        
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'phase_space_warping_analyzer',
            qos_profile=qos_profile
        )
        
        # Initialize logger with detailed configuration information
        self.get_logger().info('Phase Space Warping Analyzer initialized with configuration:')
        self.get_logger().info(f'  Buffer size: {self.buffer_size}')
        self.get_logger().info(f'  Analysis interval: {self.analysis_interval_steps} timesteps')
        self.get_logger().info(f'  Embedding dimension: {self.embedding_dimension}')
        self.get_logger().info(f'  Time delay: {self.time_delay}')
        self.get_logger().info(f'  Reference percentage: {self.reference_percentage}')
        self.get_logger().info(f'  k-neighbors: {self.k_neighbors}')
        self.get_logger().info(f'  SVD threshold: {self.svd_threshold}')
        self.get_logger().info('Collecting initial data to establish reference manifold...')
    
    def pose_callback(self, msg):
        """
        Process incoming pose data and track simulation timesteps.
        
        Args:
            msg (PoseStamped): The incoming pose message
        """
        # Extract position data with high precision
        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float64)
        
        # Extract orientation as quaternion for complete state representation
        orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ], dtype=np.float64)
        
        # Store the complete state vector with simulation step index
        self.pose_buffer.append((self.step_counter, position, orientation))
        
        # Increment step counter
        self.step_counter += 1
        
        # Check if buffer is full for the first time
        if len(self.pose_buffer) == self.buffer_size and not self.analysis_ready:
            self.analysis_ready = True
            self.get_logger().info(f'Buffer filled with {self.buffer_size} samples. Analysis ready.')
        
        # Check if it's time to perform analysis based on timestep count
        if self.analysis_ready and (self.step_counter % self.analysis_interval_steps == 0):
            self.perform_psw_analysis()
    
    def perform_psw_analysis(self):
        """
        Execute Phase-Space Warping analysis to detect parameter drift in system dynamics.
        
        This implementation follows the mathematical framework described in:
        Chelidze, D., & Cusumano, J. P. (2006). Phase space warping: nonlinear time‐series 
        analysis for slowly drifting systems.
        """
        try:
            self.get_logger().debug(f"Starting PSW analysis at timestep {self.step_counter}")
            self.get_logger().debug(f"Buffer size: {len(self.pose_buffer)}")
            
            # Extract position and orientation data
            positions = np.array([p[1] for p in self.pose_buffer])
            orientations = np.array([p[2] for p in self.pose_buffer])
            
            self.get_logger().debug(f"Positions shape: {positions.shape}")
            self.get_logger().debug(f"Orientations shape: {orientations.shape}")
            
            # Create phase space embedding using delay coordinates
            self.get_logger().debug("Creating Takens embedding...")
            embedded_data = self.create_takens_embedding(positions, orientations)
            self.get_logger().debug(f"Embedded data shape: {embedded_data.shape}")
            
            # If this is the first run, establish reference model
            if not self.reference_established:
                self.get_logger().debug("Establishing reference manifold...")
                self.establish_reference_manifold(embedded_data)
                self.reference_established = True
                self.get_logger().info('Reference manifold established from initial flight data')
                return
            
            # Calculate tracking function and confidence metrics
            self.get_logger().debug("Calculating tracking function...")
            tracking_vector, confidence, mahalanobis_dist = self.calculate_tracking_function(embedded_data)
            self.get_logger().debug(f"Tracking vector shape: {tracking_vector.shape}")
            self.get_logger().debug(f"Confidence: {confidence}")
            self.get_logger().debug(f"Mahalanobis distances shape: {mahalanobis_dist.shape}")
            
            # Calculate PSW metrics
            tracking_norms = np.linalg.norm(tracking_vector, axis=1)
            self.get_logger().debug(f"Tracking norms shape: {tracking_norms.shape}")
            self.get_logger().debug(f"Tracking norms min/max/mean: {np.min(tracking_norms)}/{np.max(tracking_norms)}/{np.mean(tracking_norms)}")
            
            tracking_magnitude = np.mean(tracking_norms)
            tracking_variance = np.var(tracking_norms)
            max_warping = np.max(tracking_norms)
            
            # Calculate statistical significance using Mahalanobis distance
            significance = np.mean(mahalanobis_dist)
            self.get_logger().debug(f"Significance: {significance}")
            
            # Store historical PSW data for trend analysis
            self.psw_history.append((tracking_magnitude, significance, confidence))
            self.get_logger().debug(f"PSW history length: {len(self.psw_history)}")
            
            # Calculate drift rate if we have enough history
            drift_rate = 0.0
            if len(self.psw_history) > 2:
                # Simple linear regression on recent tracking magnitudes
                recent_magnitudes = np.array([h[0] for h in self.psw_history[-5:]])
                self.get_logger().debug(f"Recent magnitudes: {recent_magnitudes}")
                
                x = np.arange(len(recent_magnitudes))
                A = np.vstack([x, np.ones(len(x))]).T
                
                self.get_logger().debug(f"Regression matrix A shape: {A.shape}")
                
                try:
                    slope, _ = np.linalg.lstsq(A, recent_magnitudes, rcond=None)[0]
                    drift_rate = slope
                    self.get_logger().debug(f"Calculated drift rate: {drift_rate}")
                except np.linalg.LinAlgError as e:
                    self.get_logger().error(f"Error calculating drift rate: {str(e)}")
                    drift_rate = 0.0

            tracking_magnitude = np.nan_to_num(tracking_magnitude, nan=0.0, posinf=0.0, neginf=0.0)
            tracking_variance = np.nan_to_num(tracking_variance, nan=0.0, posinf=0.0, neginf=0.0)
            max_warping = np.nan_to_num(max_warping, nan=0.0, posinf=0.0, neginf=0.0)
            confidence = np.clip(np.nan_to_num(confidence, nan=0.5), 0.0, 1.0)
            significance = np.nan_to_num(significance, nan=0.0, posinf=0.0, neginf=0.0)
            drift_rate = np.nan_to_num(drift_rate, nan=0.0, posinf=0.0, neginf=0.0)
            
            # Publish results
            self.publish_results(tracking_magnitude, tracking_variance, max_warping, 
                                confidence, significance, drift_rate)
            
            self.get_logger().info(f'PSW Analysis at timestep {self.step_counter}:')
            self.get_logger().info(f'  Tracking magnitude: {tracking_magnitude:.4f}')
            self.get_logger().info(f'  Statistical significance: {significance:.4f}')
            self.get_logger().info(f'  Confidence: {confidence:.4f}')
            self.get_logger().info(f'  Drift rate: {drift_rate:.6f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in PSW analysis: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def create_takens_embedding(self, positions, orientations):
        """
        Create a Takens embedding of the system state using delay coordinates.
        
        This implements the delay-coordinate embedding theorem (Takens, 1981) to
        reconstruct the phase space from time series observations.
        
        Args:
            positions (np.ndarray): Array of position vectors
            orientations (np.ndarray): Array of orientation quaternions
            
        Returns:
            np.ndarray: The embedded data in phase space
        """
        # Number of points in the embedded space
        N = len(positions) - (self.embedding_dimension - 1) * self.time_delay
        
        # Calculate the dimension of the embedded space
        # 3 for position + 4 for quaternion = 7 per embedding dimension
        embedded_dim = self.embedding_dimension * 7
        
        # Initialize the embedded data array
        embedded_data = np.zeros((N, embedded_dim))
        
        # Create the embedding
        for i in range(N):
            vector = []
            for j in range(self.embedding_dimension):
                delay_index = i + j * self.time_delay
                # Concatenate position and orientation for complete state
                vector.extend(positions[delay_index])
                vector.extend(orientations[delay_index])
            embedded_data[i] = np.array(vector)
        
        # Apply dimensionality reduction to focus on the most significant dynamics
        # This helps mitigate the curse of dimensionality in high-dimensional embeddings
        pca = PCA(n_components=self.svd_threshold, svd_solver='full')
        reduced_data = pca.fit_transform(embedded_data)
        
        self.get_logger().debug(f'Embedding created with dimension {embedded_dim}, '
                              f'reduced to {reduced_data.shape[1]} dimensions '
                              f'capturing {self.svd_threshold*100:.1f}% of variance')
        
        return reduced_data
    
    def establish_reference_manifold(self, embedded_data):
        """
        Establish a reference manifold from the initial portion of the data.
        
        This creates a statistical model of the nominal system dynamics to serve
        as a baseline for detecting parameter drift.
        
        Args:
            embedded_data (np.ndarray): The embedded data in phase space
        """
        # Use the first portion of data as reference
        ref_size = int(len(embedded_data) * self.reference_percentage)
        self.reference_data = embedded_data[:ref_size]
        
        # Create nearest neighbor model for the reference data
        self.nn_model = NearestNeighbors(n_neighbors=self.k_neighbors, algorithm='auto')
        self.nn_model.fit(self.reference_data[:-1])  # Exclude last point as it has no successor
        
        # Store the successors for each reference point
        self.reference_successors = self.reference_data[1:]
        
        # Calculate the transition matrices for each point in the reference data
        self.reference_transitions = []
        for i in range(len(self.reference_data) - 1):
            y_i = self.reference_data[i]
            y_i_plus_1 = self.reference_data[i + 1]
            self.reference_transitions.append(y_i_plus_1 - y_i)
        
        # Convert to numpy array for easier handling
        self.reference_transitions = np.array(self.reference_transitions)
        
        # Check if we have enough data points
        if len(self.reference_transitions) < 2:
            self.get_logger().error(f"Not enough reference transitions: {len(self.reference_transitions)}")
            # Create fallback values
            dim = embedded_data.shape[1]
            self.mean_transition = np.zeros(dim)
            self.cov_transition = np.eye(dim) * 1e-3
            self.inv_cov_transition = np.eye(dim) * 1e3
            return
        
        # Calculate the mean and covariance of the transition vectors
        try:
            # Try with increased support_fraction
            mcd = MinCovDet(support_fraction=0.8).fit(self.reference_transitions)
            self.mean_transition = mcd.location_
            self.cov_transition = mcd.covariance_
        except ValueError as e:
            self.get_logger().warn(f"MinCovDet failed: {str(e)}. Using empirical covariance instead.")
            # Calculate mean manually
            self.mean_transition = np.mean(self.reference_transitions, axis=0)
            
            # Calculate covariance manually
            # First center the data
            centered_data = self.reference_transitions - self.mean_transition
            # Then calculate covariance
            dim = self.reference_transitions.shape[1]
            self.cov_transition = np.zeros((dim, dim))
            
            for i in range(len(centered_data)):
                x = centered_data[i].reshape(-1, 1)  # Column vector
                self.cov_transition += x @ x.T
            
            # Normalize by n-1
            self.cov_transition /= max(1, len(centered_data) - 1)
        
        # Add regularization to ensure invertibility
        # Make sure cov_transition is a numpy array with the right shape
        if not isinstance(self.cov_transition, np.ndarray):
            dim = self.reference_transitions.shape[1]
            self.get_logger().warn(f"Covariance is not a numpy array. Creating identity matrix of size {dim}.")
            self.cov_transition = np.eye(dim) * 1e-3
        else:
            # Add regularization
            dim = self.cov_transition.shape[0]
            self.cov_transition += np.eye(dim) * 1e-6
        
        # Calculate the inverse covariance matrix for Mahalanobis distance calculations
        try:
            self.inv_cov_transition = linalg.inv(self.cov_transition)
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Error inverting covariance matrix: {str(e)}")
            # Use pseudo-inverse as fallback
            self.inv_cov_transition = linalg.pinv(self.cov_transition)
    
    def calculate_tracking_function(self, embedded_data):
        """
        Calculate the tracking function using local linear models from reference data.
        
        This implements the core PSW methodology, comparing the actual system evolution
        to the predicted evolution based on the reference dynamics.
        
        Args:
            embedded_data (np.ndarray): The embedded data in phase space
            
        Returns:
            tuple: (tracking_vector, confidence, mahalanobis_distances)
        """
        # We'll analyze the latter portion of the data (non-reference)
        ref_size = int(len(embedded_data) * self.reference_percentage)
        test_data = embedded_data[ref_size:]
        
        self.get_logger().debug(f"Reference size: {ref_size}")
        self.get_logger().debug(f"Test data shape: {test_data.shape}")
        self.get_logger().debug(f"Reference data shape: {self.reference_data.shape}")
        
        # Initialize tracking vector and metrics
        N = len(test_data) - 1
        dim = test_data.shape[1]
        tracking_vector = np.zeros((N, dim))
        confidence_values = np.zeros(N)
        mahalanobis_distances = np.zeros(N)
        
        self.get_logger().debug(f"N: {N}, dim: {dim}")
        
        # For each point in the test data (except the last one)
        for i in range(N):
            # Current point and its actual successor
            y_i = test_data[i]
            y_i_plus_1_actual = test_data[i+1]
            
            self.get_logger().debug(f"Processing point {i}/{N}")
            
            # Find k nearest neighbors in the reference data
            distances, indices = self.nn_model.kneighbors([y_i], n_neighbors=min(self.k_neighbors, len(self.reference_data)-1))
            
            self.get_logger().debug(f"Found {len(indices[0])} neighbors")
            self.get_logger().debug(f"Neighbor distances min/max: {np.min(distances)}/{np.max(distances)}")
            
            # Calculate confidence based on distance to nearest neighbors
            # Using a chi-square based metric for statistical validity
            mean_distance = np.mean(distances)
            confidence = chi2.sf(mean_distance, df=dim)
            confidence_values[i] = confidence
            
            self.get_logger().debug(f"Mean distance: {mean_distance}, Confidence: {confidence}")
            
            # Get the neighbors and their successors
            neighbors = self.reference_data[indices[0]]
            neighbor_indices = indices[0]
            
            self.get_logger().debug(f"Neighbor indices shape: {neighbor_indices.shape}")
            
            # Make sure we don't go out of bounds
            valid_indices = [idx for idx in neighbor_indices if idx < len(self.reference_successors)]
            if len(valid_indices) < len(neighbor_indices):
                self.get_logger().warn(f"Some neighbor indices were out of bounds: {len(valid_indices)}/{len(neighbor_indices)} valid")
            
            if len(valid_indices) == 0:
                self.get_logger().error("No valid neighbor indices found!")
                # Skip this point
                continue
            
            neighbor_successors = self.reference_successors[valid_indices]
            
            self.get_logger().debug(f"Neighbors shape: {neighbors.shape}")
            self.get_logger().debug(f"Neighbor successors shape: {neighbor_successors.shape}")
            
            # Build local linear model: y_{i+1} = A*y_i + b
            # Using weighted least squares to emphasize closer neighbors
            mean_dist = np.mean(distances[0][:len(valid_indices)])
            if mean_dist <= 0 or np.isnan(mean_dist):
                # If mean distance is zero or NaN, use uniform weights
                weights = np.ones(len(valid_indices)) / len(valid_indices)
                self.get_logger().warn("Mean distance is zero or NaN. Using uniform weights.")
            else:
                weights = np.exp(-distances[0][:len(valid_indices)] / mean_dist)
                # Handle any potential NaN values in weights
                weights = np.nan_to_num(weights, nan=1.0)
                sum_weights = np.sum(weights)
                if sum_weights > 0:
                    weights = weights / sum_weights
                else:
                    weights = np.ones(len(valid_indices)) / len(valid_indices)
            
            self.get_logger().debug(f"Weights shape: {weights.shape}")
            self.get_logger().debug(f"Weights min/max/mean: {np.min(weights)}/{np.max(weights)}/{np.mean(weights)}")
            
            # Fix: Solve for each dimension separately to avoid dimension mismatch
            for d in range(dim):
                # Extract the d-th dimension of the successors
                y_successors_d = neighbor_successors[:, d]
                
                # Construct the design matrix with a column of ones for the intercept
                X = np.column_stack([neighbors, np.ones(len(neighbors))])
                
                self.get_logger().debug(f"Dimension {d}: X shape: {X.shape}, y shape: {y_successors_d.shape}")
                
                # Solve the weighted least squares problem for this dimension
                # (X^T W X)^{-1} X^T W y
                W = np.diag(weights)
                XTW = X.T @ W
                
                # Check for potential numerical issues
                cond_num = np.linalg.cond(XTW @ X)
                self.get_logger().debug(f"Condition number of XTW @ X: {cond_num}")
                
                # Solve the linear system
                try:
                    beta_d = np.linalg.solve(XTW @ X, XTW @ y_successors_d)
                    
                    self.get_logger().debug(f"Beta shape: {beta_d.shape}")
                    
                    # Extract the coefficient vector and intercept for this dimension
                    A_d = beta_d[:-1]
                    b_d = beta_d[-1]
                    
                    # Predict the successor for this dimension
                    y_i_plus_1_predicted_d = np.dot(A_d, y_i) + b_d
                    
                    self.get_logger().debug(f"Actual: {y_i_plus_1_actual[d]}, Predicted: {y_i_plus_1_predicted_d}")
                    
                    # Store the tracking vector component for this dimension
                    tracking_vector[i, d] = y_i_plus_1_actual[d] - y_i_plus_1_predicted_d
                    
                except np.linalg.LinAlgError as e:
                    # Handle potential singular matrix issues
                    self.get_logger().warn(f"Linear algebra error in dimension {d}: {str(e)}")
                    # Use a simple average of neighbor successors as fallback
                    y_i_plus_1_predicted_d = np.average(neighbor_successors[:, d], weights=weights)
                    tracking_vector[i, d] = y_i_plus_1_actual[d] - y_i_plus_1_predicted_d
            
            # Calculate Mahalanobis distance to determine statistical significance
            # of the deviation from reference dynamics
            transition_vector = y_i_plus_1_actual - y_i
            delta = transition_vector - self.mean_transition
            
            try:
                mahalanobis_distances[i] = np.sqrt(delta @ self.inv_cov_transition @ delta)
                self.get_logger().debug(f"Mahalanobis distance: {mahalanobis_distances[i]}")
            except Exception as e:
                self.get_logger().warn(f"Error calculating Mahalanobis distance: {str(e)}")
                mahalanobis_distances[i] = 0.0
        
        # Average confidence across all points
        mean_confidence = np.mean(confidence_values)
        
        self.get_logger().debug(f"Final tracking vector shape: {tracking_vector.shape}")
        self.get_logger().debug(f"Mean confidence: {mean_confidence}")
        self.get_logger().debug(f"Mahalanobis distances shape: {mahalanobis_distances.shape}")
        
        return tracking_vector, mean_confidence, mahalanobis_distances

    
    def publish_results(self, tracking_magnitude, tracking_variance, max_warping, 
                    confidence, significance, drift_rate):
        """
        Publish PSW analysis results for Simulink consumption.
        
        This method publishes a Float32MultiArray message to the /phase_space_warping_analyzer topic
        containing seven metrics that characterize the system's deviation from reference
        dynamics. Simulink can subscribe to this topic to receive these metrics for
        downstream processing and decision-making.
        
        The published array contains the following elements in order:
        
        [0] timestep: Current simulation timestep (float)
            - Indicates when this analysis was performed in simulation time
            - Useful for synchronizing analysis results with simulation state
            - Range: [0, inf), increases monotonically
        
        [1] tracking_magnitude: Average norm of the tracking vector (float)
            - Measures the overall magnitude of deviation from expected dynamics
            - Higher values indicate greater deviation from reference behavior
            - Range: [0, inf), typical values: 0.01-10.0
        
        [2] tracking_variance: Variance of the tracking vector norms (float)
            - Measures the consistency/variability of the deviations
            - Higher values indicate more erratic/unpredictable behavior
            - Range: [0, inf), typical values: 0.001-5.0
        
        [3] max_warping: Maximum norm in the tracking vector (float)
            - Indicates the largest single deviation from expected dynamics
            - Useful for detecting brief but significant anomalies
            - Range: [0, inf), typical values: 0.1-20.0
        
        [4] confidence: Confidence in the analysis results (float)
            - Based on statistical distance to reference data
            - Higher values indicate more reliable analysis
            - Range: [0, 1], where 1 is highest confidence
        
        [5] significance: Statistical significance of the detected drift (float)
            - Based on Mahalanobis distance from reference dynamics
            - Higher values indicate more statistically significant deviations
            - Range: [0, inf), typical values: 0.5-10.0
        
        [6] drift_rate: Estimated rate of parameter drift (float)
            - Slope of recent tracking magnitudes over time
            - Positive values indicate increasing deviation (worsening condition)
            - Negative values indicate decreasing deviation (improving condition)
            - Range: (-inf, inf), typical values: -0.1 to 0.1
        
        Args:
            tracking_magnitude (float): Average norm of the tracking vector
            tracking_variance (float): Variance of the tracking vector norms
            max_warping (float): Maximum norm in the tracking vector
            confidence (float): Confidence in the analysis results
            significance (float): Statistical significance of the detected drift
            drift_rate (float): Estimated rate of parameter drift
        """
        msg = Float32MultiArray()
        
        # Create and populate the message layout
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "results"
        msg.layout.dim[0].size = 7
        msg.layout.dim[0].stride = 7
        
        # Function to ensure values are valid float32
        def safe_float(value):
            if np.isnan(value) or np.isinf(value):
                self.get_logger().warn(f"Invalid value detected: {value}, replacing with 0.0")
                return 0.0
            # Clamp to float32 range
            max_float32 = np.finfo(np.float32).max
            min_float32 = np.finfo(np.float32).min
            if value > max_float32:
                self.get_logger().warn(f"Value too large: {value}, clamping to {max_float32}")
                return float(max_float32)
            if value < min_float32:
                self.get_logger().warn(f"Value too small: {value}, clamping to {min_float32}")
                return float(min_float32)
            return float(value)
        
        # Populate data array with PSW metrics, ensuring all values are valid float32
        try:
            msg.data = [
                safe_float(self.step_counter),
                safe_float(tracking_magnitude),
                safe_float(tracking_variance),
                safe_float(max_warping),
                safe_float(confidence),
                safe_float(significance),
                safe_float(drift_rate)
            ]
            
            # Publish the metrics for Simulink to consume
            self.publisher.publish(msg)
            
            # Debug output of published values
            self.get_logger().debug(f"Published results:")
            self.get_logger().debug(f"  [0] Timestep: {msg.data[0]}")
            self.get_logger().debug(f"  [1] Tracking magnitude: {msg.data[1]:.4f}")
            self.get_logger().debug(f"  [2] Tracking variance: {msg.data[2]:.4f}")
            self.get_logger().debug(f"  [3] Max warping: {msg.data[3]:.4f}")
            self.get_logger().debug(f"  [4] Confidence: {msg.data[4]:.4f}")
            self.get_logger().debug(f"  [5] Significance: {msg.data[5]:.4f}")
            self.get_logger().debug(f"  [6] Drift rate: {msg.data[6]:.6f}")
        except Exception as e:
            self.get_logger().error(f"Error publishing results: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PhaseSpaceWarpingAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Phase Space Warping Analyzer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()