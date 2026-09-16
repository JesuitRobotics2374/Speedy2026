package frc.robot.subsystems.vision;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public final class PoseEstimateValues {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;
    public final Matrix<N3, N1> standardDeviations;

    /**
     * Creates the class to hold these values
     * 
     * @param estimate - Estimated pose as per Vision
     * @param time     - Timestamp (seconds) of when the estimate was made
     * @param stdDev   - The standard deviations for the estimate
     */
    public PoseEstimateValues(Pose3d estimate, double time, Matrix<N3, N1> stdDev) {
        estimatedPose = estimate;
        timestampSeconds = time;
        standardDeviations = stdDev;
    }
}