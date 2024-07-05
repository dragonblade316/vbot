package frc.robot.util.Odometry;

import org.ejml.data.ZMatrix;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


//Inspired by orbit and mechanical advantage
public class VSwervePoseEstimator {

    private final SwerveDriveKinematics kinematics;

    private Pose2d estimatedPose;

    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;
    private SwerveModulePosition[] previousPositions;

    private double odometryFOM = 0;
    private double visionFOM = 0;

    private TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(2);
    private VisionObservation latestVisionObservation = null;

    public static class OdometryObservation {
        SwerveModulePosition[] positions;
        SwerveModuleState[] states;
        Rotation2d gyroAngle;
        double timestamp;

        public OdometryObservation(
            SwerveModulePosition[] positions,
            SwerveModuleState[] states,
            Rotation2d gyroAngle,
            double timestamp) {
            
            this.positions = positions;
            this.states = states;
            this.gyroAngle = gyroAngle;
            this.timestamp = timestamp;
        }
    }

    public class VisionObservation {
        Pose2d pose;
        double timestamp;
    }



    public VSwervePoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] positions, Pose2d pose) {
        this.kinematics = kinematics;
        this.gyroOffset = pose.getRotation().minus(gyroAngle);
        this.previousAngle = pose.getRotation().plus(gyroOffset);
        previousPositions = positions;
        estimatedPose = pose;
    }


    public void updateOdometry(OdometryObservation observation) {
        //detect skidding
        double skiddingRatio = getSkiddingRatio(observation.states, kinematics);
        System.out.println(skiddingRatio);

        //TODO: tune this
        if (skiddingRatio > 2 ) odometryFOM += 0.2;

        Logger.recordOutput("Odometry/SkidRatio", skiddingRatio);
        Logger.recordOutput("Odometry/OFOM", odometryFOM);
        

        // Set fom and invalidate modules if nececary
        //TODO: figure out how to invalidate;

        Twist2d twist = kinematics.toTwist2d(new SwerveDriveWheelPositions(previousPositions), new SwerveDriveWheelPositions(observation.positions));
        previousPositions = observation.positions;
        // Check gyro connected
        if (observation.gyroAngle != null) {
        // Update dtheta for twist if gyro connected
        twist =
            new Twist2d(
                twist.dx, twist.dy, observation.gyroAngle.plus(gyroOffset).minus(previousAngle).getRadians());
        previousAngle = observation.gyroAngle.plus(gyroOffset);
        }
        // Add twist to odometry pose
        //odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp, estimatedPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }

    //going to need to find some 
    public void updateVision(VisionObservation observation) {
        
        var tempOPose = multiplyPose(estimatedPose, odometryFOM);
        var tempVPose = multiplyPose(observation.pose, visionFOM);

        estimatedPose = new Pose2d(tempOPose.getX() + tempVPose.getX(), tempOPose.getY() + tempVPose.getY(), tempOPose.getRotation().plus(tempVPose.getRotation())).div(odometryFOM + visionFOM);
    }

    private Pose2d multiplyPose(Pose2d pose, double multiplier) {
        return new Pose2d(pose.getX() * multiplier, pose.getY() * multiplier, pose.getRotation().times(multiplier));
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }
    
    // public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    //     return null;    
    // }


    /**
     * the method comes from 1690's <a href="https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674">online software session</a>
     * gets the skidding ratio from the latest , that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the  ratio between the maximum and minimum magnitude of the "translational" part of the speed of the modules
     * 
     * @param swerveStatesMeasured the swerve states measured from the modules
     * @param swerveDriveKinematics the kinematics
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     * */
    public static double getSkiddingRatio(SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
        final double angularVelocityOmegaMeasured = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
        final double[] swerveStatesTranslationalPartMagnitudes = new double[swerveStatesMeasured.length];

        for (int i =0; i < swerveStatesMeasured.length; i++) {
            final Translation2d swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
                    swerveStatesRotationalPartAsVector = convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
                    swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
        for (double translationalSpeed:swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

    private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
        return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }
}
