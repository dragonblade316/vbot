package frc.robot.util.vlib;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;


//Inspired by/copyed from orbit, mechanical advantage, and the wpilib PoseEstimator class
public class VSwervePoseEstimator {

    private final SwerveDriveKinematics kinematics;

    private Pose2d estimatedPose;
    private Pose2d odometryPose;

    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;
    private SwerveModulePosition[] previousPositions;

    private double robotFOM = 0;
    private double odometryFOM = 0;
    private double visionFOM = 0;

    // private ExtendedKalmanFilter kf;

    private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    private double prevXAccel = 0.0;
    private double prevYAccel = 0.0;
    private double prevOmegaAccel = 0.0;

    private TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer = TimeInterpolatableBuffer.createBuffer(2);
    private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

    private double last_kalman_predect = Timer.getFPGATimestamp();

    //copied from citrus circuts (with a couple changes)
    private ExtendedKalmanFilter kf = new ExtendedKalmanFilter<>(
        Nat.N3(), 
        Nat.N3(), 
        Nat.N3(), 
        (x, u) -> u, 
        (x, u) -> x, 
        null, 
        null, 
        0.02);




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
        odometryPose = pose;
    }


    public void updateOdometry(OdometryObservation observation) {
        //detect skidding
        double skiddingRatio = getSkiddingRatio(observation.states, kinematics);
        //System.out.println(skiddingRatio);

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
        odometryPose = odometryPose.exp(twist);
     
        if (visionUpdates.isEmpty()) {
            estimatedPose = odometryPose;
        } else {
            var visionUpdate = visionUpdates.get(visionUpdates.lastKey());
            var newPose = visionUpdate.compensate(odometryPose);

            
            kf.correct(VecBuilder.fill(0, 0, 0), VecBuilder.fill(newPose.getX(), newPose.getY(), newPose.getRotation().getRadians()));
        }

        //the timestamp stuff is needed because the odom can be 250 hz or 50 depending on mode
        kf.predict(VecBuilder.fill(0.0, 0.0, 0.0), Timer.getFPGATimestamp() - last_kalman_predect);
        last_kalman_predect = Timer.getFPGATimestamp();

        

        // Calculate diff from last odometry pose and add onto pose estimate
        //estimatedPose = estimatedPose.exp(twist);

         

        
    }

    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        // Step 0: If there are no odometry updates to sample, skip.
        if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
        return Optional.empty();
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().lastKey();
        timestampSeconds =
            MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only information.
        if (visionUpdates.isEmpty() || timestampSeconds < visionUpdates.firstKey()) {
        return odometryPoseBuffer.getSample(timestampSeconds);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to sample at.
        double floorTimestamp = visionUpdates.floorKey(timestampSeconds);
        var visionUpdate = visionUpdates.get(floorTimestamp);

        // Step 4: Get the pose measured by odometry at the time of the sample.
        var odometryEstimate = odometryPoseBuffer.getSample(timestampSeconds);

        // Step 5: Apply the vision compensation to the odometry pose.
        return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
    }

    //going to need to find some 
    public void updateVision(VisionObservation observation) {
        
        var tempOPose = multiplyPose(estimatedPose, odometryFOM);
        var tempVPose = multiplyPose(observation.pose, visionFOM);

        if (odometryPoseBuffer.getInternalBuffer().isEmpty()
            || odometryPoseBuffer.getInternalBuffer().lastKey() - 2
                > observation.timestamp) {
            return;
        }

        // Step 1: Clean up any old entries
        cleanUpVisionUpdates();

        // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
        var odometrySample = odometryPoseBuffer.getSample(observation.timestamp);

        if (odometrySample.isEmpty()) {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
        // made.
        var visionSample = sampleAt(observation.timestamp);

        if (visionSample.isEmpty()) {
            return;
        }

        
        var visionUpdate = new VisionUpdate(new Pose2d(tempOPose.getX() + tempVPose.getX(), tempOPose.getY() + tempVPose.getY(), tempOPose.getRotation().plus(tempVPose.getRotation())).div(odometryFOM + visionFOM), odometryPose);
        estimatedPose = visionUpdate.compensate(odometryPose);

        visionUpdates.put(observation.timestamp, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        visionUpdates.tailMap(observation.timestamp, false).entrySet().clear();

       

        //I dont think the rest (except for the one thing I did not comment) is needed. instead I am doing obbit style pose merging and may add a kalman filter

        // Step 4: Measure the twist between the old pose estimate and the vision pose.
        //var twist = visionSample.get().log(observation.pose);

        // Step 5: We should not trust the twist entirely, so instead we scale this twist by a Kalman
        // gain matrix representing how much we trust vision measurements compared to our current pose.
        //var k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

        // Step 6: Convert back to Twist2d.
        //var scaledTwist =
        //    new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

        // Step 7: Calculate and record the vision update.
        //var visionUpdate = new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
        // visionUpdates.put(observation.timestamp, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        // visionUpdates.tailMap(observation.timestamp, false).entrySet().clear();

        // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
        // it's guaranteed to be the latest vision update.
        //estimatedPose = visionUpdate.compensate(odometryPose);

        //estimatedPose = new Pose2d(tempOPose.getX() + tempVPose.getX(), tempOPose.getY() + tempVPose.getY(), tempOPose.getRotation().plus(tempVPose.getRotation())).div(odometryFOM + visionFOM);
    }

    private void cleanUpVisionUpdates() {
        // Step 0: If there are no odometry samples, skip.
        if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }

        // Step 1: Find the oldest timestamp that needs a vision update.
        double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().firstKey();

        // Step 2: If there are no vision updates before that timestamp, skip.
        if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) {
            return;
        }

        // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
        double newestNeededVisionUpdateTimestamp = visionUpdates.floorKey(oldestOdometryTimestamp);

        // Step 4: Remove all entries strictly before the newest timestamp we need.
        visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
    }


    public void setPose(Pose2d pose) {
        estimatedPose = pose;
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

    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose2d visionPose;

        // The pose estimated based solely on odometry.
        private final Pose2d odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
        this.visionPose = visionPose;
        this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
         * relative to this record's odometry pose to being relative to this record's vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose2d compensate(Pose2d pose) {
            var delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }


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


    // public double detectCollision() {
    //     double xAccel = accelerometer.getX();
    //     double yAccel = accelerometer.getY();
    //     double omegaAccel = accelerometer.getZ();

    //     double xJerk = (xAccel - prevXAccel) / 0.02;
    //     double yJerk = (yAccel - prevYAccel) / 0.02;
    //     double omegaJerk = (omegaAccel - prevOmegaAccel) / 0.02;
    // }
}
