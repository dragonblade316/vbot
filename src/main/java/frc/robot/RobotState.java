package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Odometry.VSwervePoseEstimator;

public class RobotState {
    private static RobotState instance;
    private RobotState() {}

    public static RobotState get_instance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    public VSwervePoseEstimator poseEstimator = new VSwervePoseEstimator(
        DriveConstants.kinematics, 
        new Rotation2d(), 
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
        }, 
        new Pose2d()
    );

    //ngl I saw the record keywork in the mech advantage codebase and decided to try it out
    //TODO: replace these constants with actual math
    public static record AimingFunctions() {
        public static Supplier<Rotation2d> armAngle = () -> Rotation2d.fromDegrees(20);
        public static Supplier<Rotation2d> heading = () -> Rotation2d.fromDegrees(0);
        public static DoubleSupplier flywheelSpeed = () -> 0;
    }

    //drive base: (these will be completely removed in favor of the VSwervePoseEstimator)
    // public Pose2d robotPose = new Pose2d();
    // public Twist2d translationVelocity = new Twist2d();

    //climber
    public boolean climbersUp = false;

    //intake and carrier:
    public boolean containsPiece = false;
    public boolean barfing = false;

    //shooter:

    //might move this to the shooter subsystem
    public enum FlywheelState {
        READY,
        ACCELERATING,
        INACTIVE
    }

    public FlywheelState shooterFlywheelState = FlywheelState.INACTIVE;
    public boolean headingAligned = false;
    public boolean armInPosition = false;
}
