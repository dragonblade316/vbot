package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.vlib.FieldUtils;
import frc.robot.util.vlib.VSwervePoseEstimator;

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
        public static Supplier<Translation2d> heading = () -> FieldUtils.apply(new Translation2d(0.15, 5.5));
        public static DoubleSupplier flywheelSpeed = () -> 2500;
    }

    public static record LobbingFunctions() {
        public static Supplier<Rotation2d> armAngle = () -> Rotation2d.fromDegrees(20);
        public static Supplier<Translation2d> heading = () -> new Translation2d(0.15, 5.5);
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
    public enum SmartFireMode {
        Standard, //handles lobbing and speaker depending on the position of the robot
        AMPTRAP, //handles the amp and trap fire mode
    }
    public SmartFireMode smartFireMode = SmartFireMode.Standard;
    public Command AmpTrapModeCommand() {
        return Commands.startEnd(() -> smartFireMode = SmartFireMode.AMPTRAP, () -> smartFireMode = SmartFireMode.Standard);
    }

    public enum FlywheelState {
        READY,
        ACCELERATING,
        INACTIVE
    }

    public FlywheelState shooterFlywheelState = FlywheelState.INACTIVE;
    public boolean headingAligned = false;
    public boolean armInPosition = false;
}
