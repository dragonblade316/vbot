package frc.robot.subsystems.drive.Controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPICleaner;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class TeleopController {

    private final double DEADBAND = 0.05;

    private double x = 0;
    private double y = 0;
    private double omega = 0;

    public void updateInputs(double x, double y, double omega) {
        this.x = x;
        this.y = y;
        this.omega = omega;
    }

    public ChassisSpeeds update() {
        // Apply deadband
        double linearMagnitude =
            MathUtil.applyDeadband(
                Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection =
            new Rotation2d(x, y);
        double omega = MathUtil.applyDeadband(this.omega, DEADBAND);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
            new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();

        // Convert to field relative speeds & send command
        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
          
        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
            omega * DriveConstants.MAX_ANGULAR_SPEED,
            isFlipped
                ? RobotState.get_instance().poseEstimator.getEstimatedPose().getRotation().plus(new Rotation2d(Math.PI))
                : RobotState.get_instance().poseEstimator.getEstimatedPose().getRotation());

        Logger.recordOutput("Drive/TeleopControllerOutput", speeds);
        return speeds;
    }
}
