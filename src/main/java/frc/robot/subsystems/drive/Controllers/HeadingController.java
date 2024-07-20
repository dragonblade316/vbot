package frc.robot.subsystems.drive.Controllers;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.vlib.FieldUtils;
import frc.robot.util.vlib.control.VPIDController;

public class HeadingController {
    PIDController controller;
    Supplier<Rotation2d> heading;

    

    public HeadingController(Supplier<Rotation2d> heading) {
        controller = new VPIDController("Heading Controller", 4.5, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);

        this.heading = heading;
    }

    //leaving this here as a reference but this controller should not have this logic since it can not be easily transfered to pathplanner
    // public HeadingController fromTranslation(Supplier<Translation2d> pose) {
    //     var desiredAngle = Rotation2d.fromRadians(RobotState.get_instance().poseEstimator.getEstimatedPose().getTranslation().minus(pose.get()).getAngle().getRadians());
    //     return new HeadingController(() -> desiredAngle);
    // }

    public ChassisSpeeds update(ChassisSpeeds input) {
        var robotPose = FieldUtils.apply(RobotState.get_instance().poseEstimator.getEstimatedPose().getRotation());
        input.omegaRadiansPerSecond = controller.calculate(robotPose.getRadians(), heading.get().getRadians());
        Logger.recordOutput("Drive/TargetHeading", heading.get());
        return input;
    }
}
