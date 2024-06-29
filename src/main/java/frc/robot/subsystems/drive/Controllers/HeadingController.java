package frc.robot.subsystems.drive.Controllers;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.VPIDController;

public class HeadingController {
    PIDController controller;
    Supplier<Rotation2d> heading;

    public HeadingController(Supplier<Rotation2d> heading) {
        controller = new PIDController(4.5, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);

        this.heading = heading;
    }

    public ChassisSpeeds update(ChassisSpeeds input) {
        input.omegaRadiansPerSecond = controller.calculate(RobotState.get_instance().robotPose.getRotation().getRadians(), heading.get().getRadians());
        Logger.recordOutput("Drive/TargetHeading", heading.get());
        return input;
    }
}
