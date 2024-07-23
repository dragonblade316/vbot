package frc.robot.subsystems.apriltags;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.vlib.swerve.VSwervePoseEstimator;
import frc.robot.util.vlib.swerve.VSwervePoseEstimator.VisionObservation;

public class Apriltags extends SubsystemBase {

    ApriltagIO io;
    ApriltagIOInputsAutoLogged inputs = new ApriltagIOInputsAutoLogged();


    public Apriltags() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        RobotState.get_instance().poseEstimator.updateVision(null);
    }
}



