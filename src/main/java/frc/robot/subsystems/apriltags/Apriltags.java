package frc.robot.subsystems.apriltags;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.vlib.swerve.VSwervePoseEstimator;
import frc.robot.util.vlib.swerve.VSwervePoseEstimator.VisionObservation;

public class Apriltags extends SubsystemBase {

    //ApriltagIO io;
    //ApriltagIOInputsAutoLogged inputs = new ApriltagIOInputsAutoLogged();

    private class AprilCam {
        public ApriltagIO io;
        public ApriltagIOInputsAutoLogged inputs;

        public AprilCam(ApriltagIO io) {
            this.io = io;
            this.inputs = new ApriltagIOInputsAutoLogged();
        }
    }

    private AprilCam[] cameras;

    public Apriltags(ApriltagIO... cams) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        cameras = new AprilCam[cams.length];
        for (int i = 0; i < cams.length; i++) {
            cameras[i] = new AprilCam(cams[i]);
        }
    }

    @Override
    public void periodic() {
        for (AprilCam camera : cameras) {
            camera.io.updateInputs(camera.inputs);
        }
        RobotState.get_instance().poseEstimator.updateVision(null);
    }
}



