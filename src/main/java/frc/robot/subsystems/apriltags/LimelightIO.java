package frc.robot.subsystems.apriltags;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class LimelightIO implements ApriltagIO {
    String name;
    boolean useMegatag2;

    public LimelightIO(String name, boolean useMegatag2) {

    }

    public void updateInputs(ApriltagIOInputs inputs) {
        PoseEstimate poseEstimate;
        if (useMegatag2) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        }
        
        inputs.robotPose = poseEstimate.pose;
        inputs.targetsDetected = poseEstimate.tagCount;
        
        inputs.visionTimestamp = poseEstimate.timestampSeconds;
        
        

        
    }

    //TODO: Update megatag2 inputs
}
