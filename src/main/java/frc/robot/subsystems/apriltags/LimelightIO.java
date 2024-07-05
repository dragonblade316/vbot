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
        
        inputs.timestamp = poseEstimate.timestampSeconds;
        
        //inputs.tagEntries = new aprilTagEntry[poseEstimate.rawFiducials.length];
        for (int i = 0; i <= poseEstimate.rawFiducials.length -1; i++) {
            //inputs.tagEntries[i] = new aprilTagEntry(poseEstimate.rawFiducials[i].id, poseEstimate.rawFiducials[i].distToRobot);
        }

        
    }

    //TODO: Update megatag2 inputs
}
