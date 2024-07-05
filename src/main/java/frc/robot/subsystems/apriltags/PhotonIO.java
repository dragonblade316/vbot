package frc.robot.subsystems.apriltags;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.apriltags.ApriltagIO.ApriltagIOInputs;

public class PhotonIO implements ApriltagIO {
    PhotonCamera cam;
    PhotonPoseEstimator poseEstimator;
    AprilTagFieldLayout field;
    

    public PhotonIO(String name) {
        cam = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);
        field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    @Override
    public void updateInputs(ApriltagIOInputs inputs) {
        var results = cam.getLatestResult().getMultiTagResult();
        var pose = poseEstimator.update();

        pose.get().targetsUsed.get(4).getBestCameraToTarget();

        inputs.robotPose = pose.get().estimatedPose.toPose2d();

        //inputs.robotPose = ;
    }
}
