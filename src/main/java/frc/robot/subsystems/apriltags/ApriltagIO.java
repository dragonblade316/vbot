package frc.robot.subsystems.apriltags;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.units.Measure;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

public interface ApriltagIO {
    @AutoLog
    public static class ApriltagIOInputs {
        //subject to change as I explore photon vision
        
        
        int targetsDetected = 0;
        Pose2d robotPose = new Pose2d();
        double distanceToClosestTargetMeters = 0;
        double timestamp = 0;
        
    }

    public default void updateInputs(ApriltagIOInputs inputs) {}


}
