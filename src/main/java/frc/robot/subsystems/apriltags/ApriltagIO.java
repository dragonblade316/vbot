package frc.robot.subsystems.apriltags;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

public interface ApriltagIO {
    @AutoLog
    public static class ApriltagIOInputs {
        //subject to change as I explore photon vision
        
        
        int targetsDetected = 0;
        Pose2d robotPose = new Pose2d();
        double timestamp = 0;
        
        //aprilTagEntry[] tagEntries = null;

        
    }

    public class aprilTagEntry {
        int id;
        double distanceMetersToRobot;

        public aprilTagEntry(int id, double distanceMetersToRobot) {
            this.id = id;
            this.distanceMetersToRobot = distanceMetersToRobot;
        }
    }

    public default void updateInputs(ApriltagIOInputs inputs) {}


}
