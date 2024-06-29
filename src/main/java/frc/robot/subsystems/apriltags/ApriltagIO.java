package frc.robot.subsystems.apriltags;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import org.littletonrobotics.junction.AutoLog;

public interface ApriltagIO {
    @AutoLog
    public static class ApriltagIOInputs {
        //subject to change as I explore photon vision
        double tx = 0;
        double ty = 0;
        double ta = 0;
        int targetsDetected = 0;
        Pose2d robotPose = new Pose2d();
    }

    public void updateInputs(ApriltagIOInputs inputs);

    public default void accept_rotation(Rotation3d rotation, Twist3d anguler_velocity) {}


}
