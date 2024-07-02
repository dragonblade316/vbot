package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldUtils;

public class AutoCommands {

    //This will need refactored once a custom odometry solution is made
    public static void resetPose(Drive drive, Pose2d pose) {
        drive.setPose(FieldUtils.apply(pose));
    }

    
}
