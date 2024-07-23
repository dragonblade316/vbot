package frc.robot.util.vlib.swerve.acceleration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vlib.TunableDouble;

public class SkidAccelLimiter {
    private TunableDouble maxSkidAccel;

    public SkidAccelLimiter(double maxSkidAccel) {

    }

    public ChassisSpeeds update(ChassisSpeeds wantedAccel) {
        //TODO: figure this out
        return wantedAccel;
    }
}
