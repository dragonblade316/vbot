package frc.robot.util.vlib.swerve.acceleration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vlib.TunableDouble;
import frc.robot.util.vlib.swerve.SwerveMath;

public class SkidAccelLimiter {
    private TunableDouble maxSkidAccel;
    private double robotRadius = 0.0;

    public SkidAccelLimiter(String tablekey, double maxSkidAccel, double robotRadius) {
        this.maxSkidAccel = new TunableDouble(tablekey + "/SkidMaxAccel", maxSkidAccel);
        this.robotRadius = robotRadius;
    }

    public ChassisSpeeds update(ChassisSpeeds wantedAccel) {

        double accel = SwerveMath.calculateVel(wantedAccel, robotRadius);
        if (accel > maxSkidAccel.get()) {
            wantedAccel = wantedAccel.times(maxSkidAccel.get() / accel);
        }

        //TODO: figure this out
        return wantedAccel;
    }
}
