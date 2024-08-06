package frc.robot.util.vlib.swerve.acceleration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vlib.TunableDouble;

public class TiltAccelLimiter {
    private TunableDouble XMaxAccel;
    private TunableDouble YMaxAccel;

    public TiltAccelLimiter(String tableKey, double XMaxAccel, double YMaxAccel) {
        this.XMaxAccel = new TunableDouble(tableKey + "/XMaxAccel", XMaxAccel);
        this.YMaxAccel = new TunableDouble(tableKey + "/YMaxAccel", YMaxAccel);
    }

    public ChassisSpeeds update(ChassisSpeeds desiredAccel) {
        double x = desiredAccel.vxMetersPerSecond;
        double y = desiredAccel.vyMetersPerSecond;

        return new ChassisSpeeds(
            Math.min(Math.abs(x), XMaxAccel.get()) * Math.signum(x), 
            Math.min(Math.abs(y), YMaxAccel.get()) * Math.signum(y), 
            desiredAccel.omegaRadiansPerSecond);
    }
}
