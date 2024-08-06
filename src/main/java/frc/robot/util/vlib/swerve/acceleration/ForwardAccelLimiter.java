package frc.robot.util.vlib.swerve.acceleration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vlib.TunableDouble;
import frc.robot.util.vlib.swerve.SwerveMath;

public class ForwardAccelLimiter {

    private TunableDouble maxAccel;
    private double maxVelocityMeters;
    private double robotRadius;

    public ForwardAccelLimiter(String tableKey, double maxAccel, double maxVelocityMeters, double robotRadius)  {
        this.maxAccel = new TunableDouble(tableKey + "/ForwardMaxAccel", maxAccel);
    }


    public ChassisSpeeds update(ChassisSpeeds CurrentVelocity, ChassisSpeeds desiredAccel) {
        double currentMaxAccel = maxAccel.get() * (1-(SwerveMath.calculateVel(CurrentVelocity, robotRadius)/maxVelocityMeters));
        

        if (currentMaxAccel < SwerveMath.calculateVel(desiredAccel, robotRadius)) {
            desiredAccel.times(currentMaxAccel / SwerveMath.calculateVel(desiredAccel, robotRadius));
        }

        return desiredAccel;
    }
}
