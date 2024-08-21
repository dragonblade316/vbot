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
        this.maxVelocityMeters = maxVelocityMeters;
        this.robotRadius = robotRadius;
    }


    public ChassisSpeeds update(ChassisSpeeds CurrentVelocity, ChassisSpeeds desiredAccel) {
        double currentMaxAccel = maxAccel.get() * (1-(SwerveMath.calculateVel(CurrentVelocity, robotRadius)/maxVelocityMeters));
       
        // System.out.println(currentMaxAccel);
        System.out.println("before "+desiredAccel);
        if (currentMaxAccel < SwerveMath.calculateVel(desiredAccel, robotRadius)) {
            desiredAccel = desiredAccel.times(currentMaxAccel / SwerveMath.calculateVel(desiredAccel, robotRadius));
        }
        System.out.println("after "+desiredAccel);
        return desiredAccel;
    }
}
