package frc.robot.util.vlib.swerve.acceleration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vlib.TunableDouble;

public class ForwardAccelLimiter {

    private TunableDouble maxAccel;
    private double maxVelocityMeters;
    private double robotRadius;

    public ForwardAccelLimiter(String tableKey, double maxAccel, double maxVelocityMeters, double robotRadius)  {
        this.maxAccel = new TunableDouble(tableKey + "/MaxAccel", maxAccel);
    }

    private double calculateVel(ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double omega = speeds.omegaRadiansPerSecond;

        // Calculate linear velocity
        double linearVelocity = Math.hypot(vx, vy);

        // Calculate maximum tangential velocity due to rotation
        double tangentialVelocity = Math.abs(omega) * robotRadius;

        // Combine linear and rotational velocities
        // This is a simplification and may overestimate in some cases
        return Math.hypot(linearVelocity, tangentialVelocity);
    }

    public ChassisSpeeds update(ChassisSpeeds CurrentVelocity, ChassisSpeeds desiredAccel) {
        double currentMaxAccel = maxAccel.get() * (1-(calculateVel(CurrentVelocity)/maxVelocityMeters));
        

        if (currentMaxAccel < calculateVel(desiredAccel)) {
            desiredAccel.times(currentMaxAccel / calculateVel(desiredAccel));
        }

        return desiredAccel;
    }
}
