package frc.robot.util.vlib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveMath {
    public static double calculateVel(ChassisSpeeds speeds, double robotRadius) {
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
}
