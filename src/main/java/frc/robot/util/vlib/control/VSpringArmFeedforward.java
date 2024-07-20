package frc.robot.util.vlib.control;

import edu.wpi.first.math.util.Units;


/* This class is made for VERN's 2024 robot Bebop.
* It may be useful on other robots other than Bebop but idk how useful
*/
public class VSpringArmFeedforward extends VArmFeedforward {
    public VSpringArmFeedforward(String key, double ks, double kg, double kv, double ka) {
        super(key, ks, kg, kv, ka);
        
    }

    public VSpringArmFeedforward(String key, double ks, double kg, double kv) {
        super(key, ks, kg, kv);
    }

    @Override
    public double calculate(
        double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {

        double g = Math.cos(positionRadians - Units.degreesToRadians(45));

        if (positionRadians < Units.degreesToRadians(45)) {
            g = 0;
        }

        return ks.get() * Math.signum(velocityRadPerSec)
            + kg.get() * g
            + kv.get() * velocityRadPerSec
            + ka.get() * accelRadPerSecSquared;
    }
}