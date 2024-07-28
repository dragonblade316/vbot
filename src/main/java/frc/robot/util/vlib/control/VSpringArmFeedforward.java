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
        //double g = Math.cos(positionRadians);

        if (positionRadians < Units.degreesToRadians(45)) {
            g = 0;
        }


        //If I finish this (which is not gurenteed) then this math needs to be rewriten.
        //basically the gravity component (kg) needs to return to normal and an antigravity component needs to be added to compensate for the springs
        //the antigravity component will either be a modified gravity component where the grav is highest straght up. or I will do this https://discord.com/channels/176186766946992128/1255246812705587303/1255246812705587303
        return super.ks.get() * Math.signum(velocityRadPerSec)
            + super.kg.get() * g
            + super.kv.get() * velocityRadPerSec
            + super.ka.get() * accelRadPerSecSquared;
    }
}
