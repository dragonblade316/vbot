package frc.robot.subsystems.arm.armUtils;

public class VArmFeedforward {
    /** The static gain, in volts. */
    public final double ks;

    /** The gravity gain, in volts. */
    public final double kg;

    /** The velocity gain, in volt seconds per radian. */
    public final double kv;

    /** The acceleration gain, in volt seconds² per radian. */
    public final double ka;

    /** Arm feedforward protobuf for serialization. */

    /** Arm feedforward struct for serialization. */

    /**
     * Creates a new ArmFeedforward with the specified gains. Units of the gain values will dictate
     * units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     * @throws IllegalArgumentException for kv &lt; zero.
     * @throws IllegalArgumentException for ka &lt; zero.
     */
    public VArmFeedforward(double ks, double kg, double kv, double ka) {
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        if (kv < 0.0) {
            throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
        }
        if (ka < 0.0) {
            throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
        }
    }

    /**
     * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero.
     * Units of the gain values will dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     */
    public VArmFeedforward(double ks, double kg, double kv) {
        this(ks, kg, kv, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
     *     your encoder does not follow this convention, an offset should be added.
     * @param velocityRadPerSec The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(
        double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {

        double g = Math.cos(positionRadians - Math.PI/2);

        if (positionRadians < Math.PI/2) {
            g = 0;
        }

        return ks * Math.signum(velocityRadPerSec)
            + kg * g
            + kv * velocityRadPerSec
            + ka * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
     * zero).
     *
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
     *     your encoder does not follow this convention, an offset should be added.
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity) {
        return calculate(positionRadians, velocity, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param currentAngle The current angle in radians. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel to the floor). If
     *     your encoder does not follow this convention, an offset should be added.
     * @param currentVelocity The current velocity setpoint in radians per second.
     * @param nextVelocity The next velocity setpoint in radians per second.
     * @param dt Time between velocity setpoints in seconds.
     * @return The computed feedforward in volts.
     */
    //I dont think we need this
//    public double calculate(
//        double currentAngle, double currentVelocity, double nextVelocity, double dt) {
//        return ArmFeedforwardJNI.calculate(
//            ks, kv, ka, kg, currentAngle, currentVelocity, nextVelocity, dt);
//    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply, a position, and an
     * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
     * profile are simultaneously achievable - enter the acceleration constraint, and this will give
     * you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
     *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
     *     not follow this convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @return The maximum possible velocity at the given acceleration and angle.
     */
    public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply, a position, and an
     * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
     * profile are simultaneously achievable - enter the acceleration constraint, and this will give
     * you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
     *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
     *     not follow this convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @return The minimum possible velocity at the given acceleration and angle.
     */
    public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka) / kv;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage supply, a position, and
     * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
     * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
     *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
     *     not follow this convention, an offset should be added.
     * @param velocity The velocity of the arm.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg - velocity * kv) / ka;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage supply, a position, and
     * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
     * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
     *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
     *     not follow this convention, an offset should be added.
     * @param velocity The velocity of the arm.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, angle, velocity);
    }

}
