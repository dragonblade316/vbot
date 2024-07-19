package frc.robot.util.vlib.control;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.util.vlib.TunableDouble;

/**
 * VSimpleMotorFeedforward
 */
public class VSimpleMotorFeedforward extends SimpleMotorFeedforward {
    
    /** The static gain. */
  public TunableDouble ks;

  /** The velocity gain. */
  public TunableDouble kv;

  /** The acceleration gain. */
  public TunableDouble ka;

  /** The period. */
  private double m_dt;

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains and period. Units of the gain
   * values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   * @param dtSeconds The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  public VSimpleMotorFeedforward(String key, double ks, double kv, double ka, double dtSeconds) {

    //this is the dumbest thing I have ever done but I think it works.
    super(0, 0);

    this.ks = new TunableDouble(key + "/ks", ks);
    this.kv = new TunableDouble(key + "/kv", kv);
    this.ka = new TunableDouble(key + "/ka", ka);
    if (kv < 0.0) {
      throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
    }
    if (ka < 0.0) {
      throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
    }
    if (dtSeconds <= 0.0) {
      throw new IllegalArgumentException(
          "period must be a positive number, got " + dtSeconds + "!");
    }
    m_dt = dtSeconds;
  }

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains and period. The period is
   * defaulted to 20 ms. Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   */
  public VSimpleMotorFeedforward(String key, double ks, double kv, double ka) {
    this(key, ks, kv, ka, 0.020);
  }

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains. Acceleration gain is defaulted
   * to zero. The period is defaulted to 20 ms. Units of the gain values will dictate units of the
   * computed feedforward.
   *
   * @param ks The static gain.
   * @param kv The velocity gain.
   */
  public VSimpleMotorFeedforward(String key, double ks, double kv) {
    this(key, ks, kv, 0, 0.020);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param velocity The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward.
   * @deprecated Use the current/next velocity overload instead.
   */
  @SuppressWarnings("removal")
  @Deprecated(forRemoval = true, since = "2025")
  public double calculate(double velocity, double acceleration) {
    return ks.get() * Math.signum(velocity) + kv.get() * velocity + ka.get() * acceleration;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
   * zero).
   *
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   * @deprecated Use the current/next velocity overload instead.
   */
  @SuppressWarnings("removal")
  @Deprecated(forRemoval = true, since = "2025")
  public double calculate(double velocity) {
    return calculate(velocity, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete control when the
   * setpoint does not change.
   *
   * @param <U> The velocity parameter either as distance or angle.
   * @param setpoint The velocity setpoint.
   * @return The computed feedforward.
   */
  public <U extends Unit<U>> Measure<Voltage> calculate(Measure<Velocity<U>> setpoint) {
    return calculate(setpoint, setpoint);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete control.
   *
   * @param <U> The velocity parameter either as distance or angle.
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @return The computed feedforward.
   */
  public <U extends Unit<U>> Measure<Voltage> calculate(
      Measure<Velocity<U>> currentVelocity, Measure<Velocity<U>> nextVelocity) {
    if (ka.get() == 0.0) {
      // Given the following discrete feedforward model
      //
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //
      // where
      //
      //   A_d = eᴬᵀ
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   A = −kᵥ/kₐ
      //   B = 1/kₐ
      //
      // We want the feedforward model when kₐ = 0.
      //
      // Simplify A.
      //
      //   A = −kᵥ/kₐ
      //
      // As kₐ approaches zero, A approaches -∞.
      //
      //   A = −∞
      //
      // Simplify A_d.
      //
      //   A_d = eᴬᵀ
      //   A_d = exp(−∞)
      //   A_d = 0
      //
      // Simplify B_d.
      //
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   B_d = A⁻¹((0) - I)B
      //   B_d = A⁻¹(-I)B
      //   B_d = -A⁻¹B
      //   B_d = -(−kᵥ/kₐ)⁻¹(1/kₐ)
      //   B_d = (kᵥ/kₐ)⁻¹(1/kₐ)
      //   B_d = kₐ/kᵥ(1/kₐ)
      //   B_d = 1/kᵥ
      //
      // Substitute these into the feedforward equation.
      //
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //   uₖ = (1/kᵥ)⁺(rₖ₊₁ − (0) rₖ)
      //   uₖ = kᵥrₖ₊₁
      return Volts.of(ks.get() * Math.signum(nextVelocity.magnitude()) + kv.get() * nextVelocity.magnitude());
    } else {
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //
      // where
      //
      //   A_d = eᴬᵀ
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   A = −kᵥ/kₐ
      //   B = 1/kₐ
      double A = -kv.get() / ka.get();
      double B = 1.0 / ka.get();
      double A_d = Math.exp(A * m_dt);
      double B_d = 1.0 / A * (A_d - 1.0) * B;
      return Volts.of(
          ks.get() * Math.signum(currentVelocity.magnitude())
              + 1.0 / B_d * (nextVelocity.magnitude() - A_d * currentVelocity.magnitude()));
    }
  }

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param acceleration The acceleration of the motor.
   * @return The maximum possible velocity at the given acceleration.
   */
  public double maxAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - ks.get() - acceleration * ka.get()) / kv.get();
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param acceleration The acceleration of the motor.
   * @return The minimum possible velocity at the given acceleration.
   */
  public double minAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + ks.get() - acceleration * ka.get()) / kv.get();
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param velocity The velocity of the motor.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double velocity) {
    return (maxVoltage - ks.get() * Math.signum(velocity) - velocity * kv.get()) / ka.get();
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param velocity The velocity of the motor.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, velocity);
  } 
}