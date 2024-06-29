package frc.robot.subsystems.intake;

public interface IntakeIO {
    public static class IntakeIOInputs {
        public double appliedVolts;

        //putting this here in case I decide to try out closed loop control for the intake
        public double velocity;
    }

    public default void setVoltage(double voltage) {}
}
