package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO {
    public static final int INTAKE_MOTOR_ID = 255;

    private CANSparkMax motor;

    public IntakeIOSparkMax() {
        motor = new CANSparkMax(INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        
    }

    public void setVelocity() {
        
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }


}
