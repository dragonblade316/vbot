package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOSparkMax implements ClimberIO{
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;

    public ClimberIOSparkMax() {
        motor = new CANSparkMax(0, MotorType.kBrushless);
        encoder = motor.getEncoder();
        limitSwitch = new DigitalInput(0);

        encoder.setPositionConversionFactor(0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput();
        inputs.atLowerLimit = limitSwitch.get();
        inputs.heightFromBaseMeters = encoder.getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void zeroEncoder() {
        encoder.setPosition(0);
    }
}
