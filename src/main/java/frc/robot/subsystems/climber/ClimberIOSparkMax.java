package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOSparkMax implements ClimberIO{
    private CANSparkMax moter;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = moter.getAppliedOutput();
        inputs.atLowerLimit = limitSwitch.get();
        inputs.encoder_value = encoder.getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        moter.setVoltage(voltage);
    }

    @Override
    public void zeroEncoder() {
        encoder.setPosition(0);
    }
}
