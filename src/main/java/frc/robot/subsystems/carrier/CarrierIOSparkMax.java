package frc.robot.subsystems.carrier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;

public class CarrierIOSparkMax implements CarrierIO {
    PowerDistribution pdh = new PowerDistribution();
    DigitalInput sensor;
    CANSparkMax motor;
    RelativeEncoder encoder;

    // -----------------------------------
    //  turns on the pdh channel that 
    //  powers the line break sensor
    // -----------------------------------
    //pdh.setSwitchableChannel(true);

    public CarrierIOSparkMax() {
        // -----------------------------------
        //  turns on the pdh channel that 
        //  powers the line break sensor
        // -----------------------------------
        pdh.setSwitchableChannel(true);

        sensor = new DigitalInput(1);
        motor = new CANSparkMax(0, MotorType.kBrushless);

    }

    public void updateInputs(CarrierIOInputs input) {
        input.isPiecePresent = sensor.get();
        input.appliedVolts = motor.getAppliedOutput();
        input.velocity = encoder.getVelocity();
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

}
