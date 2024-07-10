package frc.robot.subsystems.carrier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.InputMismatchException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;

public class CarrierIOSparkMax implements CarrierIO {
    private static final double GEAR_RATIO = 5;

    PowerDistribution pdh = new PowerDistribution();
    DigitalInput sensor;
    CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController feedback;

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
        motor = new CANSparkMax(12, MotorType.kBrushless);
        encoder = motor.getEncoder();
        feedback = motor.getPIDController();
    }

    public void updateInputs(CarrierIOInputs input) {
        input.isPiecePresent = sensor.get();
        input.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        input.velocityRPM = encoder.getVelocity() / GEAR_RATIO;
        input.positionRotations = encoder.getPosition() / GEAR_RATIO;
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double rpm, double ffVoltage) {
        feedback.setReference(
        //idk the ratio right now
        rpm * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVoltage,
        ArbFFUnits.kVoltage);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        feedback.setP(kp);
        feedback.setI(ki);
        feedback.setD(kd);
        feedback.setFF(0);
    }

}
