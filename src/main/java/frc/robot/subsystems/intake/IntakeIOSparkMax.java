package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
    public static final int INTAKE_MOTOR_ID = 11;

    //I thought it should be 1/5 but that kept producing 0 for some reason so now we have this
    public static final double GEAR_RATIO = 5;

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController pid;


    public IntakeIOSparkMax() {
        motor = new CANSparkMax(INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();

        //TODO: invert this motor
        //TODO: figure this out
        // encoder.setVelocityConversionFactor(1/5);
        // encoder.setPositionConversionFactor(1/5);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.velocityRPM = encoder.getVelocity() / GEAR_RATIO;
        inputs.positionRotations = encoder.getPosition() / GEAR_RATIO;
    }

    @Override
    public void setVelocity(double RPM, double ffVoltage) {
        pid.setReference(
        RPM * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVoltage,
        ArbFFUnits.kVoltage);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);
        pid.setFF(0);
    }


}
