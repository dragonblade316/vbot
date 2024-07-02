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

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController pid;

    public IntakeIOSparkMax() {
        motor = new CANSparkMax(INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

        //TODO: figure this out
        encoder.setVelocityConversionFactor(1/5);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput();
        inputs.velocity = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }

    @Override
    public void setVelocity(double radiansPerSecond, double ffVoltage) {
        pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond),
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
