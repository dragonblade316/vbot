package frc.robot.subsystems.extender;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ExtenderIOSparkMax implements ExtenderIO {
    CANSparkMax motor;
    SparkPIDController pid;
    RelativeEncoder encoder;

    public ExtenderIOSparkMax() {
        motor = new CANSparkMax(0, MotorType.kBrushless);
        pid = motor.getPIDController();
        encoder = motor.getEncoder();

        encoder.setPositionConversionFactor(0);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double meters) {
        pid.setReference(meters, ControlType.kPosition);
    }

    @Override 
    public void setPID(double kp, double ki, double kd) {
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);
        //I dont think this subsystem paticulerly needs a feed forward
        pid.setFF(0);
    }

    @Override
    public void zeroEncoder() {
        encoder.setPosition(0);
    }


}
