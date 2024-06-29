package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOSparkMax implements ArmIO{
    CANSparkMax moter = new CANSparkMax(1, MotorType.kBrushless);
    CANcoder encoder = new CANcoder(1);

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(encoder.getPosition().getValue() - 0.0); //TODO: offset.
        inputs.appliedVolts = moter.getAppliedOutput();
    }

    @Override
    public void setVoltage(double voltage) {
        moter.setVoltage(voltage);
    }
}