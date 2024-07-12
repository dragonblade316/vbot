package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface ArmIO {
    
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d angle = Rotation2d.fromDegrees(0);
        public double appliedVolts = 0.0;
        public double velocityRadPerSec = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    //The arm should only ever be run in a closed loop (except I just realized this is an IO layer that should not have control over that
//    public default void setAngle(Rotation2d angle) {};

    public default void setVoltage(double voltage) {}
}
