package frc.robot.subsystems.rollers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.rollers.Rollers.Goal;

//yet again stealing ideas from mech advantage 
//I think this may be utterly useless. there is no reason for this interface to exist
public interface GenericRollers<G extends GenericRollers.Goal> {
    public interface Goal {
        public Double getRpmGoal();
    }

    public G getGoal();
    public void setVoltage(double voltage);
    public SysIdRoutine getSysIdRoutine(Rollers rollers);
    public void periodic();
}
