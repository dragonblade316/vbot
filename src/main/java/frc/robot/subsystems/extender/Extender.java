package frc.robot.subsystems.extender;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extender extends SubsystemBase {
    ExtenderIO io;
    ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();
    //we should not need a feedforward here since there is no change in how the model behaves depending on how far it goes

    public enum Goal {
        RETRACTED(0),
        AMP(.7),
        TRAP(1.0),

        ;
        public double meters;

        private Goal(double meters) {
            this.meters = meters;
        }
    }

    public Extender(ExtenderIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
                break;
            case REPLAY:
                break;
            case SIM:
                io.setPID(3.5, 0, 0);
                break;
            default:
                break;
        }
    }
    
    public void setGoal(Goal target) {
        io.setPosition(target.meters);
    }

    //this is for overrides just in case the rope slips or something
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Extender", inputs);
    }

    public Command setGoalCommand(Goal goal) {
        return Commands.startEnd(() -> this.setGoal(goal), () -> this.setGoal(Goal.RETRACTED), this);
    }
}

