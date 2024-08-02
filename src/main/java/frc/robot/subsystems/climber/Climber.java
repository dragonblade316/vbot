package frc.robot.subsystems.climber;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        this.io = io;

        switch (Constants.getMode()) {
            case REAL:
                break;
            // case REPLAY:
            //     break;
            case SIM:
                io.setPID(0, 0, 0);
                break;
            default:
                break;
        }
    }

    //TODO: figure out how on earth the climber will work

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    } 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        //TODO: update global state\
    }
}

