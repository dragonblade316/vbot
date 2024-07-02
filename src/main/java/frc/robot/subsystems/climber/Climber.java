package frc.robot.subsystems.climber;


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

        switch (Constants.currentMode) {
            case REAL:
                break;
            case REPLAY:
                break;
            case SIM:
                io.setPID(0, 0, 0);
                break;
            default:
                break;
        }
    }

    public void climberUp() {

    }

    public void climberDown() {
        
    }

    public void climberUpOverride() {

    }

    public void stopOverride() {

    }

    public void climberDownOverride() {

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        //TODO: update global state\
    }
}

