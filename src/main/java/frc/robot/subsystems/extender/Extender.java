package frc.robot.subsystems.extender;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extender extends SubsystemBase {
    ExtenderIO io;
    ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    public enum Target {
        RETRACTED(0),
        AMP(.7),
        TRAP(1.0),

        ;
        public double meters;

        private Target(double meters) {
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
                io.setPID(0.1, 0, 0);
                break;
        
            default:
                break;
        }
    }
    
    public void moveToTarget(Target target) {
        io.setPosition(target.meters);
    }

    //this is for overrides just in case the rope slips or something
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
}

