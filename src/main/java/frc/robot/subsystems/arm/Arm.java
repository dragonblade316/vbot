package frc.robot.subsystems.arm;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.armUtils.VArmFeedforward;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private VArmFeedforward feedforward;
    //TODO: fix the VPID and use here
    private PIDController feedback;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(18);
    private TrapezoidProfile.State setPointState;

    //remember, this is all in Rads Per Second.
    TrapezoidProfile profile = new TrapezoidProfile(new Constraints(1, 1));

    private static final int LOWEST_ANGLE = 12;
    private static final int HIGHEST_ANGLE = 74;

    private boolean climblock = false;

    //all of these are in degrees, they will be converted to rads
    public enum Position {
        TRAVERSE(17),
        AMP(45),
        TRAP(43.5),
        SPEAKER_DEAD_REKON(50),
        CLIMB(73);

        public final Double angle;

        private Position(double angle) {
            this.angle = angle;
        }
    }

    public Arm(ArmIO io) {
        switch (Constants.currentMode) {
            case REAL:
                feedforward = new VArmFeedforward(0, 0, 0);
                feedback = new PIDController(0, 0, 0);
                break;
        
            default:
                feedforward = new VArmFeedforward(0, 0, 0);
                feedback = new PIDController(0, 0, 0);
                break;
        }

        io.updateInputs(inputs);
        setPointState = new TrapezoidProfile.State(inputs.angle.getRadians(), inputs.velocityRadPerSec);
    }

    public void setPosition(Rotation2d angle) {
        targetAngle = angle;
    }

    public void setPosition(Position position) {
        setPosition(Rotation2d.fromDegrees(position.angle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        //Tomorow we are going to need to figure out how to create a TrapezoidProfile.state setpoint
        var setpointState = profile.calculate(
            0.02, 
            setPointState, new TrapezoidProfile.State(
            MathUtil.clamp(
                targetAngle.getRadians(),
                Units.degreesToRadians(LOWEST_ANGLE),
                Units.degreesToRadians(HIGHEST_ANGLE)),
            0.0));
        io.setVoltage(feedback.calculate(inputs.angle.getRadians(), targetAngle.getRadians()) + feedforward.calculate(setpointState.position, setpointState.velocity));
    }
}

