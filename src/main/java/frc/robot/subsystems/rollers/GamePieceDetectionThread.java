package frc.robot.subsystems.rollers;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;

public class GamePieceDetectionThread {
    private static GamePieceDetectionThread INSTANCE;
    Notifier notifier;
    Supplier<Boolean> isNotePresent;
    Supplier<Boolean> isIntaking;
    //this can be either a setVoltage or a setVelocity function. The goal is zero no matter what so just use what works best
    Consumer<Double> stopMotor;

    private GamePieceDetectionThread() {
        notifier = new Notifier(() -> check());
    }

    public static GamePieceDetectionThread getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new GamePieceDetectionThread();
        }
        return INSTANCE;
    }

    public void init(Supplier<Boolean> isNotePresent, Consumer<Double> stopMotor, Supplier<Boolean> isIntaking) { 
        this.isNotePresent = isNotePresent;
        this.stopMotor = stopMotor;
        this.isIntaking = isIntaking;
        notifier.startPeriodic(0.005);
    }

    private void check() {
        if (isNotePresent.get()) {
            stopMotor.accept(0.0);
        }
    }

    

    
}
