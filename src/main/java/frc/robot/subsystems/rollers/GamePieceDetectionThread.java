package frc.robot.subsystems.rollers;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;

public class GamePieceDetectionThread {
    private static GamePieceDetectionThread INSTANCE;
    Notifier notifier;
    Supplier<Boolean> isNotePresent;
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

    public void init(Supplier<Boolean> isNotePresent, Consumer<Double> stopMotor) { 
        this.isNotePresent = isNotePresent;
        this.stopMotor = stopMotor;
        notifier.startPeriodic(0.01);
    }

    private void check() {
        if (isNotePresent.get()) {
            stopMotor.accept(0.0);
        }
    }

    

    
}
