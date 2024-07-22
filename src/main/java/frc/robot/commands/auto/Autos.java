package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.rollers.Rollers;

public class Autos {
    public static Command NeutralGood(Drive drive, Arm arm, Flywheel flywheel, Rollers rollers) {

        Command shootInPlace = AutoCommands.turnInPlace(drive)
            .alongWith(AutoCommands.autoShoot(drive, flywheel, rollers, arm))
            .until(() -> !RobotState.get_instance().containsPiece);
        Command intake = new InstantCommand(() -> rollers.setGoal(Rollers.Goal.Intake));
 

        return AutoCommands.resetPose(new Pose2d(0.7032250761985779, 6.7294135093688965, Rotation2d.fromRadians(1.0441690157952843))).andThen(
            AutoCommands.startShooter(flywheel, arm),
            AutoCommands.repathToChoreo("SAtoN1", drive),
            AutoCommands.turnInPlace(drive)
            .alongWith(AutoCommands.autoShoot(drive, flywheel, rollers, arm))
            .until(() -> !RobotState.get_instance().containsPiece), 
            new InstantCommand(() -> rollers.setGoal(Rollers.Goal.Intake)),
            AutoCommands.repathToChoreo("SAtoN1.1", drive),
            Commands.either(
                AutoCommands.turnInPlace(drive)
                    .alongWith(AutoCommands.autoShoot(drive, flywheel, rollers, arm))
                    .until(() -> !RobotState.get_instance().containsPiece), 
                AutoCommands.seek(drive, null).andThen(
                    Commands.either(shootInPlace, Commands.none(), () -> RobotState.get_instance().containsPiece
                )), 
                () -> RobotState.get_instance().containsPiece
            ),
            new InstantCommand(() -> rollers.setGoal(Rollers.Goal.Intake)),
            AutoCommands.repathToChoreo("N1toC1", drive)
        );
    }

    
}
