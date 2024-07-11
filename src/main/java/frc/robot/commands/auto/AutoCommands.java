package frc.robot.commands.auto;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.FlywheelState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.carrier.Carrier;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveMode;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.gamepieceseeker.GamePieceSeeker;
import frc.robot.util.FieldUtils;

public class AutoCommands {

    //This will need refactored once a custom odometry solution is made
    public static void resetPose(Pose2d pose) {
        RobotState.get_instance().poseEstimator.setPose(FieldUtils.apply(pose));
    }

    //TODO: test if this works considering the heading is not gurenteed 
    public static Command repathToChoreo(String trajName, Drive drive) {
        //if I dont put this here I gurrentee I will forget (should this be a command)
        drive.setMode(DriveMode.Auto);

        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajName);
        PathConstraints constraints = new PathConstraints(5.5, 3, 3, 1);
        
        //TODO: global pathfinding constraints
        return AutoBuilder.pathfindThenFollowPath(path, constraints, 0);
    }

    public Command startShooter(Flywheel flywheel, Arm arm) {
        return new InstantCommand(() -> flywheel.setVelocity(RobotState.AimingFunctions.flywheelSpeed), flywheel)
            .andThen(new InstantCommand(() -> arm.setPosition(RobotState.AimingFunctions.armAngle), arm));
    }

    //heading mode must be set by the auto since this can be run to attempt shoot on the move
    public Command autoShoot(Drive drive, Flywheel flywheel, Carrier carrier, Arm arm) {
        var state = RobotState.get_instance();
        return startShooter(flywheel, arm).andThen(new InstantCommand(() -> drive.setHeading(RobotState.AimingFunctions.heading))).andThen(Commands.run(() -> {
            if (state.armInPosition && state.headingAligned && state.shooterFlywheelState == FlywheelState.READY) {
                carrier.carrierShoot();
            }
        }).until(() -> state.containsPiece));
    }

    //TODO: this needs the note detector
    public static Command seek(Drive drive, GamePieceSeeker seeker) {
        return null;
    }

    
}
