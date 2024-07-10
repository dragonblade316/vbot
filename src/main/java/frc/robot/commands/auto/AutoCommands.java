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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.gamepieceseeker.GamePieceSeeker;
import frc.robot.util.FieldUtils;

public class AutoCommands {

    //This will need refactored once a custom odometry solution is made
    public static void resetPose(Pose2d pose) {
        RobotState.get_instance().poseEstimator.setPose(FieldUtils.apply(pose));
    }

    //TODO: test if this works considering the heading is not gurenteed 
    public static Command repathToChoreo(String trajName) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajName);
        
        PathConstraints constraints = new PathConstraints(5.5, 3, 3, 1);
        
        //TODO: global pathfinding constraints
        return AutoBuilder.pathfindThenFollowPath(path, constraints, 0);
    }

    public Command startShooter(Flywheel flywheel, Arm arm) {
        return Commands.run(() -> flywheel.setVelocity(RobotState.AimingFunctions.flywheelSpeed), flywheel)
            .andThen(Commands.run(() -> arm.setPosition(RobotState.AimingFunctions.armAngle), arm));
    }

    //TODO: this needs the note detector
    public static Command seek(Drive drive, GamePieceSeeker seeker) {
        return null;
    }

    
}
