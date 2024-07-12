package frc.robot.util.vlib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

//TODO: This will be used to handle field cordinates for the shooter. This is here since odometry is based on the global system. it will be losely be based on https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/AllianceFlipUtil.java
public class FieldUtils {
    public static final double FIELD_LENGTH = 16.541; //in meters

    //TODO: make these work
    public static final boolean FLIPX = false;
    public static final boolean FLIPY = false;

    static boolean shouldFlip() {
        if (!DriverStation.getAlliance().isPresent()) {
            return false;
        }
        
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return true;
        }
        return false;
    }
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(FIELD_LENGTH - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
    public static double apply(double xCoordinate) {
        if (shouldFlip()) {
            return FIELD_LENGTH - xCoordinate;
        } else {
            return xCoordinate;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), -rotation.getSin());
        } else {
            return rotation;
        }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                    FIELD_LENGTH - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }
}