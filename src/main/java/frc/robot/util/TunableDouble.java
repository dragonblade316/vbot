package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TunableDouble {
    private double number;
    LoggedDashboardNumber tuner;
    String key;

    public TunableDouble(String key, double default_value) {

    }
}
