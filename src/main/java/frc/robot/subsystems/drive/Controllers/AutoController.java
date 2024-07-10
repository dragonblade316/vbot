package frc.robot.subsystems.drive.Controllers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


//wow this is so complex (why did I make this necicary)
public class AutoController {
    private ChassisSpeeds speeds = new ChassisSpeeds();
    
    
    public void updateInputs(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public ChassisSpeeds update() {
        System.out.println("update");
        return speeds;
    }
}
