package frc.robot.Utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class StaticAutoAimData {
    public final ChassisSpeeds chassisSpeeds;
    public final boolean closeEnoughToShootTarget;
    public StaticAutoAimData() {
        this.chassisSpeeds = new ChassisSpeeds();
        this.closeEnoughToShootTarget = false;
    }
    public StaticAutoAimData(ChassisSpeeds chassisSpeeds, boolean closeEnoughToShootTarget) {
        this.chassisSpeeds = chassisSpeeds;
        this.closeEnoughToShootTarget = closeEnoughToShootTarget;
    }
}
