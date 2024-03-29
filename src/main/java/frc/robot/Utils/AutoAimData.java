package frc.robot.Utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoAimData {
    public final ChassisSpeeds chassisSpeeds;
    public final double pivotSpeed;
    public final boolean closeEnoughToShootTarget;
    public AutoAimData(ChassisSpeeds chassisSpeeds, double pivotSpeed, boolean closeEnoughToShootTarget) {
        this.chassisSpeeds = chassisSpeeds;
        this.pivotSpeed = pivotSpeed;
        this.closeEnoughToShootTarget = closeEnoughToShootTarget;
    }
}
