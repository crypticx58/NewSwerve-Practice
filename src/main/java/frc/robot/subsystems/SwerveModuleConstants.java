package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveModuleConstants {
    public final int moduleId;
    public final int driveMotorId;
    public final int turnMotorId;
    public final int absoluteEncoderId;
    public final double absoluteEncoderOffset;
    public final InvertedValue driveMotorInvertedValue;
    public final boolean turnMotorInverted;
    public final boolean absoluteEncoderInverted;
    public SwerveModuleConstants(int moduleId, int driveMotorId, int turnMotorId, int absoluteEncoderId,
            double absoluteEncoderOffset, InvertedValue driveMotorInvertedValue, boolean turnMotorInverted,
            boolean absoluteEncoderInverted) {
        this.moduleId = moduleId;
        this.driveMotorId = driveMotorId;
        this.turnMotorId = turnMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.driveMotorInvertedValue = driveMotorInvertedValue;
        this.turnMotorInverted = turnMotorInverted;
        this.absoluteEncoderInverted = absoluteEncoderInverted;
    }
}
