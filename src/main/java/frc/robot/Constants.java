// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class PhysicalConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 6.667;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kPhysicalMaxSpeedMetersPerSec = 1; // Untested

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // distance between riht and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //
    }

    public static class SwerveModuleConstantsInstances {
        public static final SwerveModuleConstants kFrontLeftModule = new SwerveModuleConstants(1,
                11, 21, 3, -282.91,
                InvertedValue.CounterClockwise_Positive, false,
                false);

        public static final SwerveModuleConstants kFrontRightModule = new SwerveModuleConstants(2,
                14, 24, 0, -338.84,
                InvertedValue.Clockwise_Positive, false,
                false);

        public static final SwerveModuleConstants kBackRightModule = new SwerveModuleConstants(3,
                13, 23, 2, -86.67,
                InvertedValue.Clockwise_Positive, false,
                false);

        public static final SwerveModuleConstants kBackLeftModule = new SwerveModuleConstants(4,
                12, 22, 1, -217.85,
                InvertedValue.CounterClockwise_Positive, false,
                false);

    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = 1;
        public static double kTeleDriveMaxSpeedMetersPerSecond = 1.15; //Test values
        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 0.75; //Tst Values
        public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1;
    }
    public class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 1.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kPXController = 3.5;
        public static final double kPYController = 3.5;
    
        public static final double kPThetaController = 0.2;
    
        public static final edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints kThetaControllerConstraints = new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(90, 90);
    
    }
    public static class IOConstants {
        public static final double kDeadband = 0.1;
    }

    public static class IntakeConstants {
        public static final double noteIdleDistanceInches = 0.5;
    }
}
