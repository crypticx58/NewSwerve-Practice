// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveModuleConstants;

import java.util.List;
import java.util.Optional;

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
        public static final int MECHANISM_CONTROLLER_PORT = 1;
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
    public  static class FieldConstants{
        public enum BlueAllianceNotes{
            CenterNote(new Pose2d(2.9,4.1, Rotation2d.fromDegrees(0))),
            MiddleNote(new Pose2d(2.9,5.55, Rotation2d.fromDegrees(0))),
            wallNote(new Pose2d(2.9,7, Rotation2d.fromDegrees(0)));
            public final Pose2d pose2d;
            BlueAllianceNotes(Pose2d pose){
                this.pose2d = pose;
            }
        }
        public enum RedAllianceNotes{
            CenterNote(new Pose2d(13.67,0, Rotation2d.fromDegrees(180))),
            MiddleNote(new Pose2d(13.67,0, Rotation2d.fromDegrees(180))),
            wallNote(new Pose2d(13.67,0, Rotation2d.fromDegrees(180)));
            public final Pose2d pose2d;
            RedAllianceNotes(Pose2d pose){
                this.pose2d = pose;
            }
        }

        public enum MiddleFieldNotes{
            TopNote(new Pose2d(8.29,7.44, new Rotation2d())),
            MiddleTopNote(new Pose2d(8.29,5.77, new Rotation2d())),
            MiddleNote(new Pose2d(8.29,4.11, new Rotation2d())),
            MiddleBottomNote(new Pose2d(8.29,2.44, new Rotation2d())),
            BottomNote(new Pose2d(8.29,0.77, new Rotation2d()));
            public final Pose2d pose2d;
            MiddleFieldNotes(Pose2d pose){
                this.pose2d = pose;
            }
        }
        public static Pose2d getAllianceCenterNotePose(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceNotes.CenterNote.pose2d;
                } else {
                    return RedAllianceNotes.CenterNote.pose2d;
                }
            }
            return BlueAllianceNotes.CenterNote.pose2d;
        }
        public static Pose2d getAllianceMiddleNotePose(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceNotes.MiddleNote.pose2d;
                } else {
                    return RedAllianceNotes.MiddleNote.pose2d;
                }
            }
            return BlueAllianceNotes.MiddleNote.pose2d;
        }
        public static Pose2d getAllianceWallNotePose(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceNotes.wallNote.pose2d;
                } else {
                    return RedAllianceNotes.wallNote.pose2d;
                }
            }
            return BlueAllianceNotes.wallNote.pose2d;
        }
    }
    public  static class VisionConstants{
        public static final double optimalShootRangeSpeaker = 5;
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public enum BlueAllianceTargetIds{
            CenterSpeaker(7),
            LeftSpeaker(8),
            Amp(6),
            RightFeed(9),
            LeftFeed(10);
            public final int fiducialID;
            BlueAllianceTargetIds(int fiducialID){
                this.fiducialID = fiducialID;
            }
        }
        public enum RedAllianceTargetIds{
            CenterSpeaker(4), // switch these (Supposed to be 4)
            RightSpeaker(3), // switch these
            Amp(5),
            RightFeed(1),
            LeftFeed(2);
            public final int fiducialID;
            RedAllianceTargetIds(int fiducialID){
                this.fiducialID = fiducialID;
            }
        }
        public enum BlueAllianceTargetPoses{
            Speaker(aprilTagFieldLayout.getTagPose(BlueAllianceTargetIds.CenterSpeaker.fiducialID).get()
                    .transformBy(new Transform3d(
                            new Translation3d(0, 0, Units.inchesToMeters(36)),
                            new Rotation3d(0,0,0)
                    ))),
            Amp(aprilTagFieldLayout.getTagPose(BlueAllianceTargetIds.Amp.fiducialID).get()
                    .transformBy(new Transform3d(
                            new Translation3d(0, 0, -Units.inchesToMeters(8)),
                            new Rotation3d(0,0,0)
                    ))),
            Feed(aprilTagFieldLayout.getTagPose(BlueAllianceTargetIds.RightFeed.fiducialID).get()
                    .transformBy(new Transform3d(
                    new Translation3d(0, 0, 0),
                            new Rotation3d(0,0,0)
                    )));
            final Pose3d targetPose;
            BlueAllianceTargetPoses(Pose3d targetPose){
                this.targetPose = targetPose;
            }
        }
        public enum RedAllianceTargetPoses{
            Speaker(aprilTagFieldLayout.getTagPose(RedAllianceTargetIds.CenterSpeaker.fiducialID).get()
                    .transformBy(new Transform3d(
                            new Translation3d(0, 0, Units.inchesToMeters(36)),
                            new Rotation3d(0,0,0)
                    ))),
            Amp(aprilTagFieldLayout.getTagPose(RedAllianceTargetIds.Amp.fiducialID).get()
                    .transformBy(new Transform3d(
                            new Translation3d(0, 0, -Units.inchesToMeters(8)),
                            new Rotation3d(0,0,0)
                    ))),
            Feed(aprilTagFieldLayout.getTagPose(RedAllianceTargetIds.RightFeed.fiducialID).get()
                    .transformBy(new Transform3d(
                    new Translation3d(0, 0, 0),
                            new Rotation3d(0,0,0)
                    )));
            public final Pose3d targetPose;
            RedAllianceTargetPoses(Pose3d targetPose){
                this.targetPose = targetPose;
            }
        }
        public static List<Integer> getSpeakerIdsForAlliance(){
            return List.of(VisionConstants.getCenterSpeakerIdForAlliance(), VisionConstants.getOffsetSpeakerIdForAlliance());
        }
        public static int getCenterSpeakerIdForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetIds.CenterSpeaker.fiducialID;
                } else {
                    return RedAllianceTargetIds.CenterSpeaker.fiducialID;
                }
            }
            return BlueAllianceTargetIds.CenterSpeaker.fiducialID;
        }
        public static int getOffsetSpeakerIdForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetIds.LeftSpeaker.fiducialID;
                } else {
                    return RedAllianceTargetIds.RightSpeaker.fiducialID;
                }
            }
            return BlueAllianceTargetIds.LeftSpeaker.fiducialID;
        }
        public static int getAmpIdForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetIds.Amp.fiducialID;
                } else {
                    return RedAllianceTargetIds.Amp.fiducialID;
                }
            }
            return BlueAllianceTargetIds.Amp.fiducialID;
        }
        public static int getRigtFeedIdForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetIds.RightFeed.fiducialID;
                } else {
                    return RedAllianceTargetIds.RightFeed.fiducialID;
                }
            }
            return BlueAllianceTargetIds.RightFeed.fiducialID;
        }
        public static int getLeftFeedIdForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetIds.LeftFeed.fiducialID;
                } else {
                    return RedAllianceTargetIds.LeftFeed.fiducialID;
                }
            }
            return BlueAllianceTargetIds.LeftFeed.fiducialID;
        }
        public static Pose3d getSpeakerTargetPoseForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetPoses.Speaker.targetPose;
                } else {
                    return RedAllianceTargetPoses.Speaker.targetPose;
                }
            }
            return BlueAllianceTargetPoses.Speaker.targetPose;
        }
        public static Pose3d getAmpTargetPoseForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetPoses.Amp.targetPose;
                } else {
                    return RedAllianceTargetPoses.Amp.targetPose;
                }
            }
            return BlueAllianceTargetPoses.Amp.targetPose;
        }
        public static Pose3d getFeedTargetPoseForAlliance(){
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return BlueAllianceTargetPoses.Feed.targetPose;
                } else {
                    return RedAllianceTargetPoses.Feed.targetPose;
                }
            }
            return BlueAllianceTargetPoses.Feed.targetPose;
        }
    }
}
