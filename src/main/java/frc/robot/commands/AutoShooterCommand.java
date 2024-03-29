package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utils.InputsManager.SwerveInputsManager;
import frc.robot.Utils.InputsManager.VisionTargetingInputsManager;
import frc.robot.Utils.StaticAutoAimData;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;


public class AutoShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final SwerveSubsystem  swerveSubsystem = SwerveSubsystem.getInstance();
    private final OdometrySubsystem odometrySubsystem = OdometrySubsystem.getInstance();
    private final SwerveInputsManager swerveInputsManager;
    private final VisionTargetingInputsManager visionTargetingInputsManager;
    private Instant lastSeenTarget;
    private long lastNoteDetection;
    private boolean wasCloseEnoughToShoot;
    public AutoShooterCommand(SwerveInputsManager swerveInputsManager, VisionTargetingInputsManager visionTargetingInputsManager) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.swerveInputsManager = swerveInputsManager;
        this.visionTargetingInputsManager = visionTargetingInputsManager;
        addRequirements(this.swerveSubsystem);
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        //shooterSubsystem.startShooter(0.65);
    }

    @Override
    public void execute() {
        List<Integer> fiducialIdTargets = new ArrayList<Integer>();
        Pose3d aimPose = new Pose3d();

        if (visionTargetingInputsManager.getSpeakerAim()){
            fiducialIdTargets.add(Constants.VisionConstants.getCenterSpeakerIdForAlliance());
            fiducialIdTargets.add(Constants.VisionConstants.getOffsetSpeakerIdForAlliance());
            aimPose = Constants.VisionConstants.getSpeakerTargetPoseForAlliance();
        } else if (visionTargetingInputsManager.getAmpAim()) {
            fiducialIdTargets.add(Constants.VisionConstants.getAmpIdForAlliance());
            aimPose = Constants.VisionConstants.getAmpTargetPoseForAlliance();
        } else if (visionTargetingInputsManager.getFeedAim()) {
            fiducialIdTargets.add(Constants.VisionConstants.getRigtFeedIdForAlliance());
            fiducialIdTargets.add(Constants.VisionConstants.getLeftFeedIdForAlliance());
            Constants.VisionConstants.getFeedTargetPoseForAlliance();
        }

        StaticAutoAimData autoAimData = shooterSubsystem.staticAutoAim(fiducialIdTargets, aimPose, odometrySubsystem.getPose(), 2.5);
        ChassisSpeeds chassisSpeeds = autoAimData.chassisSpeeds;
        //double pivotSpeed = autoAimData.pivotSpeed;
        boolean closeEnoughToShoot = autoAimData.closeEnoughToShootTarget;

        ChassisSpeeds swerveChassisSpeeds = swerveInputsManager.getChassisSpeeds().times(0.25);
        chassisSpeeds.vxMetersPerSecond = swerveChassisSpeeds.vxMetersPerSecond;
        chassisSpeeds.vyMetersPerSecond = swerveChassisSpeeds.vyMetersPerSecond;

        //shooterSubsystem.startPivot(pivotSpeed);
        swerveSubsystem.setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

        if (shooterSubsystem.noteDetected()){
            if (closeEnoughToShoot || (wasCloseEnoughToShoot && Duration.between(lastSeenTarget, Instant.now()).toMillis()<=1000)){
                lastSeenTarget = Instant.now();
                wasCloseEnoughToShoot = true;
                shooterSubsystem.startShooter(1);
                if (shooterSubsystem.getShooterSpeedRps() >= 200000){
                    shooterSubsystem.starInterface(1);
                    //lastNoteDetection = System.currentTimeMillis();
                }
            }
        } else {
            wasCloseEnoughToShoot = false;
            // if ((System.currentTimeMillis()-lastNoteDetection) <= 750){
            //     shooterSubsystem.starInterface(1);
            // }
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !shooterSubsystem.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopSwerveModuleMotors();
        //shooterSubsystem.stopPivot();
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopInterface();
        if (!interrupted) {
            RobotContainer.mechanismsController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25);
        }
    }
}
