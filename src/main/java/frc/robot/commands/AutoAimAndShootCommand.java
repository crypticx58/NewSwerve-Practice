package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils.StaticAutoAimData;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.time.Duration;
import java.time.Instant;
import java.util.List;


public class AutoAimAndShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final OdometrySubsystem odometrySubsystem = OdometrySubsystem.getInstance();
    private final List<Integer> fiducialIdTargets;
    private final Pose3d aimPose;
    private final double errorTolerance;
    private Instant lastSeenTarget;
    private long lastNoteDetection;
    private boolean wasCloseEnoughToShoot;

    public AutoAimAndShootCommand(List<Integer> fiducialIdTargets, Pose3d aimPose, double errorTolerance) {
        this.fiducialIdTargets = fiducialIdTargets;
        this.aimPose = aimPose;
        this.errorTolerance = errorTolerance;
        addRequirements(this.shooterSubsystem, this.swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        StaticAutoAimData autoAimData = shooterSubsystem.staticAutoAim(fiducialIdTargets, aimPose, odometrySubsystem.getPose(), errorTolerance);
        ChassisSpeeds chassisSpeeds = autoAimData.chassisSpeeds;
        boolean closeEnoughToShoot = autoAimData.closeEnoughToShootTarget;

        //shooterSubsystem.startPivot(pivotSpeed);
        swerveSubsystem.setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

        if (shooterSubsystem.noteDetected()){
            if (closeEnoughToShoot || (wasCloseEnoughToShoot && Duration.between(lastSeenTarget, Instant.now()).toMillis()<=1000)){
                lastSeenTarget = Instant.now();
                wasCloseEnoughToShoot = true;
                shooterSubsystem.startShooter(0.65);
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
    }
}
