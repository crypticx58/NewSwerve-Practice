package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.RobotContainer;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoIntakeCommand extends Command {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private boolean intaking = false;
    public AutoIntakeCommand() {
        addRequirements(intakeSubsystem);
        addRequirements(swerveSubsystem);
        addRequirements(shooterSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        intaking = false;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (intaking && !shooterSubsystem.noteDetected()) {
            Pair<ChassisSpeeds, PhotonTrackedTarget> autoData = intakeSubsystem.getAutoChassisSpeeds();
            ChassisSpeeds autoChassisSpeeds = autoData.getFirst();
            PhotonTrackedTarget bestTarget = autoData.getSecond();
            startIntakeProccess();
            swerveSubsystem.setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(autoChassisSpeeds));
        }
        if (!intaking && !shooterSubsystem.noteDetected()) {
            Pair<ChassisSpeeds, PhotonTrackedTarget> autoData = intakeSubsystem.getAutoChassisSpeeds();
            ChassisSpeeds autoChassisSpeeds = autoData.getFirst();
            PhotonTrackedTarget bestTarget = autoData.getSecond();
            startIntakeProccess();
            swerveSubsystem.setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(autoChassisSpeeds));
            intaking = true;
        } else {
            if (shooterSubsystem.noteDetected()) {
                intaking = false;
                stopIntakeProccess();
            }
        }
        
    //    if (intaking){
        
    //    } else if (!shooterSubsystem.noteDetected()) {
    //     Pair<ChassisSpeeds, PhotonTrackedTarget> autoData = intakeSubsystem.getAutoChassisSpeeds(25);
    //     ChassisSpeeds autoChassisSpeeds = autoData.getFirst();
    //     PhotonTrackedTarget bestTarget = autoData.getSecond();
    //     if (bestTarget != null && bestTarget.getPitch() <= 0){
    //         startIntakeProccess();
    //     }
        
    //     intaking = true;
    //    } else {
    //     intaking = false;
    //    }
       
    }

    public void startIntakeProccess(){

       intakeSubsystem.startIntake(1);
       shooterSubsystem.startShooter(-0.15);
       shooterSubsystem.starInterface(1);
    }

    public void stopIntakeProccess(){
        swerveSubsystem.stopSwerveModuleMotors();
         intakeSubsystem.stopIntake();
         shooterSubsystem.stopShooter();
         shooterSubsystem.stopInterface();
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return shooterSubsystem.noteDetected();
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
         stopIntakeProccess();
        if (!interrupted) {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25);
            // new Thread(()->{
            //     try {
            //         //shooterSubsystem.starInterface(-0.25);
            //         Thread.sleep(500);
            //         //shooterSubsystem.stopInterface();
            //         RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            //     } catch(Exception e) {
            //     }
            // });
        }
    }
}
