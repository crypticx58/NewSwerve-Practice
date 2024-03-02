package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class AutoIntakeCommand extends Command {

    SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    PhotonCamera camera;
    PIDController turnPIDController;
    PIDController drivePIDController;
    public AutoIntakeCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        turnPIDController = new PIDController(0.25,0,0);
        drivePIDController = new PIDController(0.25, 0, 0);
        addRequirements(swerveSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        intakeSubsystem.startIntake(0.75);
        if (result.hasTargets()){
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds((drivePIDController.calculate(bestTarget.getArea(), 100)/100)*1.5,
                                                            0, 
                                                            turnPIDController.calculate(bestTarget.getYaw()/30, 0)*1.5);
            //System.out.println(bestTarget.getArea());
            //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.35*Math.abs((50-bestTarget.getArea())/100), 0, -bestTarget.getYaw()/30);
            // intakeSubsystem.startIntake(0.45);
            swerveSubsystem.setModuleStates(PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        } else {
            // intakeSubsystem.stopIntake();
        }
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
        return false;
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
         swerveSubsystem.setModuleStates(PhysicalConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0)));
         intakeSubsystem.stopIntake();
    }
}
