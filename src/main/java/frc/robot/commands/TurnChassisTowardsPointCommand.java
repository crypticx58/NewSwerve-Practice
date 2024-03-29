package frc.robot.commands;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class TurnChassisTowardsPointCommand extends Command {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final OdometrySubsystem odometrySubsystem = OdometrySubsystem.getInstance();
    private final PIDController turnProfiledPIDController;
    private final Vector<N2> target;
    private Rotation2d targetAngle;
    private final boolean reversed;
    public TurnChassisTowardsPointCommand(Vector<N2> target, boolean reversed) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.target = target;
        this.reversed = reversed;
        turnProfiledPIDController = new PIDController(0.65, 0, 0);
        //turnProfiledPIDController.setTolerance(0.5);
        addRequirements(this.swerveSubsystem);
    }


    @Override
    public void initialize() {
         Pose2d robotPose = odometrySubsystem.getPose();
        Pose2d forwardDir = robotPose.transformBy(new Transform2d(1,0,new Rotation2d()));
        Pose2d lateralDir = robotPose.transformBy(new Transform2d(0,1,new Rotation2d()));
        targetAngle = robotPose.getRotation().plus(OdometrySubsystem.aimChassisTowardsPoint(robotPose.getTranslation().toVector(), forwardDir.getTranslation().toVector(), lateralDir.getTranslation().toVector(), target));
        if (reversed)
            targetAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
        turnProfiledPIDController.setSetpoint(targetAngle.getRadians());
    }


    @Override
    public void execute() {
       Pose2d robotPose = odometrySubsystem.getPose();
        // Pose2d forwardDir = new Pose2d(robotPose.getTranslation(), robotPose.getRotation()).transformBy(new Transform2d(1,0,new Rotation2d()));
        // Pose2d lateralDir = new Pose2d(robotPose.getTranslation(), robotPose.getRotation()).transformBy(new Transform2d(0,1,new Rotation2d()));
        // targetAngle = new Pose2d(robotPose.getTranslation(), robotPose.getRotation()).getRotation().plus(OdometrySubsystem.aimChassisTowardsPoint(robotPose.getTranslation().toVector(), forwardDir.getTranslation().toVector(), lateralDir.getTranslation().toVector(), target));
        // turnProfiledPIDController.setGoal(targetAngle.getRadians());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,
                                                        turnProfiledPIDController.calculate(robotPose.getRotation().getRadians()));
        swerveSubsystem.setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
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
        return turnProfiledPIDController.atSetpoint();
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
        swerveSubsystem.stopSwerveModuleMotors();
        if (!interrupted){
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25);
        }
    }
}
