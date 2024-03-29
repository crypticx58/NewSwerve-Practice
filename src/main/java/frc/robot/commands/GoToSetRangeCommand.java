package frc.robot.commands;

import edu.wpi.first.math.Nat;
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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.Optional;



public class GoToSetRangeCommand extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final OdometrySubsystem odometrySubsystem = OdometrySubsystem.getInstance();
    private final double setRange;
    private final Vector<N2> targetVec;
    private final PIDController distancePIDController;
    private final Optional<List<Integer>> targetFiducialIds;

    public GoToSetRangeCommand(double range, Vector<N2> targetVec, Optional<List<Integer>> targetFiducialIds) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.setRange = range;
        this.targetVec = targetVec;
        distancePIDController = new PIDController(0.25,0,0);
        this.targetFiducialIds = targetFiducialIds;
        distancePIDController.setTolerance(0.1);
        addRequirements(this.shooterSubsystem, this.swerveSubsystem);
    }

    @Override
    public void initialize() {
        distancePIDController.reset();
        distancePIDController.setSetpoint(setRange);
    }

    @Override
    public void execute() {
        Pose2d robotPose = odometrySubsystem.getPose();
        Vector<N2> robotVec = robotPose.getTranslation().toVector();
        Vector<N2> targetPoseRelativeToRobotPose = targetVec.minus(robotVec);
        double distanceFromTarget = targetPoseRelativeToRobotPose.norm();

        Vector<N2> traversalVector = new Vector<N2>(Nat.N2());
        traversalVector.set(0,0,targetPoseRelativeToRobotPose.get(0,0));
        traversalVector.set(1,0,targetPoseRelativeToRobotPose.get(1,0));
        traversalVector = traversalVector.unit().times(-distancePIDController.calculate(distanceFromTarget));

        Vector<N2> robotForwardVec = robotPose.transformBy(new Transform2d(1, 0, new Rotation2d())).getTranslation().toVector().minus(robotVec);
        Vector<N2> robotLateralVec = robotPose.transformBy(new Transform2d(0, 1, new Rotation2d())).getTranslation().toVector().minus(robotVec);

        Vector<N2> traversalForwardComponent = robotForwardVec.times(traversalVector.dot(robotForwardVec));
        Vector<N2> traversalLateralComponent = robotLateralVec.times(traversalVector.dot(robotLateralVec));

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                traversalForwardComponent.norm(),
                traversalLateralComponent.norm(),
                0
        );

        swerveSubsystem.setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (distancePIDController.atSetpoint() || (targetFiducialIds.isPresent() ? shooterSubsystem.targetsDetected(targetFiducialIds.get()) : false));
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25);
            new Thread(()->{
                try {
                    
                    Thread.sleep(500);
                    
                    RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                } catch(Exception e) {
                }
            });
        }
    }
}
