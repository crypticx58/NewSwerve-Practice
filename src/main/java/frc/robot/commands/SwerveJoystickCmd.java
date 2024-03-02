package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;


public class SwerveJoystickCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final Supplier<Double> xSpeedFunc, ySpeedFunc, turnSpeedFunc;
    private final Supplier<Boolean> fieldOrientatedFunc;
    private final SlewRateLimiter xRateLimiter, yRateLimiter;
    
    public SwerveJoystickCmd(Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc,
                             Supplier<Double> turnSpeedFunc, Supplier<Boolean> fieldOrientatedFunc) {
       this.xSpeedFunc = xSpeedFunc;
       this.ySpeedFunc = ySpeedFunc;
       this.turnSpeedFunc = turnSpeedFunc;
       this.fieldOrientatedFunc = fieldOrientatedFunc;
       
       this.xRateLimiter = new SlewRateLimiter(0.85);
       this.yRateLimiter = new SlewRateLimiter(0.85);
       ///this.turnRateLimiter = new SlewRateLimiter(0.5);
       addRequirements(this.swerveSubsystem);
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
        double xSpeed = xSpeedFunc.get() * 0.4;
        double ySpeed = ySpeedFunc.get() * 0.4;
        double turnSpeed = turnSpeedFunc.get() * 0.75;

        xSpeed = Math.abs(xSpeed)<Constants.IOConstants.kDeadband?0:xSpeed;
        ySpeed = Math.abs(ySpeed)<Constants.IOConstants.kDeadband?0:ySpeed;
        turnSpeed = Math.abs(turnSpeed)<Constants.IOConstants.kDeadband?0:turnSpeed;

        xSpeed = xRateLimiter.calculate(xSpeed) * Constants.OperatorConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yRateLimiter.calculate(ySpeed) * Constants.OperatorConstants.kTeleDriveMaxSpeedMetersPerSecond;
        //turnSpeed = turnRateLimiter.calculate(turnSpeed) * Constants.OperatorConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientatedFunc.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getHeadingRotation2d());
        } else{
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }
        
        SwerveModuleState[] swerveModuleStates = Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(swerveModuleStates);
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
        swerveSubsystem.stopSwerveModuleMotors();
    }
}
