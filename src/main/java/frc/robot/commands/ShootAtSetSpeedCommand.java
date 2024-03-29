package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootAtSetSpeedCommand extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final double shootSpeedRps;
    public ShootAtSetSpeedCommand(double shootSpeedRps) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.shootSpeedRps = shootSpeedRps;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.startShooter(1);
    }

    @Override
    public void execute() {
        if (shooterSubsystem.getShooterSpeedRps() >= shootSpeedRps){
            shooterSubsystem.starInterface(1);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !shooterSubsystem.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopInterface();
        if (!interrupted) {
            RobotContainer.mechanismsController.setRumble(GenericHID.RumbleType.kBothRumble, 0.25);
        }
    }
}
