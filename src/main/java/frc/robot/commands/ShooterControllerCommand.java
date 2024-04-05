package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;


public class ShooterControllerCommand extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private Supplier<Boolean> interfaceFunc;
    private Supplier<Double> pivotFunc, shooterFunc;
    public ShooterControllerCommand(Supplier<Double> pivotFunc, Supplier<Double> shooterFunc,
                                    Supplier<Boolean> interfaceFunc) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.interfaceFunc = interfaceFunc;
        this.pivotFunc = pivotFunc;
        this.shooterFunc = shooterFunc;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double interfaceStrength = interfaceFunc.get()?1:0;
        double pivotStrength = Math.abs(pivotFunc.get())<=0.15?0:pivotFunc.get();
        double shooterStrength = Math.abs(shooterFunc.get())<=0.15?0:shooterFunc.get();
        interfaceStrength = shooterStrength<0?shooterStrength:interfaceStrength;

        //shooterSubsystem.startPivot(pivotStrength);
        shooterSubsystem.startShooter(shooterStrength);
        shooterSubsystem.starInterface(interfaceStrength);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //shooterSubsystem.stopPivot();
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopInterface();
    }
}
