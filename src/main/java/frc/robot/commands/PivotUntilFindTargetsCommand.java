//package frc.robot.commands;
//
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.ShooterSubsystem;
//
//import java.util.List;
//
//
//public class PivotUntilFindTargetsCommand extends Command {
//    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
//    private final double lowerBound;
//    private final double upperBound;
//    private final ProfiledPIDController pitchProfiledPIDController;
//    private final List<Integer> targetFiducialIds;
//    private boolean currentGoalIsUpperBound;
//    public PivotUntilFindTargetsCommand(double lowerBound, double upperBound, List<Integer> targetFiducialIds) {
//        this.lowerBound = lowerBound;
//        this.upperBound = upperBound;
//        this.targetFiducialIds = targetFiducialIds;
//        pitchProfiledPIDController = new ProfiledPIDController(0.075, 0, 0, new TrapezoidProfile.Constraints(10, 15));
//        pitchProfiledPIDController.setTolerance(5);
//        addRequirements(this.shooterSubsystem);
//    }
//
//    /**
//     * The initial subroutine of a command.  Called once when the command is initially scheduled.
//     */
//    @Override
//    public void initialize() {
//        if (shooterSubsystem.getPivotPosRotations()<=lowerBound){
//            pitchProfiledPIDController.setGoal(upperBound);
//            currentGoalIsUpperBound = true;
//        } else if (shooterSubsystem.getPivotPosRotations()>=upperBound) {
//            pitchProfiledPIDController.setGoal(lowerBound);
//            currentGoalIsUpperBound = false;
//        } else  {
//            pitchProfiledPIDController.setGoal(upperBound);
//            currentGoalIsUpperBound = true;
//        }
//    }
//
//    /**
//     * The main body of a command.  Called repeatedly while the command is scheduled.
//     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
//     */
//    @Override
//    public void execute() {
//        if (pitchProfiledPIDController.atGoal()){
//            if (currentGoalIsUpperBound) {
//                pitchProfiledPIDController.setGoal(lowerBound);
//                currentGoalIsUpperBound = false;
//            } else {
//                pitchProfiledPIDController.setGoal(upperBound);
//                currentGoalIsUpperBound = true;
//            }
//        }
//        shooterSubsystem.startPivot(-pitchProfiledPIDController.calculate(shooterSubsystem.getPivotPosRotations()));
//    }
//
//
//    @Override
//    public boolean isFinished() {
//        // TODO: Make this return true when this Command no longer needs to run execute()
//        return shooterSubsystem.targetsDetected(targetFiducialIds);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        shooterSubsystem.stopPivot();
//    }
//}
