// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final SwerveJoystickCmd joystickCmd;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        joystickCmd = new SwerveJoystickCmd(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()-> -driverController.getRightX(), ()->true);
        SwerveSubsystem.getInstance().setDefaultCommand(joystickCmd);
        SmartDashboard.putNumber("xKP", 2.75);
        SmartDashboard.putNumber("xKI", 1);
        SmartDashboard.putNumber("xKD", 0.025);

        SmartDashboard.putNumber("yKP", 2.75);
        SmartDashboard.putNumber("yKI", 1);
        SmartDashboard.putNumber("yKD", 0.025);

        SmartDashboard.putNumber("gyroKP", 2.8);
        SmartDashboard.putNumber("gyroKI", 1.25);
        SmartDashboard.putNumber("gyroKD", 0.025);
        configureBindings();

        
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
//        new Trigger(exampleSubsystem::exampleCondition)
//                .onTrue(new ExampleCommand(exampleSubsystem));
//
//        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
//        // cancelling on release.
//        driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand()
        SwerveSubsystem sys = SwerveSubsystem.getInstance();
        new Trigger(driverController::getBButtonPressed).onTrue(new InstantCommand(sys::zeroHeading));
        //IntakeSubsystem intake = IntakeSubsystem.getInstance();

        //intakeSubsystem.setDefaultCommand(new StartEndCommand(()->intakeSubsystem.startIntake(driverController.getLeftTriggerAxis()), ()->intakeSubsystem.stopIntake(), intakeSubsystem));
        intakeSubsystem.setDefaultCommand(Commands.runEnd(()->intakeSubsystem.startIntake(driverController.getLeftTriggerAxis()*0.65), intakeSubsystem::stopIntake, intakeSubsystem));
        // intakeSubsystem.setDefaultCommand(new StartEndCommand(()->intake.startIntake(0.5), ()->intake.stopIntake(), intake));
        new Trigger(driverController::getRightBumper).whileTrue(new AutoIntakeCommand());
        
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2.5, 1.5).setKinematics(Constants.PhysicalConstants.kDriveKinematics);

        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
        //                                          List.of(
        //                                             new Translation2d(2,2),
        //                                             new Translation2d(4,0),
        //                                             new Translation2d(2,-2)
        //                                          ), 
        //                                         new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), trajectoryConfig);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                                                 List.of(
                                                    new Translation2d(2,0),
                                                    new Translation2d(2,1.5),
                                                    new Translation2d(4,1.5)
                                                 ), 
                                                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), trajectoryConfig);


        ProfiledPIDController profiledPIDController =  new ProfiledPIDController(SmartDashboard.getNumber("gyroKP", 0), SmartDashboard.getNumber("gyroKI", 0), SmartDashboard.getNumber("gyroKD", 0), Constants.AutoConstants.kThetaControllerConstraints);
        profiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
         trajectory,
         SwerveSubsystem.getInstance()::getPose, 
         Constants.PhysicalConstants.kDriveKinematics, 
         new PIDController(SmartDashboard.getNumber("xKP", 0), SmartDashboard.getNumber("xKI", 0), SmartDashboard.getNumber("xKD", 0)), 
         new PIDController(SmartDashboard.getNumber("yKP", 0), SmartDashboard.getNumber("yKI", 0), SmartDashboard.getNumber("yKD", 0)),
         profiledPIDController,
         SwerveSubsystem.getInstance()::setModuleStates,
         SwerveSubsystem.getInstance());
    
        return Commands.runOnce(()->SwerveSubsystem.getInstance().resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand).andThen(Commands.runOnce(()->SwerveSubsystem.getInstance().stopSwerveModuleMotors()));
        
        // An example command will be run in autonomous
       // return Autos.exampleAuto(exampleSubsystem);
    }
}
