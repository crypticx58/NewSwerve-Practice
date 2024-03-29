// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.InputsManager.SwerveInputsManager;
import frc.robot.Utils.InputsManager.VisionTargetingInputsManager;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.Optional;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSystem = SwerveSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public static final XboxController mechanismsController =
            new XboxController(OperatorConstants.MECHANISM_CONTROLLER_PORT);

    private final SwerveJoystickCmd joystickCmd;
    private final SwerveInputsManager swerveInputsManager;
    private final VisionTargetingInputsManager visionTargetingInputsManager;
    private final SendableChooser<Command> autoChooser;
    //private final AutoBuilder autoBuilder;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        swerveInputsManager = new SwerveInputsManager(()-> -driverController.getLeftY(),
                                        ()-> -driverController.getLeftX(),
                                        ()-> -driverController.getRightX(),
                                        ()->true,
                                        0.65,
                                        0.8);
        visionTargetingInputsManager = new VisionTargetingInputsManager(mechanismsController::getXButton, mechanismsController::getAButton, mechanismsController::getBButton);
        joystickCmd = new SwerveJoystickCmd(swerveInputsManager);
        SwerveSubsystem.getInstance().setDefaultCommand(joystickCmd);
//        SmartDashboard.putNumber("xKP", 2.75);
//        SmartDashboard.putNumber("xKI", 1);
//        SmartDashboard.putNumber("xKD", 0.025);
//
//        SmartDashboard.putNumber("yKP", 2.75);
//        SmartDashboard.putNumber("yKI", 1);
//        SmartDashboard.putNumber("yKD", 0.025);
//
//        SmartDashboard.putNumber("gyroKP", 2.8);
//        SmartDashboard.putNumber("gyroKI", 1.25);
//        SmartDashboard.putNumber("gyroKD", 0.025);
        NamedCommands.registerCommand("TurnShooterTowardsSpeaker", new TurnChassisTowardsPointCommand(VisionConstants.getSpeakerTargetPoseForAlliance().toPose2d().getTranslation().toVector(), true));
//        NamedCommands.registerCommand("PivotUntilFindSpeakerTargets", new PivotUntilFindTargetsCommand(20, 35,
//                                                                     List.of(VisionConstants.getCenterSpeakerIdForAlliance(),
//                                                                      VisionConstants.getOffsetSpeakerIdForAlliance())
//                                                                      ));
        NamedCommands.registerCommand("AutoIntake", new AutoIntakeCommand());
        NamedCommands.registerCommand("AutoAimAndShoot", new AutoAimAndShootCommand(
                List.of(VisionConstants.getCenterSpeakerIdForAlliance()),
                VisionConstants.getSpeakerTargetPoseForAlliance(),
                2.5)
        );
        NamedCommands.registerCommand("PositionAndShootSpeaker", new SequentialCommandGroup(
                new TurnChassisTowardsPointCommand(VisionConstants.getSpeakerTargetPoseForAlliance().toPose2d().getTranslation().toVector(), true),
                new GoToSetRangeCommand(
                        5,
                        VisionConstants.getSpeakerTargetPoseForAlliance().toPose2d().getTranslation().toVector(),
                        Optional.of(VisionConstants.getSpeakerIdsForAlliance())
                ),
                new AutoAimAndShootCommand(
                        VisionConstants.getSpeakerIdsForAlliance(),
                        VisionConstants.getSpeakerTargetPoseForAlliance(),
                        0.25
                )
        ));
        NamedCommands.registerCommand("Shoot", new ShootAtSetSpeedCommand(200_000));
        NamedCommands.registerCommand("PathfindToWallNote", AutoBuilder.pathfindToPose(Constants.FieldConstants.getAllianceWallNotePose(), new PathConstraints(3,3, 1.75,3.5)));
        NamedCommands.registerCommand("PathfindToMiddleNote", AutoBuilder.pathfindToPose(Constants.FieldConstants.getAllianceMiddleNotePose(), new PathConstraints(3,3, 1.75,3.5)));
        NamedCommands.registerCommand("PathfindToCenterNote", AutoBuilder.pathfindToPose(Constants.FieldConstants.getAllianceCenterNotePose(), new PathConstraints(3,3, 1.75,3.5)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Routine", autoChooser);

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
    private void configureBindings() {
        new Trigger(driverController::getBButtonPressed).onTrue(new InstantCommand(swerveSystem::zeroHeading));
        new Trigger(()->driverController.getRightTriggerAxis()<1).onTrue(Commands.runEnd(
                ()->{
                    intakeSubsystem.startIntake(driverController.getRightTriggerAxis());
                    shooterSubsystem.starInterface(driverController.getRightTriggerAxis());
                    },
                ()->{
                    intakeSubsystem.stopIntake();
                    shooterSubsystem.stopShooter();
                    }));
        // intakeSubsystem.setDefaultCommand(Commands.runEnd(
        //         ()->{
        //             intakeSubsystem.startIntake(driverController.getRightTriggerAxis());
        //             shooterSubsystem.starInterface(driverController.getRightTriggerAxis());
        //             },
        //         ()->{
        //             intakeSubsystem.stopIntake();
        //             shooterSubsystem.stopShooter();
        //             },
        //         intakeSubsystem));
        new Trigger(driverController::getRightBumper).whileTrue(new AutoIntakeCommand());
        new Trigger(driverController::getYButton).whileTrue(new GoToSetRangeCommand(
                10,
                VisionConstants.getSpeakerTargetPoseForAlliance().toPose2d().getTranslation().toVector(),
                Optional.of(List.of(VisionConstants.getCenterSpeakerIdForAlliance(),
                        VisionConstants.getOffsetSpeakerIdForAlliance()))
        ));
        new Trigger(driverController::getXButton).whileTrue(
                new TurnChassisTowardsPointCommand(
                        VisionConstants.getSpeakerTargetPoseForAlliance()
                                .toPose2d()
                                .getTranslation()
                                .toVector(),
                        true
                )
        );

        
        new Trigger(mechanismsController::getLeftBumper).whileTrue(new AutoShooterCommand(swerveInputsManager, visionTargetingInputsManager));
        shooterSubsystem.setDefaultCommand(new ShooterControllerCommand(
                ()->mechanismsController.getRightY()*1,
                ()->mechanismsController.getLeftY()*-0.7,
                mechanismsController::getYButton)
        );

        new Trigger(mechanismsController::getXButton).whileTrue(new ShootAtSetSpeedCommand(200_000));


        //new Trigger(mechanismsController::getRightBumper)

        new Trigger(driverController::getLeftBumper).and(driverController::getYButton).whileTrue(swerveSystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new Trigger(driverController::getLeftBumper).and(driverController::getAButton).whileTrue(swerveSystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        new Trigger(driverController::getLeftBumper).and(driverController::getXButton).whileTrue(swerveSystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        new Trigger(driverController::getLeftBumper).and(driverController::getBButton).whileTrue(swerveSystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {

        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2.5, 1.5).setKinematics(Constants.PhysicalConstants.kDriveKinematics);

        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
        //                                          List.of(
        //                                             new Translation2d(1,0)
        //                                          ), 
        //                                         new Pose2d(new Translation2d(1,0), Rotation2d.fromDegrees(0)), trajectoryConfig);


        // ProfiledPIDController profiledPIDController =  new ProfiledPIDController(SmartDashboard.getNumber("gyroKP", 0), SmartDashboard.getNumber("gyroKI", 0), SmartDashboard.getNumber("gyroKD", 0), Constants.AutoConstants.kThetaControllerConstraints);
        // profiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //  trajectory,
        //  SwerveSubsystem.getInstance()::getPose, 
        //  Constants.PhysicalConstants.kDriveKinematics, 
        //  new PIDController(SmartDashboard.getNumber("xKP", 0), SmartDashboard.getNumber("xKI", 0), SmartDashboard.getNumber("xKD", 0)), 
        //  new PIDController(SmartDashboard.getNumber("yKP", 0), SmartDashboard.getNumber("yKI", 0), SmartDashboard.getNumber("yKD", 0)),
        //  profiledPIDController,
        //  SwerveSubsystem.getInstance()::setModuleStates,
        //  SwerveSubsystem.getInstance());
    
        // return Commands.runOnce(()->swerveSystem.resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand).andThen(Commands.runOnce(()->swerveSystem.stopSwerveModuleMotors()));
        

        return autoChooser.getSelected();
        
    }
}
