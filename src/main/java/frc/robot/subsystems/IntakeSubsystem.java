package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem INSTANCE;
    private final CANSparkMax intakeMotor;
    private final Ultrasonic ultrasonicSensor;
    private boolean noteIdle = false;
    private boolean intakeEnabled = false;
    private final MedianFilter m_filter = new MedianFilter(5);
    private final DigitalInput infaredProximitySensor;

    CANSparkMax ShooterMotor1;
    CANSparkMax ShooterMotor2;
    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private IntakeSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        intakeMotor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(33);
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        ultrasonicSensor = new Ultrasonic(2, 3);
        infaredProximitySensor = new DigitalInput(0);

        ShooterMotor1 = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
        ShooterMotor1.restoreFactoryDefaults();
        ShooterMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);

        ShooterMotor2 = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
        ShooterMotor2.restoreFactoryDefaults();
        ShooterMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public boolean getNoteIdle(){
        return noteIdle;
    }
    public boolean getIntakeEnabled(){
        return intakeEnabled;
    }
    public void startIntake(double speed){
        //intakeMotor.set(speed);
        ShooterMotor1.set(speed*0.65);
        ShooterMotor2.set(-speed*0.65);
    }

    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        noteIdle = ultrasonicSensor.getRangeInches() <= Constants.IntakeConstants.noteIdleDistanceInches;
        //
        // ultrasonicSensor.setEnabled(true);
        // ultrasonicSensor.setAutomaticMode(true);

        //ultrasonicSensor.ping();
        //SmartDashboard.putNumber("UltrasonicSensor", m_filter.calculate(ultrasonicSensor.getRangeInches()));
        //System.out.println(m_filter.calculate(ultrasonicSensor.getRangeInches()));
        
    }
}

