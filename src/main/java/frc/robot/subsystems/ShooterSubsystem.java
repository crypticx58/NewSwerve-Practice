package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    private static ShooterSubsystem INSTANCE;
    CANSparkMax topShooterMotor;
    CANSparkMax bottomShooterMotor;
    CANSparkMax pivotMotor;
    
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ShooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        topShooterMotor = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
        topShooterMotor.restoreFactoryDefaults();
        topShooterMotor.setSmartCurrentLimit(33);
        topShooterMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        bottomShooterMotor = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
        bottomShooterMotor.restoreFactoryDefaults();
        topShooterMotor.setSmartCurrentLimit(33);
        bottomShooterMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        
        pivotMotor = new CANSparkMax(0,CANSparkLowLevel.MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setSmartCurrentLimit(33);
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    private void startShooter(double speed){
        topShooterMotor.set(speed);
        bottomShooterMotor.set(-speed);
    }

    private void stopShooter(){
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }
}


