package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final int moduleId;
    private final TalonFX driveMotor;
    private final TalonSRX turnMotor;
    public final AnalogPotentiometer absEncoder;
    private final SwerveModuleConstants swerveModuleConstants;
    private final StatusSignal<Double> driveMotorVelocityRotationsPerSec;
    private final StatusSignal<Double> driveMotorRotations;
    private final PIDController turnPIDController;
    //private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private Rotation2d lastAngle = new Rotation2d(0);
    private final TalonFXConfigurator configurator;
    public SwerveModule(SwerveModuleConstants swerveModuleConstants){
        // Drive motor setup
        driveMotor = new TalonFX(swerveModuleConstants.driveMotorId);
        TalonFXConfigurator driveMotorConfigurator = driveMotor.getConfigurator();
        configurator = driveMotorConfigurator;
        MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
        driveMotorOutputConfigs.Inverted = swerveModuleConstants.driveMotorInvertedValue;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        FeedbackConfigs driveMotorFeedbackConfigs = new FeedbackConfigs();
        driveMotorFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotorFeedbackConfigs.SensorToMechanismRatio = Constants.PhysicalConstants.kDriveMotorGearRatio; // Rotor Spins to Mechanism spins

        driveMotorVelocityRotationsPerSec = driveMotor.getVelocity(); // Velocity of mechanism aka wheel
        driveMotorRotations = driveMotor.getPosition(); // Poition of mechanicm aka wheel
        
        driveMotorConfigurator.apply(driveMotorOutputConfigs);
        driveMotorConfigurator.apply(driveMotorFeedbackConfigs);

        //Motion magic stuff for drive motor
        // final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        // motionMagicConfigs.MotionMagicCruiseVelocity = 800;
        // motionMagicConfigs.MotionMagicAcceleration = 1600;
        // motionMagicConfigs.MotionMagicJerk = 1600;

        // Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.kS = 0.25;
        // slot0Configs.kV = 1.5;
        // slot0Configs.kP = 3;
        // slot0Configs.kI = 0;
        // slot0Configs.kD = 0;

        // SmartDashboard.putNumber("KS", 0.25);
        // SmartDashboard.putNumber("KV", 1.5);
        // SmartDashboard.putNumber("KP", 3);
        // SmartDashboard.putNumber("KI", 0);
        // SmartDashboard.putNumber("KD", 0);

        // motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
        // motionMagicVelocityVoltage.withSlot(0);

        // driveMotorConfigurator.apply(slot0Configs);
        // driveMotorConfigurator.apply(motionMagicConfigs);

        // Turn motor setup
        turnMotor = new TalonSRX(swerveModuleConstants.turnMotorId);
        turnMotor.setInverted(swerveModuleConstants.turnMotorInverted);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        //turnMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute,0,0); //Absolute encoder, 4096 tics per rotor rotation
        //turnMotor.configFeedbackNotContinuous(false, 0); // 4096 => 0

        // PID turning controller
        turnPIDController = new PIDController(3.5, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Absolute Encoder
        absEncoder = new AnalogPotentiometer(swerveModuleConstants.absoluteEncoderId, 360, swerveModuleConstants.absoluteEncoderOffset);

        this.swerveModuleConstants = swerveModuleConstants;
        this.moduleId = swerveModuleConstants.moduleId;
    }
    public double getDriveSpeedMetersPerSec(){
        return driveMotorVelocityRotationsPerSec.refresh().getValue();
    }
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDriveDistanceMeters(), Rotation2d.fromDegrees(getAngleDeg()));
    }
    public double getDriveDistanceMeters(){
        return driveMotorRotations.refresh().getValue() * Constants.PhysicalConstants.kWheelCircumferenceMeters;
    }
    public double convertMetersPerSec2RotationsPerSec(double metersPerSec){
        return metersPerSec/Constants.PhysicalConstants.kWheelCircumferenceMeters;
    }
    public double convertTurnEncoderRawUnits2Degrees(double rawUnits){ //Assuming 4096 raw units or ticks per rotation
        return (rawUnits)*(1/4096)/*Rotor rotations*/*(Constants.PhysicalConstants.kTurningMotorGearRatio)/*Wheel rotation*/*360/*Degrees*/; 
    }
    public double getAngleDeg(){
        return Math.IEEEremainder(absEncoder.get(), 360);
    }
    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(getDriveSpeedMetersPerSec(), Rotation2d.fromDegrees(getAngleDeg()));
    }
    public int getModuleId(){
        return this.moduleId;
    }
    public void setSwerveModuleState(SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            desiredState.angle = lastAngle;
        }
        //System.out.println(desiredState);

        // Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.kS = SmartDashboard.getNumber("KS", 0);
        // slot0Configs.kV = SmartDashboard.getNumber("KV", 0);
        // slot0Configs.kP = SmartDashboard.getNumber("KP", 0);
        // slot0Configs.kI = SmartDashboard.getNumber("KI", 0);
        // slot0Configs.kD = SmartDashboard.getNumber("KD", 0);

        // configurator.apply(slot0Configs);

        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngleDeg()));
        //System.out.println("Speed meters/sec: "+desiredState.speedMetersPerSecond+"||| Speed rotations/sec: "+convertMetersPerSec2RotationsPerSec(desiredState.speedMetersPerSecond));
        //driveMotor.setControl(motionMagicVelocityVoltage.withVelocity(convertMetersPerSec2RotationsPerSec(desiredState.speedMetersPerSecond)));
        driveMotor.setControl(new DutyCycleOut(desiredState.speedMetersPerSecond/Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec)); //Percent output
        turnMotor.set(ControlMode.PercentOutput, turnPIDController.calculate(Units.degreesToRadians(getAngleDeg()), desiredState.angle.getRadians()));

        lastAngle = desiredState.angle;
    }
    public void stopMotors(){
        driveMotor.setControl(new DutyCycleOut(0));
        turnMotor.set(ControlMode.PercentOutput, 0);
    }
}
