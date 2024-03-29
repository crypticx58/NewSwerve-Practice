package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

public class SwerveSubsystem extends SubsystemBase {
    //private final OdometrySubsystem odometrySubsystem = OdometrySubsystem.getInstance();
    private static SwerveSubsystem INSTANCE;
    @SuppressWarnings("WeakerAccess")
    public static SwerveSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveSubsystem();
        }
        return INSTANCE;
    }
    SwerveModule frontLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontLeftModule);
    SwerveModule frontRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontRightModule);
    SwerveModule backLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackLeftModule);
    SwerveModule backRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackRightModule);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SysIdRoutine sysIdRoutine;
    private SwerveSubsystem() {
        //super.periodic();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        //new SwerveDriveOdometry(null, getHeadingRotation2d(), null)
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> driveVoltage)-> {
                                frontLeft.setVoltage(driveVoltage.in(BaseUnits.Voltage));
                                frontRight.setVoltage(driveVoltage.in(BaseUnits.Voltage));
                                backLeft.setVoltage(driveVoltage.in(BaseUnits.Voltage));
                                backRight.setVoltage(driveVoltage.in(BaseUnits.Voltage));
                        },
                        (SysIdRoutineLog log)->{
                            log
                                    .motor("frontLeft")
                                    .voltage(Volts.of(frontLeft.getAppliedVoltage()))
                                    .linearPosition(Meters.of(frontLeft.getDriveDistanceMeters()))
                                    .linearVelocity(MetersPerSecond.of(frontLeft.getDriveSpeedMetersPerSec()));
                            log
                                    .motor("frontRight")
                                    .voltage(Volts.of(frontRight.getAppliedVoltage()))
                                    .linearPosition(Meters.of(frontRight.getDriveDistanceMeters()))
                                    .linearVelocity(MetersPerSecond.of(frontRight.getDriveSpeedMetersPerSec()));
                            log
                                    .motor("backLeft")
                                    .voltage(Volts.of(backLeft.getAppliedVoltage()))
                                    .linearPosition(Meters.of(backLeft.getDriveDistanceMeters()))
                                    .linearVelocity(MetersPerSecond.of(backLeft.getDriveSpeedMetersPerSec()));
                            log
                                    .motor("backRight")
                                    .voltage(Volts.of(backRight.getAppliedVoltage()))
                                    .linearPosition(Meters.of(backRight.getDriveDistanceMeters()))
                                    .linearVelocity(MetersPerSecond.of(backRight.getDriveSpeedMetersPerSec()));
                        },
                        this
                        )
        );
    }

    public void zeroHeading(){
        gyro.reset();
        System.out.println("Gyro reset");
    }

    public double getHeading(){
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getHeadingRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopSwerveModuleMotors(){
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec);
        frontLeft.setSwerveModuleState(desiredStates[1]);
        frontRight.setSwerveModuleState(desiredStates[0]);
        backLeft.setSwerveModuleState(desiredStates[3]);
        backRight.setSwerveModuleState(desiredStates[2]);
    }

    public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{frontRight.getSwerveModuleState(),
                                    frontLeft.getSwerveModuleState(),
                                    backRight.getSwerveModuleState(),
                                    backLeft.getSwerveModuleState()};
    }
    public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{frontRight.getSwerveModulePosition(),
                                    frontLeft.getSwerveModulePosition(),
                                    backRight.getSwerveModulePosition(),
                                    backLeft.getSwerveModulePosition()};
    }
    @Override
    public void simulationPeriodic() {
//        super.simulationPeriodic();
          //System.out.println(backLeft.getDriveSpeedMetersPerSec());
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Gyro Heading: ", getHeading());
        SmartDashboard.putNumber("Distance driven: {"+frontRight.getModuleId()+"}", frontRight.getDriveDistanceMeters()*39.37);
        SmartDashboard.putNumber("Distance driven: {"+frontLeft.getModuleId()+"}", frontLeft.getDriveDistanceMeters()*39.37);
        SmartDashboard.putNumber("Distance driven: {"+backRight.getModuleId()+"}", backRight.getDriveDistanceMeters()*39.37);
        SmartDashboard.putNumber("Distance driven: {"+backLeft.getModuleId()+"}", backLeft.getDriveDistanceMeters()*39.37);

        SmartDashboard.putNumber("Drive Angle: {"+frontRight.getModuleId()+"}", frontRight.getAngleDeg());
        SmartDashboard.putNumber("Drive Angle: {"+frontLeft.getModuleId()+"}", frontLeft.getAngleDeg());
        SmartDashboard.putNumber("Drive Angle: {"+backRight.getModuleId()+"}", backRight.getAngleDeg());
        SmartDashboard.putNumber("Drive Angle: {"+backLeft.getModuleId()+"}", backLeft.getAngleDeg());
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.PhysicalConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}

