package frc.robot.subsystems;


import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
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
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.PhysicalConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());
    private SwerveSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        //new SwerveDriveOdometry(null, getHeadingRotation2d(), null)
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
        odometry.update(getHeadingRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Gyro Heading: ", getHeading());
        SmartDashboard.putNumber("Distance driven: {"+frontRight.getModuleId()+"}", frontRight.getDriveDistanceMeters()*39.37);
        SmartDashboard.putNumber("Distance driven: {"+frontLeft.getModuleId()+"}", frontLeft.getDriveDistanceMeters()*39.37);
        SmartDashboard.putNumber("Distance driven: {"+backRight.getModuleId()+"}", backRight.getDriveDistanceMeters()*39.37);
        SmartDashboard.putNumber("Distance driven: {"+backLeft.getModuleId()+"}", backLeft.getDriveDistanceMeters()*39.37);
    }
    public void resetOdometry(Pose2d newPose){
        odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), newPose);
    }
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
}

