package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.StaticAutoAimData;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class ShooterSubsystem extends SubsystemBase {
    
    private static ShooterSubsystem INSTANCE;
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    OdometrySubsystem odometrySubsystem = OdometrySubsystem.getInstance();
    CANSparkMax topShooterMotor;
    CANSparkMax bottomShooterMotor;
    CANSparkMax pivotMotor;
    CANSparkMax interfaceMotor;
    RelativeEncoder pivotEncoder;
    SparkPIDController pivotPIDController;
    PhotonCamera shooterCamera;
    ProfiledPIDController turnProfiledPIDController;
    ProfiledPIDController pitchProfiledPIDController;
    ElevatorFeedforward pivotFeedforward;
    DigitalInput proximitySensor;
    double previousPipelineTimeStamp;
    
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
        topShooterMotor.setSmartCurrentLimit(20);
        topShooterMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        bottomShooterMotor = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
        bottomShooterMotor.restoreFactoryDefaults();
        topShooterMotor.setSmartCurrentLimit(20);
        bottomShooterMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        
//        pivotMotor = new CANSparkMax(5,CANSparkLowLevel.MotorType.kBrushless);
//        pivotMotor.restoreFactoryDefaults();
//        pivotMotor.setSmartCurrentLimit(35);
//        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
//        pivotEncoder = pivotMotor.getEncoder();

        interfaceMotor = new CANSparkMax(3,CANSparkLowLevel.MotorType.kBrushed);
        //interfaceMotor.restoreFactoryDefaults();
        interfaceMotor.setSmartCurrentLimit(20);
        interfaceMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shooterCamera = new PhotonCamera("ShooterAimer");
        PhotonCamera.setVersionCheckEnabled(false);

        // pivotPIDController = pivotMotor.getPIDController();
        // pivotPIDController.setP(3.35);
        // pivotPIDController.setI(0);
        // pivotPIDController.setD(0);

        // int smartMotionSlot = 0;
        // pivotPIDController.setSmartMotionMaxVelocity(50000, smartMotionSlot);
        // pivotPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        // pivotPIDController.setSmartMotionMaxAccel(25000, smartMotionSlot);
        // pivotPIDController.setSmartMotionAllowedClosedLoopError(50000, smartMotionSlot);

        // pivotPIDController.setOutputRange(-1, 1);

        proximitySensor = new DigitalInput(9);

        pitchProfiledPIDController = new ProfiledPIDController(0.0225, 0, 0, new TrapezoidProfile.Constraints(15, 30));
        turnProfiledPIDController = new ProfiledPIDController(0.0225, 0, 0, new TrapezoidProfile.Constraints(15, 30));
        pivotFeedforward = new ElevatorFeedforward(0, 0, 0);
    }
    public void starInterface(double speed){
        interfaceMotor.set(speed);
    }
    public void stopInterface(){
        interfaceMotor.stopMotor();
    }
    public void startShooter(double speed){
        topShooterMotor.set(speed);
        bottomShooterMotor.set(-speed);
    }
    public void stopShooter(){
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }
//    public void startPivot(double velocitySetpoint){
//        //pivotMotor.setVoltage(pitchProfiledPIDController.calculate(pivotMotor.getEncoder().getVelocity(), velocitySetpoint));
//        pivotMotor.set(velocitySetpoint);
//        // pivotPIDController.setReference(velocitySetpoint, CANSparkBase.ControlType.kSmartMotion);
//    }
//    public void stopPivot(){
//        pivotMotor.stopMotor();
//    }
//    public void zeroPivot() {
//        pivotPIDController.setReference(0, CANSparkBase.ControlType.kPosition);
//    }
    private double getAngleBetweenTwoVecDeg(Vector<N3> vec1, Vector<N3> vec2){
        return Math.acos(vec1.dot(vec2)/(vec1.norm()*vec2.norm()))*(180/Math.PI);
    }
//    public AutoAimData autoAim(List<Integer> targetFiducialIds, Pose3d aimPose, double errorTolerance){
//        PhotonPipelineResult result = shooterCamera.getLatestResult();
//        if (result.hasTargets()) {
//            for (PhotonTrackedTarget target: result.getTargets()){
//                int fiducialId = target.getFiducialId();
//                if (targetFiducialIds.contains(fiducialId)){
//                    Transform3d targetToCamera = target.getBestCameraToTarget().inverse();
//                    Pose3d targetPose = VisionConstants.aprilTagFieldLayout.getTagPose(fiducialId).get();
//                    //Pose3d aimPose = VisionConstants.getSpeakerTargetPoseForAlliance();
//                    Pose3d cameraPose = targetPose.transformBy(targetToCamera);
//
//                    Vector<N3> targetVec = targetPose.getTranslation().toVector();
//                    Vector<N3> aimVec = aimPose.getTranslation().toVector();
//                    Vector<N3> cameraVec = cameraPose.getTranslation().toVector();
//
//                    Vector<N3> targetPoseRelativeToCameraPose = targetVec.minus(cameraVec);
//                    Vector<N3> aimPoseRelativeToCameraPose = aimVec.minus(cameraVec);
//
//                    Vector<N3> aimPoseRelativeToCameraPose_SameZ = new Vector<>(aimPoseRelativeToCameraPose.copy());
//                    aimPoseRelativeToCameraPose_SameZ.set(2, 0, targetPoseRelativeToCameraPose.get(2, 0));
//
//                    Vector<N3> aimPoseRelativeToCameraPose_SameXY = new Vector<>(aimPoseRelativeToCameraPose.copy());
//                    aimPoseRelativeToCameraPose_SameXY.set(0, 0, targetPoseRelativeToCameraPose.get(0, 0));
//                    aimPoseRelativeToCameraPose_SameXY.set(1, 0, targetPoseRelativeToCameraPose.get(1, 0));
//
//                    double aimPitchAngle = getAngleBetweenTwoVecDeg(aimPoseRelativeToCameraPose_SameXY, targetPoseRelativeToCameraPose);
//                    double aimYawAngle = getAngleBetweenTwoVecDeg(aimPoseRelativeToCameraPose_SameZ, targetPoseRelativeToCameraPose);
//                    //System.out.println(aimAngel);
//                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turnProfiledPIDController.calculate(target.getYaw(), -aimYawAngle));
//                    return new AutoAimData(chassisSpeeds, pitchProfiledPIDController.calculate(target.getPitch(), -aimPitchAngle),closeEnoughAimToShootTarget(target, new Pair<Double,Double>(-aimYawAngle, -aimPitchAngle), errorTolerance));
//                }
//            }
//        }
//        return new AutoAimData(new ChassisSpeeds(0,0,0), 0.0, false);
//    }
public StaticAutoAimData staticAutoAim(List<Integer> targetFiducialIds, Pose3d aimPose, Pose2d robotPose, double errorTolerance){
    PhotonPipelineResult result = shooterCamera.getLatestResult();
    if (result.hasTargets()) {
        for (PhotonTrackedTarget target: result.getTargets()){
            int fiducialId = target.getFiducialId();
            if (targetFiducialIds.contains(fiducialId)){
                Transform3d targetToCamera = target.getBestCameraToTarget().inverse();
                Pose3d targetPose = Constants.VisionConstants.aprilTagFieldLayout.getTagPose(fiducialId).get();
                //Pose3d aimPose = VisionConstants.getSpeakerTargetPoseForAlliance();
                Pose3d cameraPose = targetPose.transformBy(targetToCamera);

                Vector<N3> targetVec = targetPose.getTranslation().toVector();
                Vector<N3> aimVec = aimPose.getTranslation().toVector();
                Vector<N3> cameraVec = cameraPose.getTranslation().toVector();
                Vector<N2> robotVec = robotPose.getTranslation().toVector();

                Vector<N3> targetPoseRelativeToCameraPose = targetVec.minus(cameraVec);
                Vector<N3> aimPoseRelativeToCameraPose = aimVec.minus(cameraVec);

                Vector<N3> aimPoseRelativeToCameraPose_SameZ = new Vector<>(aimPoseRelativeToCameraPose.copy());
                aimPoseRelativeToCameraPose_SameZ.set(2, 0, targetPoseRelativeToCameraPose.get(2, 0));

                Vector<N3> aimPoseRelativeToCameraPose_SameXY = new Vector<>(aimPoseRelativeToCameraPose.copy());
                aimPoseRelativeToCameraPose_SameXY.set(0, 0, targetPoseRelativeToCameraPose.get(0, 0));
                aimPoseRelativeToCameraPose_SameXY.set(1, 0, targetPoseRelativeToCameraPose.get(1, 0));

                double aimPitchAngle = getAngleBetweenTwoVecDeg(aimPoseRelativeToCameraPose_SameXY, targetPoseRelativeToCameraPose);
                double aimYawAngle = getAngleBetweenTwoVecDeg(aimPoseRelativeToCameraPose_SameZ, targetPoseRelativeToCameraPose);

                Vector<N2> traversalVector = new Vector<N2>(Nat.N2());
                traversalVector.set(0,0,targetPoseRelativeToCameraPose.get(0,0));
                traversalVector.set(1,0,targetPoseRelativeToCameraPose.get(1,0));
                traversalVector = traversalVector.unit().times(pitchProfiledPIDController.calculate(target.getPitch(), -aimPitchAngle));

                Vector<N2> robotForwardVec = robotPose.transformBy(new Transform2d(1, 0, new Rotation2d())).getTranslation().toVector().minus(robotVec);
                Vector<N2> robotLateralVec = robotPose.transformBy(new Transform2d(0, 1, new Rotation2d())).getTranslation().toVector().minus(robotVec);

                Vector<N2> traversalForwardComponent = robotForwardVec.times(traversalVector.dot(robotForwardVec));
                Vector<N2> traversalLateralComponent = robotLateralVec.times(traversalVector.dot(robotLateralVec));

                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                        traversalForwardComponent.norm(),
                        traversalLateralComponent.norm(),
                        turnProfiledPIDController.calculate(target.getYaw(), -aimYawAngle)
                );
                return new StaticAutoAimData(chassisSpeeds,closeEnoughAimToShootTarget(target, new Pair<Double,Double>(-aimYawAngle, -aimPitchAngle), errorTolerance));
            }
        }
    }
    return new StaticAutoAimData();
}
    public boolean targetsDetected(List<Integer> targetFiducialIds){
        PhotonPipelineResult result = shooterCamera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                int fiducialId = target.getFiducialId();
                if (targetFiducialIds.contains(fiducialId)) {
                    return true;
                }
            }
        }
        return false;
    }
    public boolean closeEnoughAimToShootTarget(PhotonTrackedTarget target, Pair<Double,Double> targetAimCoordinates, double errorTolerance){
        return Math.sqrt(Math.pow(target.getPitch()-targetAimCoordinates.getSecond(),2) +
                         Math.pow(target.getYaw()-targetAimCoordinates.getFirst(),2)) <= errorTolerance;
    }
    public PhotonPipelineResult getLatestCameraResult(){
        return shooterCamera.getLatestResult();
    }
    public boolean noteDetected(){
        return !proximitySensor.get();
    }
    public double getShooterSpeedRps(){
        return Math.abs(topShooterMotor.getEncoder().getVelocity()*60);
    }
    // public double getPivotPosRotations(){
    //     return -pivotEncoder.getPosition();
    // }
    @Override
    public void periodic(){
        super.periodic();
        SmartDashboard.putBoolean("ProximitySensor", !proximitySensor.get());
        //SmartDashboard.putNumber("Pivot Pos", getPivotPosRotations());
    }
}


