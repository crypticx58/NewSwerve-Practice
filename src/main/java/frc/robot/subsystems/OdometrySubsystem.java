package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.InputsManager;

public class OdometrySubsystem extends SubsystemBase {
    //private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private static OdometrySubsystem INSTANCE;
    @SuppressWarnings("WeakerAccess")
    public static OdometrySubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new OdometrySubsystem();
        }
        return INSTANCE;
    }
    private final Field2d fieldVisualizer2d = new Field2d();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.PhysicalConstants.kDriveKinematics, new Rotation2d(0), swerveSubsystem.getModulePositions(), new Pose2d());
    private double previousShooterCameraResultTimeStamp;
    
    public void resetOdometry(Pose2d newPose){
        swerveDrivePoseEstimator.resetPosition(swerveSubsystem.getHeadingRotation2d(), swerveSubsystem.getModulePositions(), newPose);
    }
    public Pose2d getPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }
//    public Pose2d getVisionMeasurementFromShooter(){
//        PhotonPipelineResult result = shooterSubsystem.getLatestCameraResult();
//
//        double resultTimeStamp = result.getTimestampSeconds();
//        if (resultTimeStamp != previousShooterCameraResultTimeStamp && result.hasTargets()) {
//            for (PhotonTrackedTarget target: result.getTargets()){
//                int fiducialId = target.getFiducialId();
//                if (target.getPoseAmbiguity() <= 0.2 && fiducialId == VisionConstants.BlueAllienceTargetIds.CenterSpeaker.fiducialID){
//
//                }
//            }
//        }
//    }
    private OdometrySubsystem() {
        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, swerveSubsystem::getRobotRelativeSpeeds, InputsManager.SwerveInputsManager::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.75),
                        new PIDConstants(2.75),
                        4.5,
                        Units.inchesToMeters(20),
                        new ReplanningConfig()),
                () -> false,
                this);
        SmartDashboard.putData("Field", fieldVisualizer2d);
    }
    private static double getAngleBetweenTwoVecDeg(Vector<N2> vec1, Vector<N2> vec2){
        return Math.acos(vec1.dot(vec2)/(vec1.norm()*vec2.norm()))*(180/Math.PI);
    }
    public static Rotation2d aimChassisTowardsPoint(Vector<N2> origin, Vector<N2> forward, Vector<N2> lateral, Vector<N2> target){
        Vector<N2> forwardRelativeToOrigin = forward.minus(origin);
        Vector<N2> targetRelativeToOrigin = target.minus(origin);
        Vector<N2> lateralRelativeToOrigin = lateral.minus(origin);
        boolean isRotationPositive = lateralRelativeToOrigin.dot(targetRelativeToOrigin) >= 0;
        double aimYawAngle = getAngleBetweenTwoVecDeg(targetRelativeToOrigin, forwardRelativeToOrigin);
        return Rotation2d.fromDegrees(aimYawAngle*(isRotationPositive?1:-1));
    }
    @Override
    public void periodic() {
        super.periodic();
        swerveDrivePoseEstimator.update(swerveSubsystem.getHeadingRotation2d(), swerveSubsystem.getModulePositions());
        fieldVisualizer2d.setRobotPose(getPose());
    }

}

