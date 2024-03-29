package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem INSTANCE;
    private final CANSparkMax intakeMotor;
    PhotonCamera camera;
    PIDController turnPIDController;
    PIDController drivePIDController;

    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

    private IntakeSubsystem() {
        //intakeMotor.restoreFactoryDefaults();
        intakeMotor = new CANSparkMax(10, MotorType.kBrushed);
        
        intakeMotor.setSmartCurrentLimit(35);
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        camera = new PhotonCamera("NoteDetector");
        turnPIDController = new PIDController(0.15,0,0);
        drivePIDController = new PIDController(0.25, 0, 0);
    }
    public void startIntake(double speed){
        intakeMotor.set(-speed);
    }
    public void stopIntake(){
        intakeMotor.stopMotor();
    }
    public Pair<ChassisSpeeds,PhotonTrackedTarget> getAutoChassisSpeeds(){
        PhotonPipelineResult result =  camera.getLatestResult();
        if (result.hasTargets()){
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds((drivePIDController.calculate(bestTarget.getArea(), 100)/100)*3,
                    0,
                    turnPIDController.calculate(bestTarget.getYaw()/30, 0)*1.5);
            if (bestTarget.getArea() >= 2 && Math.abs(bestTarget.getYaw()) > 5) {
                chassisSpeeds.vxMetersPerSecond = 0;
            }
            return new Pair<ChassisSpeeds, PhotonTrackedTarget>(chassisSpeeds, bestTarget);
        } else {
            return new Pair<ChassisSpeeds, PhotonTrackedTarget>(new ChassisSpeeds(0,0,0), null);
        }
    }
    // public PhotonTrackedTarget getBestTarget(){
    //      PhotonPipelineResult result =  camera.getLatestResult();
    //     if (result.hasTargets()){
    //         PhotonTrackedTarget bestTarget = result.getBestTarget();
    //         return bestTarget;
    //     } else {
    //         return null;
    //     }
    // }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

    }
}

