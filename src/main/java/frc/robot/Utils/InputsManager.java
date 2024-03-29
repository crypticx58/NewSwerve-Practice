package frc.robot.Utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class InputsManager {
    public static final class SwerveInputsManager {
        private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
        private final Supplier<Double> xSpeedFunc, ySpeedFunc, turnSpeedFunc;
        private final Supplier<Boolean> fieldOrientatedFunc;
        private final SlewRateLimiter xRateLimiter, yRateLimiter;
        private final double translationSpeedFactor;
        private final double rotationSpeedFactor;
        public SwerveInputsManager(Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc,
                                   Supplier<Double> turnSpeedFunc, Supplier<Boolean> fieldOrientatedFunc,
                                   double translationSpeedFactor, double rotationSpeedFactor){
            xRateLimiter = new SlewRateLimiter(0.85);
            yRateLimiter = new SlewRateLimiter(0.85);

            this.xSpeedFunc = xSpeedFunc;
            this.ySpeedFunc = ySpeedFunc;
            this.turnSpeedFunc = turnSpeedFunc;
            this.fieldOrientatedFunc = fieldOrientatedFunc;
            this.translationSpeedFactor = translationSpeedFactor;
            this.rotationSpeedFactor = rotationSpeedFactor;
        }
        public ChassisSpeeds getChassisSpeeds(){
            double xSpeed = xSpeedFunc.get() * translationSpeedFactor;
            double ySpeed = ySpeedFunc.get() * translationSpeedFactor;
            double turnSpeed = turnSpeedFunc.get() * rotationSpeedFactor;

            xSpeed = Math.abs(xSpeed)<Constants.IOConstants.kDeadband?0:xSpeed;
            ySpeed = Math.abs(ySpeed)<Constants.IOConstants.kDeadband?0:ySpeed;
            turnSpeed = Math.abs(turnSpeed)<Constants.IOConstants.kDeadband?0:turnSpeed;

            xSpeed = xRateLimiter.calculate(xSpeed) * Constants.OperatorConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yRateLimiter.calculate(ySpeed) * Constants.OperatorConstants.kTeleDriveMaxSpeedMetersPerSecond;
            //turnSpeed = turnRateLimiter.calculate(turnSpeed) * Constants.OperatorConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            ChassisSpeeds chassisSpeeds;
            if (fieldOrientatedFunc.get()){
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getHeadingRotation2d());
            } else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            }
            return  chassisSpeeds;
        }

        public SwerveModuleState[] getSwerveModuleStates(){
            ChassisSpeeds chassisSpeeds = getChassisSpeeds();
            return Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        }
        public static void driveRobotRelative(ChassisSpeeds chassisSpeeds){
            SwerveSubsystem.getInstance().setModuleStates(Constants.PhysicalConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        }
    }
    public static final class MechanismsInputsManager{
        private final Supplier<Double> intakeSpeedFunc,pivotSpeedFunc,shooterSpeedFunc,interfaceSpeedFunc;
        public MechanismsInputsManager(Supplier<Double> intakeSpeedFunc, Supplier<Double> pivotSpeedFunc,
                                       Supplier<Double> shooterSpeedFunc, Supplier<Double> interfaceSpeedFunc) {
            this.intakeSpeedFunc = intakeSpeedFunc;
            this.pivotSpeedFunc = pivotSpeedFunc;
            this.shooterSpeedFunc = shooterSpeedFunc;
            this.interfaceSpeedFunc = interfaceSpeedFunc;
        }

        public double getIntakeSpeedFunc() {
            return intakeSpeedFunc.get();
        }

        public double getPivotSpeedFunc() {
            return pivotSpeedFunc.get();
        }

        public double getShooterSpeedFunc() {
            return shooterSpeedFunc.get();
        }

        public double getInterfaceSpeedFunc() {
            return interfaceSpeedFunc.get();
        }
    }
    public static final class VisionTargetingInputsManager{
        private final Supplier<Boolean> speakerAim,ampAim,feedAim;
        public VisionTargetingInputsManager(Supplier<Boolean> speakerAim, Supplier<Boolean> ampAim, Supplier<Boolean> feedAim) {
            this.speakerAim = speakerAim;
            this.ampAim = ampAim;
            this.feedAim = feedAim;
        }
        public boolean getSpeakerAim() {
            return speakerAim.get();
        }

        public boolean getAmpAim() {
            return ampAim.get();
        }

        public boolean getFeedAim() {
            return feedAim.get();
        }
    }
}
