package frc.robot.Utils;

public class MechanismsState {
    public double intakeSpeed;
    public double pivotSpeed;
    public double shooterSpeed;
    public double interfaceSpeed;
    public MechanismsState(){
        this(0,0,0,0);
    }
    public MechanismsState(double intakeSpeed, double pivotSpeed, double shooterSpeed, double interfaceSpeed) {
        this.intakeSpeed = intakeSpeed;
        this.pivotSpeed = pivotSpeed;
        this.shooterSpeed = shooterSpeed;
        this.interfaceSpeed = interfaceSpeed;
    }
}
