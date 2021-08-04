package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSim implements IntakeIO {
    private double intakePower = 0.0;
    private Solenoid intakePiston = new Solenoid(0);

    public IntakeIOSim() {
        SmartDashboard.putNumber("Intake Power", intakePower);
        SmartDashboard.putString("Intake Position", currentPosition() ? "Extended" : "Retracted");
    }

    @Override
    public void setPower(double inPower) {
        intakePower = inPower;
        SmartDashboard.putNumber("Intake Power", intakePower);
    }

    @Override
    public void setPosition(boolean desiredPosition) {
        intakePiston.set(desiredPosition);
        SmartDashboard.putString("Intake Position", currentPosition() ? "Extended" : "Retracted");
    }

    @Override
    public boolean currentPosition() {
        return intakePiston.get();
    }
}
