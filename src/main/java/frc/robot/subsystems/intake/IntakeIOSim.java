package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    private double intakePower;
    private boolean intakePosition;

    public IntakeIOSim() {
        intakePower = 0.0;
        intakePosition = Constants.kIntakeRetracted;
        SmartDashboard.putNumber("Intake Power", intakePower);
        SmartDashboard.putString("Intake Position", intakePosition ? "Extended" : "Retracted");
    }

    @Override
    public void setPower(double inPower) {
        intakePower = inPower;
        SmartDashboard.putNumber("Intake Power", intakePower);
    }

    @Override
    public void setPosition(boolean desiredPosition) {
        intakePosition = desiredPosition;
        SmartDashboard.putString("Intake Position", intakePosition ? "Extended" : "Retracted");
    }

    @Override
    public boolean currentPosition() {
        return intakePosition;
    }
}
