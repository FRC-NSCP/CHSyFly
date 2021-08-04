package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeIOReal implements IntakeIO {
    private final Solenoid intakePiston = new Solenoid(0);
    private final TalonSRX intakeMotor = new TalonSRX(8);
    
    @Override
    public void setPower(double intakePower) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, intakePower);
    }

    @Override
    public void setPosition(boolean desiredPosition) {
        intakePiston.set(desiredPosition);
    }

    @Override
    public boolean currentPosition() {
        return intakePiston.get();
    }
}
