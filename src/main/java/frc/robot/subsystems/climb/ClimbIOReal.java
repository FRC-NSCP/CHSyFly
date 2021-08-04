package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

public class ClimbIOReal implements ClimbIO {
    private final Solenoid leftLock = new Solenoid(1);
    private final Solenoid rightLock = new Solenoid(2);
    private final TalonSRX leftClimber = new TalonSRX(13);
    private final TalonSRX rightClimber = new TalonSRX(14);

    @Override
    public void setLeftLock(boolean lockPosition) {
        leftLock.set(lockPosition);
    }

    @Override
    public void setRightLock(boolean lockPosition) {
        rightLock.set(lockPosition);
    }

    @Override
    public void setLeftPower(double climbPower) {
        leftClimber.set(TalonSRXControlMode.PercentOutput, climbPower);
    }

    @Override
    public void setRightPower(double climbPower) {
        rightClimber.set(TalonSRXControlMode.PercentOutput, climbPower);
    }
    
}
