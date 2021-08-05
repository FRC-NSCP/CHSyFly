package frc.robot.subsystems.feeder;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class FeederIOReal implements FeederIO {
    private final CANSparkMax kicker = new CANSparkMax(6, MotorType.kBrushless);
    private final TalonSRX feeder = new TalonSRX(15);
    private final TalonSRX hopperLeft = new TalonSRX(11);
    private final TalonSRX hopperRight  = new TalonSRX(12);
    private final DigitalInput beamBreak = new DigitalInput(0);


    public FeederIOReal() {
        kicker.setSmartCurrentLimit(20);
        hopperLeft.setInverted(true);
        hopperRight.setInverted(true);
    }

    /**
     * The Pigeon is attached to the feeder motor so we need a way to access it
     * @return Feeder motor object
     */
    public TalonSRX getPigeonTalon() {
        return feeder;
    }

    @Override
    public void setLeftSide(double power) {
        hopperLeft.set(TalonSRXControlMode.PercentOutput, power);
    }

    @Override
    public void setRightSide(double power) {
        hopperRight.set(TalonSRXControlMode.PercentOutput, power);
    }

    @Override
    public void setFeeder(double power) {
        feeder.set(TalonSRXControlMode.PercentOutput, power);
    }

    @Override
    public void setKicker(double power) {
        kicker.set(power);
    }

    @Override
    public boolean getBeamBreak() {
        return !beamBreak.get();
    }
}
