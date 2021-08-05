package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class TurretIOReal implements TurretIO {
    private final Encoder encoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
    private final double encoderScale;
    private final CANSparkMax spark = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

    public TurretIOReal() {
        encoderScale = encoder.getEncodingScale();
        spark.setInverted(true);
    }

    @Override
    public void zeroEncoder() {
        encoder.reset();
    }

    @Override
    public double getPositionRadians() {
        return encoder.getRaw() / (2048.0 * encoderScale) * 2.0 * Math.PI / Constants.kTurretGearRatio;
    }

    @Override
    public double getVelocityRadiansPerSec() {
        return encoder.getRate() / 2048.0 * 2.0 * Math.PI / Constants.kTurretGearRatio;
    }

    @Override
    public void setVoltage(double volts) {
        spark.setVoltage(volts);
    }
}
