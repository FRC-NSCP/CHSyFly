package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;

public class DriveIOReal implements DriveIO {
    private final CANSparkMax leftMain = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftFollow = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rightMain = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightFollow = new CANSparkMax(4, MotorType.kBrushless);
    private final PigeonIMU imu;

    private final CANEncoder leftEnc = leftMain.getEncoder();
    private final CANEncoder rightEnc = rightMain.getEncoder();

    public DriveIOReal(TalonSRX pigeonTalon) {
        leftFollow.follow(leftMain);
        rightFollow.follow(rightMain);
        leftMain.setInverted(true);
        imu = new PigeonIMU(pigeonTalon);
    }

    @Override
    public void update() {}

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftMain.setVoltage(leftVolts);
        rightMain.setVoltage(rightVolts);
    }

    @Override
    public double getLeftPositionRadians() {
        return leftEnc.getPosition() / Constants.kDriveGearRatio * 2 * Math.PI;
    }

    @Override
    public double getRightPositionRadians() {
       return rightEnc.getPosition() / Constants.kDriveGearRatio * 2 * Math.PI;
    }

    @Override
    public double getLeftVelocityRadPerSec() {
        return leftEnc.getVelocity() / Constants.kDriveGearRatio / 60 * 2 * Math.PI;
    }

    @Override
    public double getRightVelocityRadPerSec() {
       return rightEnc.getVelocity() / Constants.kDriveGearRatio / 60 * 2 * Math.PI;
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(imu.getFusedHeading());
    }

    @Override
    public void resetEncodersAndGyro() {
        imu.setFusedHeading(0.0);
        leftEnc.setPosition(0.0);
        rightEnc.setPosition(0.0);
    }
    
}
