package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;

public class DriveIOReal implements DriveIO {
    private final CANSparkMax leftMain = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftFollow = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rightMain = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightFollow = new CANSparkMax(4, MotorType.kBrushless);
    private final CANPIDController leftPID = leftMain.getPIDController();
    private final CANPIDController rightPID = rightMain.getPIDController();
    private final PigeonIMU imu;

    private final CANEncoder leftEnc = leftMain.getEncoder();
    private final CANEncoder rightEnc = rightMain.getEncoder();

    public DriveIOReal(TalonSRX pigeonTalon) {
        leftFollow.follow(leftMain);
        rightFollow.follow(rightMain);

        leftMain.setInverted(true);
        imu = new PigeonIMU(pigeonTalon);
        leftPID.setP(Constants.kDriveKp);
        rightPID.setP(Constants.kDriveKp);
    }

    @Override
    public void update() {}

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFF, double rightFF) {
        double leftRPM = leftRadPerSec * Constants.kDriveGearRatio * 60.0 / (2.0 * Math.PI);
        double rightRPM = rightRadPerSec * Constants.kDriveGearRatio * 60.0 / (2.0 * Math.PI);

        leftPID.setReference(leftRPM, ControlType.kVelocity, 0, leftFF, CANPIDController.ArbFFUnits.kVoltage);
        rightPID.setReference(rightRPM, ControlType.kVelocity, 0, rightFF, CANPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftMain.setVoltage(leftVolts);
        rightMain.setVoltage(rightVolts);
    }

    @Override
    public void setGains(double Kp, double Kd) {
        leftPID.setP(Kp);
        leftPID.setD(Kd);
        rightPID.setP(Kp);
        rightPID.setD(Kd);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake) {
            leftMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
            rightMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
            leftFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
            rightFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } else {
            leftMain.setIdleMode(CANSparkMax.IdleMode.kCoast);
            rightMain.setIdleMode(CANSparkMax.IdleMode.kCoast);
            leftFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
            rightFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
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
    public double getHeadingRadians() {
        return Math.toRadians(imu.getFusedHeading());
    }

    @Override
    public void resetEncodersAndGyro() {
        imu.setFusedHeading(0.0);
        leftEnc.setPosition(0.0);
        rightEnc.setPosition(0.0);
    }
    
}
