package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frckit.util.GeomUtil;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;

    private final DifferentialDriveOdometry odometry;

    public DriveSubsystem(DriveIO io) {
        this.io = io;
        io.resetEncodersAndGyro();
        odometry = new DifferentialDriveOdometry(io.getHeading());
    }

    public double getLeftPositionMeters() {
        return io.getLeftPositionRadians() * Constants.kDriveWheelRadiusMeters;
    }

    public double getRightPositionMeters() {
        return io.getRightPositionRadians() * Constants.kDriveWheelRadiusMeters;
    }

    public double getLeftVelocityMetersPerSec() {
        return io.getLeftVelocityRadPerSec() * Constants.kDriveWheelRadiusMeters;
    }

    public double getRightVelocityMetersPerSec() {
        return io.getRightVelocityRadPerSec() * Constants.kDriveWheelRadiusMeters;
    }

    @Override
    public void periodic() {
        io.update();

        // Run odometry and report findings to RobotState
        odometry.update(io.getHeading(), getLeftPositionMeters(), getRightPositionMeters());
        ChassisSpeeds velocity = Constants.kDriveKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())
        );

        RobotState.getInstance().recordOdometryObservations(Robot.getTimestamp(), odometry.getPoseMeters(), velocity);
        SmartDashboard.putString("FieldToVehicle", GeomUtil.metersToInches(RobotState.getInstance().getLatestFieldToVehicle()).toString());
    }

    public void stop() {
        io.setVoltage(0, 0);
    }

    public void setPercentOut(double left, double right) {
        io.setVoltage(left * 12, right * 12);
    }
}
