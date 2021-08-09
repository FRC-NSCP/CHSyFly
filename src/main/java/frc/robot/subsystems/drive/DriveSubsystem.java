package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.commands.drive.FollowTrajectoryCommand;
import frckit.util.GeomUtil;

import java.util.List;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DifferentialDriveOdometry odometry;
    private Rotation2d heading;

    public DriveSubsystem(DriveIO io) {
        this.io = io;
        io.resetEncodersAndGyro();
        heading = new Rotation2d(io.getHeadingRadians());
        odometry = new DifferentialDriveOdometry(heading);
        SmartDashboard.putNumber("DriveLeftVel", 0);
        SmartDashboard.putNumber("DriveRightVel", 0);
        SmartDashboard.putNumber("RamLeftSet", 0);
        SmartDashboard.putNumber("RamRightSet", 0);

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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    public double getAbsHeading() {
        return io.getHeadingRadians();
    }

    public Rotation2d getHeading() {
        return heading;
    }

    @Override
    public void periodic() {
        io.update();
        heading = new Rotation2d(io.getHeadingRadians());

        // Run odometry and report findings to RobotState
        odometry.update(heading, getLeftPositionMeters(), getRightPositionMeters());
        ChassisSpeeds velocity = Constants.kDriveKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())
        );

        RobotState.getInstance().recordOdometryObservations(Robot.getTimestamp(), odometry.getPoseMeters(), velocity);

        /*
        if (io.hasResetOccurred()) {
            io.configControllers();
            System.out.println("RESETTING DRIVE CONTROLLERS");
        }
         */

    }

    public void stop() {
        io.setVoltage(0, 0);
    }

    public void setPercentOut(double left, double right) {
        io.setVoltage(left * 12, right * 12);
    }
    public void setVoltage(double left, double right) {
        io.setVoltage(left, right);
    }

    public void reset() {
        io.configControllers();
    }

    public void setVelocity(double left, double right, double leftFF, double rightFF) {
        double leftRadPerSec = left / Constants.kDriveWheelRadiusMeters;
        double rightRadPerSec = right / Constants.kDriveWheelRadiusMeters;
        SmartDashboard.putNumber("RamLeftSet", left);
        SmartDashboard.putNumber("RamRightSet", right);
        io.setVelocity(leftRadPerSec, rightRadPerSec, leftFF, rightFF);
    }

    public void setGains(double Kp, double Kd) {
        io.setGains(Kp, Kd);
    }

    public void setBrake() {
        io.setBrake(true);
    }

    public void setCoast() {
        io.setBrake(false);
    }
}
