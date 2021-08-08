package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final RamseteController follower = new RamseteController();
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;
    private final boolean forcePose;

    public FollowTrajectoryCommand(DriveSubsystem drive, Trajectory trajectory, boolean forcePose) {
        this.drive = drive;
        this.trajectory = trajectory;
        this.forcePose = forcePose;

        addRequirements(drive);

        //SmartDashboard.putNumber("DriveSparkKp", 0);
        //SmartDashboard.putNumber("DriveSparkKd", 0);
    }

    @Override
    public void initialize() {
        //drive.setGains(SmartDashboard.getNumber("DriveSparkKp", 0), SmartDashboard.getNumber("DriveSparkKd", 0));
        System.out.println("STARTING TRAJECTORY COMMAND");
        prevTime = -1;
        var initialState = trajectory.sample(0);
        if (forcePose) {
            RobotState.getInstance().forceRobotPose(initialState.poseMeters);
        }
        prevSpeeds =
                Constants.kDriveKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        timer.reset();
        timer.start();
        drive.setBrake();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        if (prevTime < 0) {
            drive.stop();
            prevTime = curTime;
            return;
        }

        var targetWheelSpeeds =
                Constants.kDriveKinematics.toWheelSpeeds(
                        follower.calculate(RobotState.getInstance().getLatestFieldToVehicle(), trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFFVolts = Constants.kDriveModel.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);
        double rightFFVolts = Constants.kDriveModel.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

        drive.setVelocity(leftSpeedSetpoint, rightSpeedSetpoint, leftFFVolts, rightFFVolts);

        prevSpeeds = targetWheelSpeeds;
        prevTime = curTime;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
