package frc.robot.commands.drive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSubsystem;

public class CharacterizeDrive extends CommandBase {
    private final DriveSubsystem drive;

    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    double priorAutospeed = 0;
    double[] numberArray = new double[10];

    public CharacterizeDrive(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        priorAutospeed = 0.0;
        NetworkTableInstance.getDefault().setUpdateRate(0.01);
    }

    @Override
    public void execute() {

        // Retrieve values to send back before telling the motors to do something
        double now = Robot.getTimestamp();

        double leftPosition = drive.getLeftPositionMeters();
        double leftRate = drive.getLeftVelocityMetersPerSec();

        double rightPosition = drive.getRightPositionMeters();
        double rightRate = drive.getRightVelocityMetersPerSec();

        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);

        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        // command motors to do things
        drive.setPercentOut((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);


        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = drive.getAbsHeading();

        telemetryEntry.setDoubleArray(numberArray);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
