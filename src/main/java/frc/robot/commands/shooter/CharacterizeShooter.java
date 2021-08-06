package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class CharacterizeShooter extends CommandBase {
    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    double priorAutospeed = 0;
    double[] numberArray = new double[6];

    ShooterSubsystem shooter;

    public CharacterizeShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
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

        double position = shooter.getPositionRadians();
        double rate = shooter.getVelocityRadPerSec();

        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        // command motors to do things
        shooter.setVoltage(autospeed * 12);

        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = motorVolts;
        numberArray[4] = position;
        numberArray[5] = rate;

        telemetryEntry.setDoubleArray(numberArray);

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(0);
    }
}
