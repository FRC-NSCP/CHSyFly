package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.shooter.HomeHood;
import org.team401.robot2020.control.VelocityProfiler;
import org.team401.util.PIDControllerLite;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;

    private final VelocityProfiler profiler = VelocityProfiler.createFromJava(Constants.KShooterAccel);
    private final SimpleMotorFeedforward model = new SimpleMotorFeedforward(
            Constants.kShooterKs,
            Constants.kShooterKv,
            Constants.kShooterKa
    );

    private boolean hoodHomed = false;

    public boolean isHoodHomed() {
        return hoodHomed;
    }

    public void setHoodHomed(boolean hoodHomed) {
        this.hoodHomed = hoodHomed;
    }

    private final PIDControllerLite hoodController = new PIDControllerLite(Constants.kHoodKp, 0.0, Constants.kHoodKd);

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    private final HomeHood homeHood = new HomeHood(this);

    @Override
    public void periodic() {
        if (!hoodHomed) {
            // If the turret isn't homed, ensure a non-interruptible home command is running
            // until it is.  Do not do anything else until this is done
            Command currentCommand = getCurrentCommand();
            if (currentCommand != null && currentCommand != homeHood) {
                currentCommand.cancel();
            }
            homeHood.schedule(false);
            return;
        }
    }

    public double getPositionRadians() {
        return io.getPositionRadians();
    }

    public double getVelocityRadPerSec() {
        return io.getVelocityRadPerSec();
    }

    public void setFlywheelVoltage(double volts) {
        io.setShooterVoltage(volts);
    }

    public void resetProfiler() {
        profiler.resetFromJava(getVelocityRadPerSec());
    }

    public void runFlywheel() {
        io.setKickerPower(Constants.kKickerPercent);
        profiler.calculateFromJava(Constants.kDt, Constants.kShooterSpeed);
        double ffVolts = model.calculate(profiler.getVelocityCommandRadPerSec(), profiler.getAccelCommandRadPerSecPerSec());
        io.setVelocity(profiler.getVelocityCommandRadPerSec(), ffVolts);
    }

    public void stop() {
        io.setHoodVoltage(0);
        io.setKickerPower(0);
        io.setShooterVoltage(0);
    }

    public void updateDejam() {
        profiler.calculateFromJava(Constants.kDt, Constants.kShooterDejamSpeed);
        double ffVolts = model.calculate(profiler.getVelocityCommandRadPerSec(), profiler.getAccelCommandRadPerSecPerSec());
        io.setVelocity(profiler.getVelocityCommandRadPerSec(), ffVolts);
    }

    public void resetHoodPosition() {
        io.zeroHood();
    }

    public void setHoodVoltage(double volts) {
        io.setHoodVoltage(volts);
    }

    public double getHoodVelocity() {
        return io.getHoodVelocity();
    }

    public void updateHood(double setpoint) {
        if (setpoint > Constants.kHoodMax) setpoint = Constants.kHoodMax;
        if (setpoint < Constants.kHoodMin) setpoint = Constants.kHoodMin;
        io.setHoodPosition(setpoint);
    }

    public void setHoodGains(double Kp, double Kd) {
        io.setHoodGains(Kp, Kd);
    }

    public boolean readyToShoot() {
        double percent = getVelocityRadPerSec() / Constants.kShooterSpeed;
        return (percent > (1 - Constants.kShooterReadySpeedPercent) && percent < (1 + Constants.kShooterReadySpeedPercent));
    }
}
