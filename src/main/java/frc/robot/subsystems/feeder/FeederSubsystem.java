package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private FeederIO io;

    public FeederSubsystem(FeederIO io) {
        this.io = io;
        SmartDashboard.putBoolean("Beam Break", seesBall());
    }
    
    @Override
    public void periodic() {
    }

    public void runFunnel(double leftPower, double rightPower) {
        io.setLeftSide(leftPower);
        io.setRightSide(rightPower);
    }

    public void runFeeder(double feederPower) {
        io.setFeeder(feederPower);
    }

    public void runLower(double power) {
        runFunnel(power, power);
        runFeeder(power);
    }

    public void runAll(double power) {
        runLower(power);
    }

    public boolean seesBall() {
        return io.getBeamBreak();
    }

    public void stop() {
        runAll(0.0);
    }
}
