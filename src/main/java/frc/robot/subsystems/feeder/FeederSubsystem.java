package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private FeederIO io;
    
    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    public void runFunnel(double leftPower, double rightPower) {
        io.setLeftSide(leftPower);
        io.setRightSide(rightPower);
    }

    public void runFeeder(double feederPower) {
        io.setFeeder(feederPower);
    }

    public void runKicker(double kickerPower) {
        io.setKicker(kickerPower);
    }

    public boolean seesBall() {
        return io.getBeamBreak();
    }

    public void stop() {
        runFunnel(0.0, 0.0);
        runFeeder(0.0);
        runKicker(0.0);
    }
}
