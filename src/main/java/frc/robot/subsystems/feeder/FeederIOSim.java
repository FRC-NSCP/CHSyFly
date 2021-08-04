package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederIOSim implements FeederIO {
    private double leftPower, rightPower, feederPower, kickerPower = 0;
    private DigitalInput beamBreak = new DigitalInput(0);

    @Override
    public void setLeftSide(double power) {
        leftPower = power;
        updateDash();
    }

    @Override
    public void setRightSide(double power) {
        rightPower = power;
        updateDash();

    }

    @Override
    public void setFeeder(double power) {
        feederPower = power;
        updateDash();

    }

    @Override
    public void setKicker(double power) {
        kickerPower = power;
        updateDash();

    }

    @Override
    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    private void updateDash() {
        SmartDashboard.putString("Feeder Powers", "Left: " + leftPower + "   Right: " + rightPower + "   Feeder: " + feederPower + "   Kicker: " + kickerPower);
    }
    
}
