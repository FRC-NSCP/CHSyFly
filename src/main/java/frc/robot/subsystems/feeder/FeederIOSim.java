package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederIOSim implements FeederIO {
    private double leftPower, rightPower, feederPower, kickerPower;
    private SendableChooser<Boolean> seesBall;


    public FeederIOSim() {
        leftPower = rightPower = feederPower = kickerPower = 0;
        seesBall.setDefaultOption("No ball detected", false);
        seesBall.addOption("Ball detected!", true);
    }

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
        return seesBall.getSelected();
    }

    private void updateDash() {
        SmartDashboard.putString("Feeder Powers", "Left: " + leftPower + "   Right: " + rightPower + "   Feeder: " + feederPower + "   Kicker: " + kickerPower);
    }
    
}
