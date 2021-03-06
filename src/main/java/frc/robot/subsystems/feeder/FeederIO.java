package frc.robot.subsystems.feeder;

public interface FeederIO {
    void setLeftSide(double power);
    void setRightSide(double power);
    void setFeeder(double power);
    boolean getBeamBreak();
}
