package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {}

    public void stop() {
        io.setPower(0.0);
    }

    public void extendIntake() {
        io.setPosition(Constants.kIntakeExtended);
    }

    public void retractIntake() {
        io.setPosition(Constants.kIntakeRetracted);
    }

    public void runIntake(double power) {
        io.setPower(power);
    }
    
}
