package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.Constants;
import frckit.util.GeomUtil;

public class DriveIOSim implements DriveIO {
    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            Constants.kDriveGearRatio,
            Constants.kRobotMOI,
            Constants.kRobotMassKg,
            Constants.kDriveWheelRadiusMeters,
            Constants.kDriveEmpTrackwidthMeters,
            null
    );

    Field2d field = new Field2d();

    public DriveIOSim() {
        // Initialize field
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void update() {
        sim.update(Constants.kDt);
        field.setRobotPose(sim.getPose());
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        sim.setInputs(leftVolts, rightVolts);
    }

    @Override
    public double getLeftPositionRadians() {
        return sim.getLeftPositionMeters() / Constants.kDriveWheelRadiusMeters;
    }

    @Override
    public double getRightPositionRadians() {
        return sim.getRightPositionMeters() / Constants.kDriveWheelRadiusMeters;
    }

    @Override
    public double getLeftVelocityRadPerSec() {
        return sim.getLeftVelocityMetersPerSecond() / Constants.kDriveWheelRadiusMeters;
    }

    @Override
    public double getRightVelocityRadPerSec() {
        return sim.getRightVelocityMetersPerSecond() / Constants.kDriveWheelRadiusMeters;
    }

    @Override
    public Rotation2d getHeading() {
        return sim.getHeading();
    }

    @Override
    public void resetEncodersAndGyro() {
        sim.setPose(GeomUtil.IDENTITY_POSE);
    }
}
