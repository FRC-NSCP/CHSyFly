package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class VisionTargetingParameters {
    private final double timestamp;
    private final Rotation2d turretError;
    private final double turretFeedVelRadPerSec;
    private final double rangeToTargetM;

    public VisionTargetingParameters(double timestamp, Rotation2d turretError, double turretFeedVelRadPerSec, double rangeToTargetM) {
        this.timestamp = timestamp;
        this.turretError = turretError;
        this.turretFeedVelRadPerSec = turretFeedVelRadPerSec;
        this.rangeToTargetM = rangeToTargetM;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Rotation2d getTurretError() {
        return turretError;
    }

    public double getTurretFeedVelRadPerSec() {
        return turretFeedVelRadPerSec;
    }

    public double getRangeToTargetM() {
        return rangeToTargetM;
    }
}
