package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frckit.vision.DualCornerVisionKinematics;
import frckit.vision.corner.DualTopCornerSupplier;
import frckit.vision.corner.LimelightCornerSupplier;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final DualCornerVisionKinematics visionKinematics = new DualCornerVisionKinematics(
            new DualTopCornerSupplier(new LimelightCornerSupplier("limelight")),
            Constants.kVisionHorizontalPlaneToLens, Constants.kVisionLensHeightM, Constants.kVisionGoalHeightM
    );

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tvEntry = table.getEntry("tv");
    private final NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
    private final NetworkTableEntry pipelineEntry = table.getEntry("pipeline");

    private int pipeline = 1; // 1 is the default vision pipeline
    private int ledMode = 1; // Start with LED off

    public Optional<Translation2d> getCameraToTarget() {
        if (seesTarget()) {
            return visionKinematics.forwardKinematics();
        } else {
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {
        pipelineEntry.setDouble(pipeline);
        ledModeEntry.setDouble(ledMode);
    }

    /**
     * Returns true if the camera sees the target.  This will always return false when the lights are off.
     * @return True if the camera can see the target, false otherwise
     */
    public boolean seesTarget() {
        return (ledMode == 3 || ledMode == 0) && tvEntry.getDouble(0.0) == 1.0;
    }

    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
    }

    public void ledOff() {
        ledMode = 1;
    }

    public void ledOn() {
        ledMode = 3;
    }

    public void ledBlink() {
        ledMode = 2;
    }
}
