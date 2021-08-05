package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frckit.util.GeomUtil;

import java.util.Optional;

public class RunVisionTracking extends CommandBase {
    private final VisionSubsystem vision;

    public RunVisionTracking(VisionSubsystem vision) {
        this.vision = vision;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.ledOn();
        vision.setPipeline(1);
    }

    @Override
    public void execute() {
        // Run vision kinematics and feed data to RobotState
        Optional<Translation2d> cameraToTargetOptional = vision.getCameraToTarget();

        if (cameraToTargetOptional.isPresent()) {
            Translation2d cameraToTarget = cameraToTargetOptional.get();
            RobotState.getInstance().recordVisionObservations(Robot.getTimestamp(), cameraToTarget);
            SmartDashboard.putString("CameraToTarget", GeomUtil.metersToInches(cameraToTarget).toString());
        }
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        vision.ledOff();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
