package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.util.PoseHistory;
import frckit.physics.state.RigidBodyState2d;
import frckit.util.GeomUtil;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private Pose2d fieldToBoot = GeomUtil.IDENTITY_POSE;
    private final PoseHistory bootToVehicle = new PoseHistory(100);
    private final PoseHistory vehicleToTurret = new PoseHistory(100);
    private ChassisSpeeds vehicleVelocity = new ChassisSpeeds();
    private double turretVelocityRadPerSec = 0.0;

    private Pose2d latestMeasuredFieldToTarget = GeomUtil.IDENTITY_POSE;

    private RobotState() {
        bootToVehicle.insert(0.0, GeomUtil.IDENTITY_POSE);
        vehicleToTurret.insert(0.0, GeomUtil.IDENTITY_POSE);
    }

    public void forceRobotPose(Pose2d fieldToVehicle) {
        Pose2d bootToVehicle = this.bootToVehicle.getLatest().orElseThrow().getPose();
        Pose2d vehicleToBoot = GeomUtil.poseInverse(bootToVehicle);
        fieldToBoot = fieldToVehicle.transformBy(GeomUtil.poseToTransform(vehicleToBoot));
    }

    public void recordOdometryObservations(double timestamp, Pose2d bootToVehicle, ChassisSpeeds velocity) {
        this.bootToVehicle.insert(timestamp, bootToVehicle);
        vehicleVelocity = velocity;
    }

    public Pose2d getFieldToVehicle(double timestamp) {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.get(timestamp).orElseThrow());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getLatestFieldToVehicle() {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.getLatest().orElseThrow().getPose());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public void recordTurretObservations(double timestamp, Rotation2d turretOriginToTurretRotation, double turretVelocityRadPerSec) {
        Pose2d newVehicleToTurret = Constants.kVehicleToTurret.transformBy(GeomUtil.transformFromRotation(turretOriginToTurretRotation));
        this.turretVelocityRadPerSec = turretVelocityRadPerSec;
        vehicleToTurret.insert(timestamp, newVehicleToTurret);
    }
}
