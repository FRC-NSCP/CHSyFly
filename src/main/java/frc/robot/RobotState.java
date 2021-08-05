package frc.robot;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionTargetingParameters;
import frc.util.PoseHistory;
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
    private VisionTargetingParameters targetingParameters = new VisionTargetingParameters(0, GeomUtil.IDENTITY_ROTATION, 0, 0);

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

    public Pose2d getPredictedFieldToVehicle(double lookaheadTime) {
        return getLatestFieldToVehicle().exp(
                new Twist2d(vehicleVelocity.vxMetersPerSecond * lookaheadTime, vehicleVelocity.vyMetersPerSecond * lookaheadTime, vehicleVelocity.omegaRadiansPerSecond * lookaheadTime)
        );
    }

    public void recordTurretObservations(double timestamp, Rotation2d turretOriginToTurretRotation, double turretVelocityRadPerSec) {
        Pose2d newVehicleToTurret = Constants.kVehicleToTurret.transformBy(GeomUtil.transformFromRotation(turretOriginToTurretRotation));
        this.turretVelocityRadPerSec = turretVelocityRadPerSec;
        vehicleToTurret.insert(timestamp, newVehicleToTurret);
    }

    public Pose2d getVehicleToTurret(double timestamp) {
        return vehicleToTurret.get(timestamp).orElseThrow();
    }

    public Pose2d getLatestVehicleToTurret() {
        return vehicleToTurret.getLatest().orElseThrow().getPose();
    }

    public Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(
                GeomUtil.poseToTransform(getVehicleToTurret(timestamp))
        );
    }

    public Pose2d getLatestFieldToTurret() {
        return getLatestFieldToVehicle().transformBy(
                GeomUtil.poseToTransform(getLatestVehicleToTurret())
        );
    }

    public Pose2d getPredictedVehicleToTurret(double lookaheadTime) {
        return getLatestVehicleToTurret().exp(
                new Twist2d(0, 0, turretVelocityRadPerSec * lookaheadTime)
        );
    }

    public Pose2d getPredictedFieldToTurret(double lookaheadTime) {
        return getPredictedFieldToVehicle(lookaheadTime).transformBy(
                GeomUtil.poseToTransform(getPredictedVehicleToTurret(lookaheadTime))
        );
    }

    public Pose2d getFieldToVisionTarget() {
        return latestMeasuredFieldToTarget;
    }

    public Pose2d getVehicleToVisionTarget(double timestamp) {
        return GeomUtil.poseInverse(getFieldToVehicle(timestamp)).transformBy(
                GeomUtil.poseToTransform(getFieldToVisionTarget())
        );
    }

    public void recordVisionObservations(double timestamp, Translation2d cameraToTarget) {
        Pose2d fieldToTarget = getFieldToTurret(timestamp)
                .transformBy(GeomUtil.poseToTransform(Constants.kTurretToCamera))
                .transformBy(GeomUtil.transformFromTranslation(cameraToTarget));

        latestMeasuredFieldToTarget = new Pose2d(fieldToTarget.getTranslation(), GeomUtil.IDENTITY_ROTATION);
    }

    public VisionTargetingParameters getTargetingParameters(double timestamp) {
        if (targetingParameters.getTimestamp() == timestamp) return targetingParameters;

        Pose2d vehicleToGoal = GeomUtil.poseInverse(getFieldToVehicle(timestamp))
                .transformBy(GeomUtil.poseToTransform(latestMeasuredFieldToTarget));

        Pose2d turretToGoal = GeomUtil.poseInverse(getVehicleToTurret(timestamp))
                .transformBy(GeomUtil.poseToTransform(getFieldToVehicle(timestamp)).inverse())
                .transformBy(GeomUtil.poseToTransform(latestMeasuredFieldToTarget));

        Rotation2d vehicleToGoalDirection = new Rotation2d(vehicleToGoal.getTranslation().getX(), vehicleToGoal.getTranslation().getY());
        Translation2d turretToGoalTranslation = vehicleToGoal.transformBy(GeomUtil.poseToTransform(Constants.kVehicleToTurret)).getTranslation();
        Rotation2d turretToGoalDirection = new Rotation2d(turretToGoalTranslation.getX(), turretToGoalTranslation.getY());

        Pose2d vehicleToTurretNow = getVehicleToTurret(timestamp);

        Pose2d fieldToTurret = getFieldToTurret(timestamp);
        Pose2d fieldToPredictedTurret = getPredictedFieldToTurret(Constants.kTurretLookaheadSeconds);

        Pose2d turretToPredictedTurret = GeomUtil.poseInverse(fieldToTurret)
                .transformBy(GeomUtil.poseToTransform(fieldToPredictedTurret));

        Pose2d predictedTurretToGoal = GeomUtil.poseInverse(turretToPredictedTurret)
                .transformBy(GeomUtil.poseToTransform(turretToGoal));

        double correctedRangeToTarget = predictedTurretToGoal.getTranslation().getNorm();

        Rotation2d turretError = vehicleToTurretNow.getRotation()
                .unaryMinus()
                .rotateBy(vehicleToGoalDirection);

        double vehicleRange = vehicleToGoal.getTranslation().getNorm();
        double feedVelocity = (-1.0 *
                ((vehicleToGoalDirection.getSin() * vehicleVelocity.vxMetersPerSecond / vehicleRange)
                + vehicleVelocity.omegaRadiansPerSecond));

        targetingParameters = new VisionTargetingParameters(timestamp, turretError, feedVelocity, correctedRangeToTarget);
        return targetingParameters;
    }
}
