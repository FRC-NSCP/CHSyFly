package org.team401.robot2020.control.turret

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Rotation2d
import frckit.util.GeomUtil
import org.team401.units.measure.Degrees
import org.team401.units.measure.Radians
import org.team401.units.measure.RadiansPerSecond
import org.team401.units.measure.acceleration.angular.MeasureRadiansPerSecondPerSecond
import org.team401.units.measure.distance.angular.MeasureRadians
import org.team401.units.measure.time.MeasureSeconds
import org.team401.units.measure.velocity.angular.MeasureRadiansPerSecond
import org.team401.util.PIDControllerLite
import org.team401.util.ProfiledPIDControllerLite
import org.team401.util.TrapezoidProfileLite
import org.team401.robot2020.util.getAcceleration
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier

/**
 * Class implementing control logic for a multi-rotational turret.  Automatically handles different control modes
 * such as rapid moves, tracking, holding, and jogging.
 *
 * @param Kp The P gain for rapid moves and holding
 * @param Kd The D gain for rapid moves and holding
 * @param trackingKp The P gain for tracking moves
 * @param trackingKd The D gain for tracking moves
 * @param Ks The static friction feed forward coefficient
 * @param Kv The velocity feed forward coefficient
 * @param Ka The acceleration feed forward coefficient
 * @param velocityConstraint The velocity constraint for rapid moves
 * @param accelerationConstraint The acceleration constraint for rapid moves
 * @param rapidThresholdDistance The minimum required rotation which will trigger a rapid move
 * @param lowerLimitAngleAbsolute The absolute lower travel limit of the turret, as measured by the encoder
 * @param upperLimitAngleAbsolute The absolute upper travel limit of the turret, as measured by the encoder
 * @param inputProvider The provider for input data about the turret (position, velocity)
 * @param outputProvider The provider for output signals of the turret (percent output control)
 * @param rate The rate the controller will be updated at, in seconds
 */
class TurretController(
    private val Kp: Double,
    private val Kd: Double,
    private val trackingKp: Double,
    private val trackingKd: Double,

    private val Ks: Double,
    private val Kv: Double,
    private val Ka: Double,

    private val velocityConstraint: MeasureRadiansPerSecond,
    private val accelerationConstraint: MeasureRadiansPerSecondPerSecond,

    private val rapidThresholdDistance: MeasureRadians,

    private val lowerLimitAngleAbsolute: MeasureRadians,
    private val upperLimitAngleAbsolute: MeasureRadians,

    private val positionRadiansSupplier: DoubleSupplier,
    private val velocityRadPerSecSupplier: DoubleSupplier,
    private val voltageConsumer: DoubleConsumer,

    private val rate: MeasureSeconds
) {
    //Controllers and model
    private val rapidConstraints = TrapezoidProfileLite.Constraints(velocityConstraint.value, accelerationConstraint.value)
    private val rapidController = ProfiledPIDControllerLite(Kp, 0.0, Kd, rapidConstraints, rate.value)
    private var lastRapidState = TrapezoidProfileLite.State()
    private val holdController = PIDControllerLite(Kp, 0.0, Kd, rate.value)
    private val trackingController = PIDControllerLite(trackingKp, 0.0, trackingKd, rate.value)
    private val model = SimpleMotorFeedforward(Ks, Kv, Ka)

    //State info
    private var currentPosition = 0.0.Radians
    private var currentVelocity = 0.0.RadiansPerSecond

    init {
        rapidController.setTolerance(Double.POSITIVE_INFINITY)
    }

    //Limit checking functions.  Return true for a safe angle, false for an unsafe angle
    private fun checkLowerLimit(angle: MeasureRadians) = (angle >= lowerLimitAngleAbsolute)
    private fun checkUpperLimit(angle: MeasureRadians) = (angle <= upperLimitAngleAbsolute)
    private fun checkLimits(angle: MeasureRadians) = checkLowerLimit(angle) && checkUpperLimit(angle)

    //Handles turret wraparound and shortest distance tracking using last known position
    private fun wrapAngle(targetRotationIn: Rotation2d): MeasureRadians {
        val targetRotation = Rotation2d(targetRotationIn.radians) //Clamp target to -180..180
        val currentRotation = Rotation2d(currentPosition.value) //Clamp current position to -180..180
        val delta = currentRotation.unaryMinus().rotateBy(targetRotation) //Solve distance to go

        var targetPosition = currentPosition + delta.radians.Radians //Attempt shortest route
        if (!checkUpperLimit(targetPosition)) {
            targetPosition -= 360.0.Degrees.toRadians() //Turret is unsafe in positive, shortest route is reverse
        }
        if (!checkLowerLimit(targetPosition)) {
            targetPosition += 360.0.Degrees.toRadians() //Turret is unsafe in negative, shortest route is forward
        }

        return targetPosition //This is now guaranteed to be clamped safely
    }

    //Checks if a move should be a rapid
    private fun shouldRapid(targetPosition: MeasureRadians): Boolean {
        return (targetPosition - currentPosition).abs() > rapidThresholdDistance
    }

    private enum class ControlMode {
        Angle,
        Jog,
        Hold,
        AbsoluteAngle
    }

    private var currentControlMode = ControlMode.Hold

    private fun updateState() {
        //Read inputs from the turret
        currentPosition = positionRadiansSupplier.asDouble.Radians
        currentVelocity = velocityRadPerSecSupplier.asDouble.RadiansPerSecond
    }

    private fun updateTracking(goal: Rotation2d, feedVelocity: MeasureRadiansPerSecond) {
        val angle = wrapAngle(goal)

        if (shouldRapid(angle)) {
            //We need to rapid to the angle
            updateRapid(angle)
        } else {
            //No rapid, standard tracking
            isInRapid = false

            val feedbackVolts = trackingController.calculate(currentPosition.value, angle.value)
            val ffVolts = model.calculate(feedVelocity.value)

            voltageConsumer.accept(feedbackVolts + ffVolts)
        }
    }

    //Rapid
    var isInRapid = false
        private set

    private fun updateRapid(goal: MeasureRadians) {
        if (!isInRapid) {
            //Rapid is starting
            lastRapidState = TrapezoidProfileLite.State(currentPosition.value, currentVelocity.value)
            rapidController.reset(lastRapidState)
            rapidController.setGoal(goal.value)
            isInRapid = true
        }
        //Update the rapid
        val feedbackVolts = rapidController.calculate(currentPosition.value, goal.value)
        val ffVolts = model.calculate(
                rapidController.setpoint.velocity,
                rapidController.getAcceleration(lastRapidState)
        )

        lastRapidState = rapidController.setpoint
        voltageConsumer.accept(feedbackVolts + ffVolts)
    }

    //Hold mode
    fun enterHold() {
        updateState()
        holdController.reset()
        holdController.setpoint = currentPosition.value
        currentControlMode = ControlMode.Hold
        isInRapid = false
    }

    fun updateHold() {
        if (currentControlMode != ControlMode.Hold) return

        updateState()
        val feedbackVolts = holdController.calculate(currentPosition.value)
        voltageConsumer.accept(feedbackVolts)
    }

    //Jog mode
    private var jogSetpoint = GeomUtil.IDENTITY_ROTATION

    fun enterJog() {
        updateState()
        trackingController.reset()
        jogSetpoint = Rotation2d(currentPosition.value)
        currentControlMode = ControlMode.Jog
    }

    fun updateJog(rate: MeasureRadiansPerSecond, dt: MeasureSeconds) {
        if (currentControlMode != ControlMode.Jog) return

        updateState()
        val jogAdvance = Rotation2d((rate * dt).value)
        jogSetpoint = jogSetpoint.rotateBy(jogAdvance)

        updateTracking(jogSetpoint, 0.0.RadiansPerSecond)
    }

    fun enterAngle() {
        updateState()
        trackingController.reset()
        currentControlMode = ControlMode.Angle
    }

    fun updateAngle(angleGoal: Rotation2d, feedVelocity: MeasureRadiansPerSecond = 0.0.RadiansPerSecond) {
        if (currentControlMode != ControlMode.Angle) return

        updateState()
        updateTracking(angleGoal, feedVelocity)
    }

    fun enterAbsoluteAngle() {
        updateState()
        lastRapidState = TrapezoidProfileLite.State(currentPosition.value, currentVelocity.value)
        rapidController.reset(lastRapidState)
        currentControlMode = ControlMode.AbsoluteAngle
    }

    fun updateAbsoluteAngle(angleAbsolute: MeasureRadians) {
        updateState()
        val feedbackVolts = rapidController.calculate(currentPosition.value, angleAbsolute.value)
        val ffVolts = model.calculate(
                rapidController.setpoint.velocity,
                rapidController.getAcceleration(lastRapidState)
        )
        lastRapidState = rapidController.setpoint

        voltageConsumer.accept(feedbackVolts + ffVolts)
    }
}