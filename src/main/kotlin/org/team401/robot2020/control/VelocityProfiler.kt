package org.team401.robot2020.control

import org.team401.units.measure.RadiansPerSecond
import org.team401.units.measure.RadiansPerSecondPerSecond
import org.team401.units.measure.Seconds
import org.team401.units.measure._ul
import org.team401.units.measure.acceleration.angular.MeasureRadiansPerSecondPerSecond
import org.team401.units.measure.time.MeasureSeconds
import org.team401.units.measure.velocity.angular.MeasureRadiansPerSecond

/**
 * Implements a simple linear velocity ramp for controlling simple systems such as a flywheel.
 */
class VelocityProfiler(private val acceleration: MeasureRadiansPerSecondPerSecond) {
    companion object {
        @JvmStatic fun createFromJava(accelerationRadPerSecPerSec: Double) = VelocityProfiler(accelerationRadPerSecPerSec.RadiansPerSecondPerSecond)
    }

    /**
     * The profiled velocity command
     */
    var velocityCommand = 0.0.RadiansPerSecond
        private set

    /**
     * The profiled acceleration command
     */
    var accelerationCommand = 0.0.RadiansPerSecondPerSecond
        private set

    var goal = 0.0.RadiansPerSecond
        private set

    fun getVelocityCommandRadPerSec() = velocityCommand.value
    fun getAccelCommandRadPerSecPerSec() = accelerationCommand.value
    fun getGoalRadPerSec() = goal.value

    /**
     * Resets the velocity command to the given value, and zeroes the acceleration
     */
    fun reset(currentSpeed: MeasureRadiansPerSecond = 0.0.RadiansPerSecond) {
        velocityCommand = currentSpeed
        accelerationCommand = 0.0.RadiansPerSecondPerSecond
        goal = 0.0.RadiansPerSecond
    }

    fun resetFromJava(currentSpeed: Double) {
        reset(currentSpeed.RadiansPerSecond)
    }

    /**
     * Updates the values of velocityCommand and accelerationCommand based on the given inputs
     */
    fun calculate(dt: MeasureSeconds, goalVelocity: MeasureRadiansPerSecond) {
        val dv = dt * acceleration

        goal = goalVelocity

        if (goalVelocity > velocityCommand) { //Goal is above current command, accelerate positively
            velocityCommand += dv
            accelerationCommand = acceleration
            if (velocityCommand > goalVelocity) {
                velocityCommand = goalVelocity
                accelerationCommand = 0.0.RadiansPerSecondPerSecond
            }
        } else if (goalVelocity < velocityCommand) { //Goal is below current command, accelerate negatively
            velocityCommand -= dv
            accelerationCommand = acceleration * (-1.0)._ul
            if (velocityCommand < goalVelocity) {
                velocityCommand = goalVelocity
                accelerationCommand = 0.0.RadiansPerSecondPerSecond
            }
        } else { //Goal is equal to current command, no acceleration
            accelerationCommand = 0.0.RadiansPerSecondPerSecond
        }
    }

    fun calculateFromJava(dt: Double, goalVelocityRadPerSec: Double) {
        calculate(dt.Seconds, goalVelocityRadPerSec.RadiansPerSecond);
    }
}