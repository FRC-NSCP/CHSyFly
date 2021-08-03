// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hid.HID;
import frc.robot.hid.TestHID;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private static double timestamp = 0.0;

    public static double getTimestamp() {
        return timestamp;
    }

    // Subsystem instances
    private HID hid;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;

    private RobotCommands commands;


    public Robot() {
        super(Constants.kDt);
    }

    @Override
    public void robotInit() {
        if (isSimulation()) {
            // Simulation mode
            hid = new TestHID();
            drive = new DriveSubsystem(new DriveIOSim());
            intake = new IntakeSubsystem(new IntakeIOSim());
        } else {
            // Real robot
            //TODO
        }

        commands = new RobotCommands(hid, drive, intake);

        drive.setDefaultCommand(commands.driveOperatorControl);
        intake.setDefaultCommand(commands.stowIntake);
    }

    @Override
    public void robotPeriodic() {
        timestamp = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        System.out.println(RobotState.getInstance().getLatestFieldToVehicle());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}
