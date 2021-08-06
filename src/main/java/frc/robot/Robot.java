// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.turret.CharacterizeTurret;
import frc.robot.hid.HID;
import frc.robot.hid.RealHID;
import frc.robot.hid.TestHID;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveIOReal;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frckit.util.GeomUtil;


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
    private ClimbSubsystem climb;
    private FeederSubsystem feeder;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private ShooterSubsystem shooter;

    private RobotCommands commands;


    public Robot() {
        super(Constants.kDt);
    }

    @Override
    public void robotInit() {
        Threads.setCurrentThreadPriority(true, 99);

        if (isSimulation()) {
            // Simulation mode
            hid = new TestHID();
            drive = new DriveSubsystem(new DriveIOSim());
            intake = new IntakeSubsystem(new IntakeIOSim());
            climb = new ClimbSubsystem(new ClimbIOSim());
            feeder = new FeederSubsystem(new FeederIOSim());
            turret = new TurretSubsystem(new TurretIOSim());
        } else {
            FeederIOReal feederIO = new FeederIOReal();
            hid = new RealHID();
            intake = new IntakeSubsystem(new IntakeIOReal());
            climb = new ClimbSubsystem(new ClimbIOReal());
            feeder = new FeederSubsystem(feederIO);
            drive = new DriveSubsystem(new DriveIOReal(feederIO.getPigeonTalon()));
            turret = new TurretSubsystem(new TurretIOReal());
            shooter = new ShooterSubsystem(new ShooterIOReal());
        }
        vision = new VisionSubsystem();

        commands = new RobotCommands(hid, drive, intake, climb, feeder, turret, vision, shooter);

        //drive.setDefaultCommand(commands.driveOperatorControl);
        //intake.setDefaultCommand(commands.stowIntake);
        //climb.setDefaultCommand(commands.runClimbers);
        //feeder.setDefaultCommand(commands.loadTower);

        turret.setDefaultCommand(commands.stowTurret);

        vision.setDefaultCommand(commands.idleVision);

        LiveWindow.disableAllTelemetry(); // Disable telemetry because it eats performance
        NetworkTableInstance.getDefault().setUpdateRate(0.01);
    }

    @Override
    public void robotPeriodic() {
        timestamp = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
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
        RobotState.getInstance().forceRobotPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
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
