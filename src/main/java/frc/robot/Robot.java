// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;

@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private boolean m_questReset = false;
    private boolean inPosition = false;
    private int count = 0;

    // private static final Current kSlipCurrent = Amps.of(120);

    LinearVelocity kMaxSpeed = MetersPerSecond.of(2.5);

    private final RobotContainer m_robotContainer = new RobotContainer();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        DataLogManager.start(); // Optional to mirror the NetworkTables-logged data to a file on disk
        Epilogue.bind(this);
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
        logMemoryUsage();
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.periodic();

       SmartDashboard.putBoolean("InPosition", inPosition);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        if (count % 250 == 0){
            count = 0;
            m_questReset = false;
        }

        if (m_robotContainer.vision.isLLTracking() && !m_questReset && count % 251 == 0){
            Pose2d questPose2d = m_robotContainer.vision.getLLRobotPose().toPose2d();
            Pose3d questPose = new Pose3d(questPose2d.getX() + 0.38, questPose2d.getY() - 0.145, 0.0, new Rotation3d(0.0, 0.0, 0.0));
            m_robotContainer.vision.setQuestPose(questPose);
            // System.out.println(DriverStation.getAlliance().get().toString());
            m_questReset = true;
        }

        if (m_robotContainer.vision.getQuestRobotPose().getX() > 12.9 && m_robotContainer.vision.getQuestRobotPose().getX() < 13.1 && m_robotContainer.vision.getQuestRobotPose().getY() > 5.3 && m_robotContainer.vision.getQuestRobotPose().getY() < 5.6)
        {
            inPosition = true;
        }
        else
        {
            inPosition = false;
        }

        count++;
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        m_robotContainer.periodic();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.periodic();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    private void logMemoryUsage() {
        try {
            Runtime runtime = Runtime.getRuntime();

            long totalMemory = runtime.totalMemory(); // bytes allocated to JVM
            long freeMemory = runtime.freeMemory();   // bytes free in allocated heap
            long usedMemory = totalMemory - freeMemory;
            long maxMemory = runtime.maxMemory();     // max heap size

            // Convert to MB for readability
            double usedMB = usedMemory / (1024.0 * 1024.0);
            double totalMB = totalMemory / (1024.0 * 1024.0);
            double maxMB = maxMemory / (1024.0 * 1024.0);

            SmartDashboard.putNumber("Memory Used (MB)", usedMB);
            SmartDashboard.putNumber("Memory Total (MB)", totalMB);
            SmartDashboard.putNumber("Memory Max (MB)", maxMB);
        } catch (Exception e) {
            SmartDashboard.putString("Memory Log Error", e.getMessage());
        }
    }
}
