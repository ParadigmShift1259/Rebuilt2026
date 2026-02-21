// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  // private double m_poseX = 0.0;
  // private double m_poseY = 0.0;

  private final Shooter m_shooter;
  private final Drive m_drive;

  private double distance = 0.0;

  /**
   * Creates a new IntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(Shooter shooter, Drive drive) {
    // m_poseX = poseX;
    // m_poseY = poseY;
    m_shooter = shooter;
    m_drive = drive;
    // m_shooter = shooter;
    // m_transfer = transfer;
    // m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(drive);
    // addRequirements(transfer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = Math.sqrt(Math.pow((m_drive.getFieldX() - 4.6), 2) + Math.pow((m_drive.getFieldY() - 4.0), 2));
    SmartDashboard.putNumber("ShooterDistance", distance);

    m_shooter.setRPMDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
