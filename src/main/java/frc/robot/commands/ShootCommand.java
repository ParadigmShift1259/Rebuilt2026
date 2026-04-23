// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Geofencing;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  // private double m_poseX = 0.0;
  // private double m_poseY = 0.0;

  private final Shooter m_shooter;
  private final Drive m_drive;
  private boolean m_isBlue;

  private double distance = 0.0;

  private Geofencing m_geofenceNeutZone;

    private boolean isBlue(){
        var allianceOptional = DriverStation.getAlliance();

        if (allianceOptional.isPresent()){
            DriverStation.Alliance alliance = allianceOptional.get();

            switch (alliance){
                case Red:
                    return false;
                case Blue:
                    return true;
            }
        }
        // else{
        //     System.out.println("Alliance Unknown");
        // }

        return false;
        // if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
        //     return true;
        // }
        // return false;
        // if (RobotBase.isReal()) return isBlue; // TODO needs physical test
        // return (DriverStationSim.getAllianceStationId().toString().contains("Blue")); // isBlue doesn't work in sim and no direct way to get alliance, so need to check id (ex. Blue1)
    }

  /**
   * Creates a new IntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(Shooter shooter, Drive drive, boolean isBlue) {
    // m_poseX = poseX;
    // m_poseY = poseY;
    m_shooter = shooter;
    m_drive = drive;
    m_isBlue = isBlue;

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
    m_isBlue = isBlue();
    m_geofenceNeutZone = m_isBlue ? Constants.m_geofenceNeutZoneIfBlue : Constants.m_geofenceNeutZoneIfRed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Shooter is recalculating distance continuously in periodic m_shooter.setRPMDistance(distance);
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
