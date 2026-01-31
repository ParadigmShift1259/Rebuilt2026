package frc.robot.commands;

import java.security.PrivateKey;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
// import com.ctre.phoenix6.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class DriveCommands {
  
    public static Command driveToPoseCommand(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
      final double TRANSLATION_KP = 2.25;
      final double TRANSLATION_KD = 0.0;

      final double ANGLE_KP = 5.0;
      final double ANGLE_KD = 0.4;

      final double ANGLE_MAX_VELOCITY = 8.0;
      final double ANGLE_MAX_ACCELERATION = 20.0;

      final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
                                                                               .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                                                                               .withDriveRequestType(DriveRequestType.Velocity)
                                                                               .withHeadingPID(4, 0, 0); // change these values for your robot

      PIDController xController = new PIDController(TRANSLATION_KP, 0, TRANSLATION_KD);
      PIDController yController = new PIDController(TRANSLATION_KP, 0, TRANSLATION_KD);
      ProfiledPIDController angleController =
          new ProfiledPIDController(
              ANGLE_KP,
              0.0,
              ANGLE_KD,
              new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
      angleController.enableContinuousInput(-Math.PI, Math.PI);

      return Commands.run(
              () -> {
                Pose2d targetPose = targetPoseSupplier.get();
                if (targetPose == Pose2d.kZero){
                    return;
                }

                Pose2d currentPose = drive.getState().Pose;
                // Calculate angular speed
                double omega =
                    angleController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());

                // Convert to field relative speeds & send command

                double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
                double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());

                if (Math.abs(xOutput) < 0.05) {
                  xOutput = 0;
                }

                if (Math.abs(yOutput) < 0.05) {
                  yOutput = 0;
                }

                ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omega);
                // boolean isFlipped =
                //     DriverStation.getAlliance().isPresent()
                //         && DriverStation.getAlliance().get() == Alliance.Red;
                drive.setControl(m_request.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withTargetDirection(targetPose.getRotation()));
              },
              drive)
          .until(
              () -> {
                      Pose2d targetPose = targetPoseSupplier.get();
                      if (targetPose == Pose2d.kZero){
                          return true;
                      }
                      return targetPose.getTranslation().getDistance(drive.getState().Pose.getTranslation()) < 0.05;
                    })

          // Reset PID controller when command starts
          .beforeStarting(
              () -> {
                Pose2d targetPose = targetPoseSupplier.get();
                if (targetPose != Pose2d.kZero){
                  angleController.reset(drive.getState().Pose.getRotation().getRadians());
                  xController.setSetpoint(targetPose.getX());
                  yController.setSetpoint(targetPose.getY());
                }
              });
  }

  //public void setPose(Pose2d pose) {

  //}
}
