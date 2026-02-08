// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommands;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
        );

    Field2d m_field = new Field2d();

    private Command driveToPoseCommand;
    private Pose2d startAndClimbStart = new Pose2d(13.71, 4.0, new Rotation2d(Math.PI));
    private Pose2d feederOutpostSideStart = new Pose2d(13.01, 5.44, new Rotation2d( -3 * Math.PI / 4));
    private Pose2d feederDepotSideStart = new Pose2d(13.01, 2.66, new Rotation2d( 3 * Math.PI / 4));
    private Pose2d outpostToDepot = new Pose2d(13.06, 4.03, Rotation2d.k180deg);
    private Pose2d autoStartPoint = Pose2d.kZero;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate* 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(4.0, 0.0, 0.0);
    
    private final SwerveRequest.RobotCentricFacingAngle driveAngleRobot = new SwerveRequest.RobotCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate* 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withHeadingPID(4.0, 0.0, 0.0);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Drive drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision();
    public final Intake intake = new Intake();

    private boolean isinBump = false;
    private boolean isinTransition = false;
    private boolean isTrackingFuel = false;
    private boolean slowmode;
    private final double X_START_BUMP = 1.0;
    private final double X_STOP_BUMP = 4.0;
    private final double TRANSITION_OFFSET = 0.25;
    private final double X_START_TRANSITION = X_START_BUMP - TRANSITION_OFFSET;
    private final double X_STOP_TRANSITION = X_STOP_BUMP + TRANSITION_OFFSET;
    private double rotFuelTracking = 0.0;
    private double robotX = 0.0;
    private double robotY = 0.0;

    private double[] tarPose;
    private Transform2d targPose3d;
    private double tarX = 0.0;
    private double tarY = 0.0;

    enum JogState{noJog, leftJog, rightJog};
    private JogState jogState = JogState.noJog;

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("StartAndClimbAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("RobotPose", m_field);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                if (isinBump){
                    Rotation2d rot = drivetrain.getPigeon2().getRotation2d(); // DO NOT GET DRIVETRAIN STATE ROTATION! Quest messes with it. Get pigeon directly
                    SmartDashboard.putNumber("PigeonRotation", rot.getDegrees());
                    SmartDashboard.putNumber("PoseRotation", drivetrain.getState().Pose.getRotation().getDegrees());
                    double rotDouble = Math.round((rot.getDegrees() - 45.0) / 90.0) * 90.0 + 45.0; // Rounds to the nearest 45 degrees
                    // double rotDouble = Math.round((-rot.getDegrees()) / 45.0 + 45.0); // Rounds to the nearest 45 degrees
                    Rotation2d targetRot = new Rotation2d(rotDouble / 180 * Math.PI);
                    // if (rot.getDegrees() > -180 && rot.getDegrees() < -90){
                    //     targetRot = new Rotation2d(Math.PI * 3.0 / 4.0);
                    // }
                    // else if (rot.getDegrees() > -90 && rot.getDegrees() < 0){
                    //     targetRot = new Rotation2d(-Math.PI / 4.0);
                    // }
                    // else if (rot.getDegrees() < 180 && rot.getDegrees() > 90){
                    //     targetRot = new Rotation2d(Math.PI * 3.0 / 4.0);
                    // }
                    // else if (rot.getDegrees() < 90 && rot.getDegrees() > 0){
                    //     targetRot = new Rotation2d(Math.PI / 4.0);
                    // }

                    SmartDashboard.putNumber("targetRot", targetRot.getDegrees());
                    return driveAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.3)
                                     .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3)
                                     .withTargetDirection(targetRot);
                }
                else if (isinTransition) {
                    Rotation2d rot = drivetrain.getState().Pose.getRotation();
                    double rotDouble = Math.round((rot.getDegrees()) / 90.0) * 90.0; // Rounds to the nearest 90 degrees
                    Rotation2d targetRot = new Rotation2d(rotDouble / 180 * Math.PI);
                    return driveAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                     .withTargetDirection(targetRot);
                }
                else if (isTrackingFuel) {
                    Rotation2d rot = drivetrain.getState().Pose.getRotation();
                    Rotation2d targetRot = new Rotation2d((rot.getDegrees() - rotFuelTracking) / 180 * Math.PI);
                    return driveAngleRobot.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                     .withVelocityY(0.0)
                                     .withTargetDirection(targetRot);
                }
                else if (jogState != JogState.noJog) {
                    double angle = 0.5; // Half a radian per sec
                    angle *= jogState == JogState.rightJog ? 1.0 : -1.0;
                    return drive.withVelocityX(0.0)
                                .withVelocityY(0.0)
                                .withRotationalRate(angle);
                }
                else{
                    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                }
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        configurePrimaryBindings();
    }

    private void configurePrimaryBindings() {
        joystick.a().onTrue(m_runIntake);
        joystick.a().onFalse(m_stopIntake);
        // joystick.a().onTrue(m_trackFuel);
        // joystick.a().onFalse(m_trackFuel);
        joystick.b().onTrue(m_resetQuest);
        // joystick.x().onTrue(DriveCommands.driveToPoseCommand(drivetrain,
        //     () -> drivetrain.getPose().transformBy(vision.photonGetTargetPose())));

        joystick.y().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        joystick.back().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        joystick.rightBumper().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));

        // joystick.y().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        // joystick.back().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        // joystick.rightBumper().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));

        joystick.start().onTrue(m_slowmode);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.leftTrigger().onTrue(m_jogLeft);
        joystick.leftTrigger().onFalse(m_jogStop);
        joystick.rightTrigger().onTrue(m_jogRight);
        joystick.rightTrigger().onFalse(m_jogStop);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Pose2d getDriveToPose() {
        String selectedAuto = SmartDashboard.getString("Auto Mode/selected", "noAuto");
        SmartDashboard.putString("autoSelected", selectedAuto);
        System.out.println("getDriveToPose() selectedAuto = " + selectedAuto);
        if (selectedAuto.equalsIgnoreCase("FeederOutpostAuto")) {
            SmartDashboard.putNumber("autoFOSX", feederOutpostSideStart.getX());
            SmartDashboard.putNumber("autoFOSY", feederOutpostSideStart.getY());
            System.out.println("getDriveToPose() selectedAuto = " + selectedAuto + " X " + feederOutpostSideStart.getX() + " Y " +feederOutpostSideStart.getY());
            return feederOutpostSideStart;
        }
        else if (selectedAuto.equalsIgnoreCase("FeederDepotAuto")) {
            SmartDashboard.putNumber("autoFDSX", feederDepotSideStart.getX());
            SmartDashboard.putNumber("autoFDSY", feederDepotSideStart.getY());
            System.out.println("getDriveToPose() selectedAuto = " + selectedAuto + " X " + feederDepotSideStart.getX() + " Y " +feederDepotSideStart.getY());
            return feederDepotSideStart;
        }
        else if (selectedAuto.equalsIgnoreCase("StartAndClimbAuto")) {
            SmartDashboard.putNumber("autoSACX", startAndClimbStart.getX());
            SmartDashboard.putNumber("autoSACY", startAndClimbStart.getY());
            System.out.println("getDriveToPose() selectedAuto = " + selectedAuto + " X " + startAndClimbStart.getX() + " Y " +startAndClimbStart.getY());
            return startAndClimbStart;
        }
        else if (selectedAuto.equalsIgnoreCase("OutpostToDepot")) {
            SmartDashboard.putNumber("autoSACX", outpostToDepot.getX());
            SmartDashboard.putNumber("autoSACY", outpostToDepot.getY());
            System.out.println("getDriveToPose() selectedAuto = " + selectedAuto + " X " + outpostToDepot.getX() + " Y " +outpostToDepot.getY());
            return outpostToDepot;
        }

        System.out.println("getDriveToPose() returning Pose2d.kZero selectedAuto = " + selectedAuto);

        return Pose2d.kZero;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void periodic() {
        SmartDashboard.putBoolean("slowMode", slowmode);

        if (vision.isTracking()){
            drivetrain.addVisionMeasurement(vision.getQuestRobotPose(), vision.getTimestamp(), QUESTNAV_STD_DEVS);
        }

        SmartDashboard.putNumber("PigeonYaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("PigeonHeading", drivetrain.getPigeon2().getRotation2d().getDegrees());

        m_field.setRobotPose(drivetrain.getPose());
        // m_field.getObject("Fuel").setPose(drivetrain.getFieldX() + getDistanceXToFuel(vision.photonGetFuelPitch()), drivetrain.getFieldY() + getDistanceYToFuel(vision.getFuelAngle()), Rotation2d.kZero);
        SmartDashboard.putData("RobotPose", m_field);

        double x = drivetrain.getFieldX();
        double y = drivetrain.getFieldY();
        isinBump = x > 11.0 && x < 13.0 && y > 5.1 && y < 5.9;
        isinTransition = false;
        // isinTransition = (x > X_START_TRANSITION && x < X_START_BUMP) || (x > X_STOP_BUMP && x < X_STOP_TRANSITION);

        if (vision.photonIsTrackingFuel()) {
            rotFuelTracking = vision.photonGetFuelAngle();
            targPose3d = vision.photonGetTargetPose();
            tarX = targPose3d.getX();
            tarY = targPose3d.getY();
        }
        // else if (vision.isTrackingFuel()) {
        //     rotFuelTracking = vision.getFuelAngle();
        //     tarPose = vision.getTargetPose();
        //     tarX = tarPose[0];
        //     tarY = tarPose[1];
        // }

        robotX = drivetrain.getFieldX();
        robotY = drivetrain.getFieldY();

        SmartDashboard.putNumber("xPose", robotX);
        SmartDashboard.putNumber("yPose", robotY);

        SmartDashboard.putBoolean("IsInBump", isinBump);
        SmartDashboard.putBoolean("IsInTransition", isinTransition);
        SmartDashboard.putBoolean("IsTrackingFuel", isTrackingFuel);

        SmartDashboard.putNumber("TargetX", tarX);
        SmartDashboard.putNumber("TargetY", tarY);

        SmartDashboard.putNumber("PhotonYaw", rotFuelTracking);

        SmartDashboard.putNumber("ObjectX", getDistanceXToFuel(vision.photonGetFuelPitch()));
        SmartDashboard.putNumber("ObjectY", getDistanceYToFuel(vision.getFuelAngle()));
    }
    InstantCommand m_runIntake = new InstantCommand(() -> intake.runIntake());
    InstantCommand m_stopIntake = new InstantCommand(() -> intake.stopIntake());

    InstantCommand m_resetQuest = new InstantCommand(() -> vision.updateQuestPose());
    InstantCommand m_trackFuel = new InstantCommand(() -> isTrackingFuel = !isTrackingFuel);
    InstantCommand m_slowmode = new InstantCommand(() -> {
        slowmode = !slowmode;
        if (slowmode){
            MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed = 
        }
        else {
            MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed;=
        }
    });
    
    InstantCommand m_jogLeft = new InstantCommand(() -> jogState = JogState.leftJog);
    InstantCommand m_jogRight = new InstantCommand(() -> jogState = JogState.rightJog);
    InstantCommand m_jogStop = new InstantCommand(() -> jogState = JogState.noJog);

    public double getDistanceXToFuel(double angle){
        return -0.28 / Math.tan(angle * Math.PI / 180.0); // 0.28 is height from the floor to the camera in meters
    }

    public double getDistanceYToFuel(double angle){
        return Math.tan(angle * Math.PI / 180.0) * getDistanceXToFuel(vision.photonGetFuelPitch());
    }
}
