// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.DriveCommands;
import frc.robot.ShiftHelpers;

@Logged
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final double defaultFeederSpeed = 0.6;//0.5;
    private final double hubXBlue = 4.6;
    private final double hubXRed = 11.91;
    private double hubX = 0.0;
    private double hubY = 4.0;
    private double offsetX = 0.0;
    private double offsetY = 0.0;

    Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
            // 9999999.0 // Trust down to 2 degrees rotational
            );
            
            Matrix<N3, N1> LIMELIGHT_STD_DEVS =
            VecBuilder.fill(
            0.5, // Trust down to 2cm in X direction
            0.5, // Trust down to 2cm in Y direction
            0.3 // Trust down to 2 degrees rotational
            // 9999999.0 // Trust down to 2 degrees rotational
        );

    private boolean isAligning = false;
    private boolean brakeMode = false;
    private double rotDeg = 0.0;
    private double distance = 0.0;
    private boolean megatag1Reset = false;
    private boolean m_trackQuest = true;
    private int count = 0;

    Field2d m_field = new Field2d();
    private Geofencing m_geofenceAlliBump;
    private Geofencing m_geofenceOppBump;
    private Geofencing m_geofenceNeutZone;

    private Pose2d startAndClimbStart = new Pose2d(13.71, 4.0, new Rotation2d(Math.PI));
    private Pose2d feederOutpostSideStart = new Pose2d(13.01, 5.44, new Rotation2d( -3 * Math.PI / 4));
    private Pose2d feederDepotSideStart = new Pose2d(13.01, 2.66, new Rotation2d( 3 * Math.PI / 4));
    private Pose2d outpostToDepot = new Pose2d(13.06, 4.03, Rotation2d.k180deg);
    // private Pose2d autoStartPoint = Pose2d.kZero;

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

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController buttonBox = new CommandXboxController(1);

    public final Drive drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Transfer transfer = new Transfer();

    private boolean isinTransition = false;
    // private boolean isTrackingFuel = false;
    private boolean isTrackingHub = false;
    private boolean slowmode = false;
    private boolean m_bOverrideBumpControl = false;
    private boolean isBlue = false;

    // private final double X_START_BUMP = 1.0;
    // private final double X_STOP_BUMP = 4.0;
    // private final double TRANSITION_OFFSET = 0.25;
    // private final double X_START_TRANSITION = X_START_BUMP - TRANSITION_OFFSET;
    // private final double X_STOP_TRANSITION = X_STOP_BUMP + TRANSITION_OFFSET;

    private double rotFuelTracking = 0.0;
    private double robotX = 0.0;
    private double robotY = 0.0;

    // private double[] tarPose;
    // private Transform2d targPose3d;
    // private double tarX = 0.0;
    // private double tarY = 0.0;

    enum JogState{noJog, leftJog, rightJog};
    private JogState jogState = JogState.noJog;

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // drivetrain.resetPose(new Pose2d(0.335, 0.355, Rotation2d.k180deg));
// for sim testing drivetrain.resetPose(new Pose2d(8.0, 6.0, Rotation2d.kZero));
        NamedCommands.registerCommand("runIntake", m_intakeSeq);
        NamedCommands.registerCommand("agitateIntake", m_agitateIntake);
        NamedCommands.registerCommand("toggleTurretOn", m_toggleTurretOn);
        NamedCommands.registerCommand("toggleTurretOff", m_toggleTurretOff);
        NamedCommands.registerCommand("stopIntake", m_stopIntake);
        NamedCommands.registerCommand("ShootCommand", m_shootSeq);
        NamedCommands.registerCommand("StopShooter", m_stopShootSeq);
        NamedCommands.registerCommand("enableFlywheel", m_enableFlywheel);

        SmartDashboard.putNumber("FrameAgitate", Intake.m_frame);
        SmartDashboard.putNumber("PartialAgitate", Intake.m_partial);
        SmartDashboard.putNumber("MidExtendAgitate", Intake.m_midExtend);

        autoChooser = AutoBuilder.buildAutoChooser("StartAndClimbAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("RobotPose", m_field);
        configureBindings();

        SmartDashboard.putBoolean("Shift Ours?", ShiftHelpers.currentShiftIsYours());
        SmartDashboard.putNumber("Shift Time", 0.0);
        SmartDashboard.putNumber("Match Time", 0.0);
        SmartDashboard.putNumber("Deploy Turns", 0.0);

        SmartDashboard.putNumber("inputRPM", 1000.0);
        SmartDashboard.putNumber("ShooterSpeed", 0.0);
        SmartDashboard.putBoolean("disableShooter", Constants.defaultFlywheel); // default disables shooter

        SmartDashboard.putNumber("FeederSpeed", defaultFeederSpeed);

        SmartDashboard.putBoolean("MegaTag2", false);

        boolean isBlue = isBlue();
        m_geofenceAlliBump = isBlue ? Constants.m_geofenceBlueBump : Constants.m_geofenceRedBump;
        m_geofenceOppBump  = isBlue ? Constants.m_geofenceRedBump : Constants.m_geofenceBlueBump;
        m_geofenceNeutZone = isBlue ? Constants.m_geofenceNeutZoneIfBlue : Constants.m_geofenceNeutZoneIfRed;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                if (!m_bOverrideBumpControl && (m_geofenceAlliBump.isInZone(drivetrain.getPose()) || m_geofenceOppBump.isInZone(drivetrain.getPose()))) {
                    if (!isAligning) {
                        isAligning = true;
                        rotDeg = drivetrain.getRotationDegrees(); // gets once per fence entry
                    }
                    return driveAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.3)
                                     .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3)
                                     .withTargetDirection(getBumpAlignAngle(rotDeg));
                }
                // else if (isInRotation()) {
                //     return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.3)
                //                 .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3)
                //                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.3); 
                // }
                // else if (isinTransition) {
                //     Rotation2d rot = drivetrain.getPose().getRotation();
                //     double rotDouble = Math.round((rot.getDegrees()) / 90.0) * 90.0; // Rounds to the nearest 90 degrees
                //     Rotation2d targetRot = new Rotation2d(rotDouble / 180 * Math.PI);
                //     return driveAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                //                      .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                //                      .withTargetDirection(targetRot);
                // }
                // else if (isTrackingFuel) {
                //     Rotation2d rot = drivetrain.getPose().getRotation();
                //     Rotation2d targetRot = new Rotation2d((rot.getDegrees() - rotFuelTracking) / 180 * Math.PI);
                //     return driveAngleRobot.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                //                      .withVelocityY(0.0)
                //                      .withTargetDirection(targetRot);
                // }
                else if (isTrackingHub) {
                    Rotation2d targetRot = new Rotation2d(Math.atan2(drivetrain.getFieldY() - hubY + offsetY, drivetrain.getFieldX() - hubX + offsetX) + (isBlue ? Math.PI : 0.0));
                    return driveAngleRobot.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                     .withTargetDirection(targetRot);
                }
                else if (brakeMode){
                    return brake;
                }
                else if (jogState != JogState.noJog) {
                    double angle = 0.5; // Half a radian per sec
                    angle *= jogState == JogState.rightJog ? 1.0 : -1.0;
                    return drive.withVelocityX(0.0)
                                .withVelocityY(0.0)
                                .withRotationalRate(angle);
                }
                else{
                    isAligning = false;
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
        configureSecondaryBindings();
    }

    private void configurePrimaryBindings() {
        joystick.a().onTrue(m_shootSeq);
        joystick.b().onTrue(m_stopShootSeq);
        joystick.x().onTrue(m_agitateIntake);
        joystick.y().onTrue(m_toggleTurret);
        joystick.povUp().onTrue(m_runIntake2);
        joystick.povRight().onTrue(m_stopIntakeSeq);
        joystick.povLeft().onTrue(m_intakeSeq);
        joystick.povDown().onTrue(m_homeIntakeSeq);

        // joystick.back().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        // joystick.rightBumper().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));

        // joystick.y().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        // joystick.back().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));
        // joystick.rightBumper().onTrue(DriveCommands.driveToPoseCommand(drivetrain, () -> getDriveToPose()));

        joystick.start().onTrue(m_slowmode);
        
        joystick.rightTrigger().onTrue(m_overrideBumpControl);
        joystick.rightBumper().onTrue(m_toggleQuest);

        joystick.leftTrigger().onTrue(new InstantCommand(() -> brakeMode = true));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> brakeMode = false));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(new SequentialCommandGroup(drivetrain.runOnce(drivetrain::seedFieldCentric)
        //                             , new InstantCommand(() -> drivetrain.getPigeon2().reset())));  
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));  

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureSecondaryBindings() {
        // Physical layout and XBox assignment
        // +-------+---------------+-------+
        // ¦Green1 ¦White2 ¦ Blue2 ¦Green3 ¦
        // ¦  X    ¦  Back ¦ Start ¦  DU   ¦
        // +-------+-------+-------¦-------¦
        // ¦Yellow1¦Green2 ¦ Red2  ¦ Blue3 ¦
        // ¦  Y    ¦  LS   ¦  RS   ¦  DD   ¦
        // +-------+-------+-------¦-------¦
        // ¦ Blue1 ¦Black2 ¦Yellow2¦ Red3  ¦
        // ¦  RB   ¦  B    ¦  A    ¦  DR   ¦
        // +-------+-------+-------¦-------¦
        // ¦Black1 ¦White1 ¦ Red1  ¦Yellow3¦        
        // ¦  LB   ¦   LT  ¦  RT   ¦  DL   ¦        
        // +-----------------------+-------+ 
        // ****************************************Buttons listed in column major order
        buttonBox.x().onTrue(m_homeIntakeSeq);                  // Green 1
        buttonBox.y().onTrue(m_toggleFlywheel);                 // Yellow 1
        buttonBox.rightBumper().onTrue(m_toggleIntakeRoller);   // Blue 1
        buttonBox.leftBumper().onTrue(m_intakeSeq);             // Black 1

        buttonBox.back().onTrue(m_resetPrevDist);               // White 2 
        //buttonBox.leftStick().onTrue(m_);                       // Green 2
        //buttonBox.b().onTrue();                                 // Black 2
        buttonBox.leftTrigger().onTrue(m_stopIntakeSeq);        // White 1

        buttonBox.start().onTrue(m_resetQuest);                                                     // Blue 2 
        buttonBox.rightStick().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));   // Red 2
        buttonBox.a().whileTrue(shortAgitateCommand.repeatedly());                                     // Yellow 2
        buttonBox.rightTrigger().onTrue(m_shootSeq);                                                // Red 1
        buttonBox.rightTrigger().whileFalse(m_stopShootSeq);                                        // Red 1

        buttonBox.povUp().onTrue(m_resetTurret);                                                    // Green 3
        buttonBox.povDown().onTrue(m_toggleTurret);                                                 // Blue 3
        buttonBox.povRight().whileTrue(agitateCommand.repeatedly());                                // Red 3
        buttonBox.povLeft().onTrue(m_stopShootSeq);                                                 // Yellow 3
    }

    public Pose2d getDriveToPose() {
        String selectedAuto = SmartDashboard.getString("Auto Mode/selected", "noAuto");
        if (selectedAuto.equalsIgnoreCase("FeederOutpostAuto")) {
            return feederOutpostSideStart;
        }
        else if (selectedAuto.equalsIgnoreCase("FeederDepotAuto")) {
            return feederDepotSideStart;
        }
        else if (selectedAuto.equalsIgnoreCase("StartAndClimbAuto")) {
            return startAndClimbStart;
        }
        else if (selectedAuto.equalsIgnoreCase("OutpostToDepot")) {
            return outpostToDepot;
        }

        return Pose2d.kZero;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void periodic() {
        boolean megatag2 = SmartDashboard.getBoolean("MegaTag2", false);

        SmartDashboard.putBoolean("slowMode", slowmode);
        SmartDashboard.putBoolean("ReadyToShoot", isTrackingHub);

        SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());

        if (vision.isTracking() && m_trackQuest){
            drivetrain.addVisionMeasurement(vision.getQuestRobotPose(), vision.getTimestamp(), QUESTNAV_STD_DEVS);
        }
        
        else if (vision.isLLTracking()){
            LimelightHelpers.PoseEstimate poseEst;
            if (!megatag2)
            {
                poseEst = vision.getBotPoseEstimate();
            }
            else {
                poseEst = vision.getBotPoseEstimateMegaTag2();
            }

            if (poseEst != null) {
                SmartDashboard.putNumber("LLRotEst", poseEst.pose.getRotation().getDegrees());
                drivetrain.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds, LIMELIGHT_STD_DEVS);
            }
        }
        
        isBlue = isBlue();
        m_geofenceAlliBump = isBlue ? Constants.m_geofenceBlueBump : Constants.m_geofenceRedBump;
        m_geofenceOppBump  = isBlue ? Constants.m_geofenceRedBump : Constants.m_geofenceBlueBump;
        m_geofenceNeutZone = isBlue ? Constants.m_geofenceNeutZoneIfBlue : Constants.m_geofenceNeutZoneIfRed;

        if (isBlue){
            hubX = hubXBlue;
        }
        else {
            hubX = hubXRed;
        }
        offsetX = drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond * shooter.TOFtable.get(shooter.m_distance);
        offsetY = drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond * shooter.TOFtable.get(shooter.m_distance);

        // Moved the distance calc to shooter to keep the flywheeel ramped up
        shooter.setIsBlue(isBlue);
        shooter.setHubX(hubX);
        shooter.setRobotPose(drivetrain.getPose());
        shooter.setRobotSpeed(drivetrain.getFieldRelativeSpeeds());
        shooter.setNeutralZone(m_geofenceNeutZone);

        SmartDashboard.putBoolean("NuetralZone?", m_geofenceNeutZone.isInZone(drivetrain.getPose()));

        // same as pigeon yaw SmartDashboard.putNumber("PigeonRotation", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("PoseRotation", drivetrain.getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("PigeonYaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("PigeonHeading", drivetrain.getPigeon2().getRotation2d().getDegrees());

        updateDashboardFieldMap();

        // m_field.getObject("Fuel").setPose(drivetrain.getFieldX() + getDistanceXToFuel(vision.photonGetFuelPitch()), drivetrain.getFieldY() + getDistanceYToFuel(vision.getFuelAngle()), Rotation2d.kZero);
        SmartDashboard.putData("RobotPose", m_field);

        isinTransition = false;
        // isinTransition = (x > X_START_TRANSITION && x < X_START_BUMP) || (x > X_STOP_BUMP && x < X_STOP_TRANSITION);

        // if (vision.photonIsTrackingFuel()) {
        //     rotFuelTracking = vision.photonGetFuelAngle();
        //     targPose3d = vision.photonGetTargetPose();
        //     tarX = targPose3d.getX();
        //     tarY = targPose3d.getY();
        // }
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

        SmartDashboard.putBoolean("IsBlue", isBlue);
        SmartDashboard.putBoolean("IsAligning", isAligning);

        SmartDashboard.putString("SimAllianceID", DriverStationSim.getAllianceStationId().toString());

        SmartDashboard.putBoolean("IsInBump", m_geofenceAlliBump.isInZone(drivetrain.getPose()));
        SmartDashboard.putBoolean("IsInTransition", isinTransition);
        // SmartDashboard.putBoolean("IsTrackingFuel", isTrackingFuel);

        // SmartDashboard.putNumber("TargetX", tarX);
        // SmartDashboard.putNumber("TargetY", tarY);

        SmartDashboard.putBoolean("Shift Ours?", ShiftHelpers.currentShiftIsYours());
        SmartDashboard.putNumber("Shift Time", ShiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime()));
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    public void updateDashboardFieldMap() {
         m_field.setRobotPose(drivetrain.getPose());
    }

    InstantCommand m_runIntake = new InstantCommand(() -> intake.runIntake(false));
    InstantCommand m_runIntake2 = new InstantCommand(() -> intake.runIntake(false));
    InstantCommand m_runIntake3 = new InstantCommand(() -> intake.runIntake(false));
    InstantCommand m_runIntake4 = new InstantCommand(() -> intake.runIntake(false));
    InstantCommand m_runIntakeReverse = new InstantCommand(() -> intake.runIntake(true));

    InstantCommand m_stopIntakeArms = new InstantCommand(()-> intake.stopArms());
    InstantCommand m_stopIntakeArms2 = new InstantCommand(()-> intake.stopArms());

    InstantCommand m_stopIntake = new InstantCommand(() -> intake.stopIntake());
    InstantCommand m_stopIntake2 = new InstantCommand(() -> intake.stopIntake());
    InstantCommand m_stopIntake3 = new InstantCommand(() -> intake.stopIntake());
    InstantCommand m_stopIntake4 = new InstantCommand(() -> intake.stopIntake());
    InstantCommand m_stopIntake5 = new InstantCommand(() -> intake.stopIntake());
    InstantCommand m_stopIntake6 = new InstantCommand(() -> intake.stopIntake());

    InstantCommand m_runKicker = new InstantCommand(() -> transfer.setFeederSpeed(SmartDashboard.getNumber("FeederSpeed", defaultFeederSpeed)));
    InstantCommand m_stopKicker = new InstantCommand(()-> transfer.stopFeeder());

    InstantCommand m_homeIntake = new InstantCommand(() -> intake.deploy(Intake.m_home));
    InstantCommand m_homeIntake2 = new InstantCommand(() -> intake.deploy(Intake.m_home));

    InstantCommand m_frameIntake = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("FrameAgitate", Intake.m_frame);
        intake.deploy(pos);
    });

//    InstantCommand m_partialIntake = new InstantCommand(() -> intake.deploy(Intake.m_partial));
//    InstantCommand m_partialIntake2 = new InstantCommand(() -> intake.deploy(Intake.m_partial));
//    InstantCommand m_partialIntake3 = new InstantCommand(() -> intake.deploy(Intake.m_partial));
    InstantCommand m_partialIntake = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("PartialAgitate", Intake.m_partial);
        intake.deploy(pos);
    });
    InstantCommand m_partialIntake2 = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("PartialAgitate", Intake.m_partial);
        intake.deploy(pos);
    });
    InstantCommand m_partialIntake3 = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("PartialAgitate", Intake.m_partial);
        intake.deploy(pos);
    });

    InstantCommand m_midExtend = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("MidExtendAgitate", Intake.m_midExtend);
        intake.deploy(pos);
    });
    InstantCommand m_midExtend2 = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("MidExtendAgitate", Intake.m_midExtend);
        intake.deploy(pos);
    });
    InstantCommand m_midExtend3 = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("MidExtendAgitate", Intake.m_midExtend);
        intake.deploy(pos);
    });
    InstantCommand m_midExtend4 = new InstantCommand(() -> {
        double pos = SmartDashboard.getNumber("MidExtendAgitate", Intake.m_midExtend);
        intake.deploy(pos);
    });

    InstantCommand m_extendIntake = new InstantCommand(() -> intake.deploy(Intake.m_extend));
    InstantCommand m_extendIntake2 = new InstantCommand(() -> intake.deploy(Intake.m_extend));
    InstantCommand m_extendIntake3 = new InstantCommand(() -> intake.deploy(Intake.m_extend));
    InstantCommand m_resetPrevDist = new InstantCommand(() -> shooter.resetPrevDist());

    InstantCommand m_runSpindexer = new InstantCommand(() -> transfer.setSpinDexSpeed(false));
    InstantCommand m_runSpindexer2 = new InstantCommand(() -> transfer.setSpinDexSpeed(false));
    InstantCommand m_runSpindexerReverse = new InstantCommand(() -> transfer.setSpinDexSpeed(true));
    InstantCommand m_stopSpindexer = new InstantCommand(() -> transfer.stopSpinDex());
    InstantCommand m_stopSpindexer2 = new InstantCommand(() -> transfer.stopSpinDex());

    InstantCommand m_stopShooter = new InstantCommand(()-> shooter.stopShooter());
    InstantCommand m_enableFlywheel = new InstantCommand(() -> SmartDashboard.putBoolean("disableShooter", Constants.defaultFlywheel));

    InstantCommand m_resetQuest = new InstantCommand(() -> vision.updateQuestPose());
    //InstantCommand m_resetQuest = new InstantCommand(() -> vision.setQuestPose(new Pose3d(feederOutpostSideStart.getX(), feederOutpostSideStart.getY(), 0.0, Rotation3d.kZero)));
    InstantCommand m_resetOdometry = new InstantCommand(() -> drivetrain.resetPose(new Pose2d(0.335, 0.355, Rotation2d.kZero)));
    // InstantCommand m_trackFuel = new InstantCommand(() -> isTrackingFuel = !isTrackingFuel);
    InstantCommand m_trackHub = new InstantCommand(() -> isTrackingHub = !isTrackingHub);
    InstantCommand m_slowmode = new InstantCommand(() -> {
        slowmode = !slowmode;
        if (slowmode){
            MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed = 
        }
        else {
            MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed;=
        }
    });

    InstantCommand m_overrideBumpControl = new InstantCommand(() -> {
        m_bOverrideBumpControl = !m_bOverrideBumpControl;
    });

    InstantCommand m_toggleQuest = new InstantCommand(() -> {
        m_trackQuest = !m_trackQuest;
    });

    // InstantCommand m_jogLeft = new InstantCommand(() -> jogState = JogState.leftJog);
    // InstantCommand m_jogRight = new InstantCommand(() -> jogState = JogState.rightJog);
    // InstantCommand m_jogStop = new InstantCommand(() -> jogState = JogState.noJog);
    
    InstantCommand m_resetTurret = new InstantCommand(() -> shooter.resetTurret());
    InstantCommand m_toggleTurret = new InstantCommand(() -> shooter.m_moveTurret = !shooter.m_moveTurret);
    InstantCommand m_toggleTurretOn = new InstantCommand(() -> shooter.m_moveTurret = true);
    InstantCommand m_toggleTurretOff = new InstantCommand(() -> shooter.m_moveTurret = false);
    InstantCommand m_toggleFlywheel = new InstantCommand(() -> { boolean isTesting = SmartDashboard.getBoolean("disableShooter", Constants.defaultFlywheel);
                                                                 SmartDashboard.putBoolean("disableShooter", !isTesting); 
                                                               } );
    InstantCommand m_toggleIntakeRoller = new InstantCommand(() -> { boolean isTesting = SmartDashboard.getBoolean("disableIntakeRoller", false);
                                                                 SmartDashboard.putBoolean("disableIntakeRoller", !isTesting); 
                                                               } );

    //WaitCommand m_waitHalfSec = new WaitCommand(0.5);
    WaitCommand m_waitQuarterSec = new WaitCommand(0.25);
    //WaitCommand m_waitHalfSec2 = new WaitCommand(0.5);
    WaitCommand m_waitHalfSec3 = new WaitCommand(0.5);
    //WaitCommand m_waitHalfSec4 = new WaitCommand(0.5);
    WaitCommand m_waitHalfSec5 = new WaitCommand(0.5);
    WaitCommand m_waitHalfSec6 = new WaitCommand(0.5);
    WaitCommand m_waitHalfSec7 = new WaitCommand(0.5);
    WaitCommand m_waitHalfSec8 = new WaitCommand(0.5);
    WaitCommand m_waitHalfSec9 = new WaitCommand(0.5);

    // Shooter is always has the flywheel ramped, so we can skip the delay and just start/stop the kicker
    SequentialCommandGroup m_stopShootSeq = new SequentialCommandGroup(m_stopKicker, m_stopSpindexer);
    SequentialCommandGroup m_shootSeq = new SequentialCommandGroup(/* m_partialIntake, */ m_runKicker, m_waitQuarterSec, m_stopIntakeArms2, m_runSpindexer2);

    SequentialCommandGroup m_intakeSeq = new SequentialCommandGroup(m_extendIntake, m_waitHalfSec3, m_runIntake, m_stopIntakeArms);
    SequentialCommandGroup m_stopIntakeSeq = new SequentialCommandGroup(/* m_frameIntake, */ m_stopIntake2);
    SequentialCommandGroup m_homeIntakeSeq = new SequentialCommandGroup(/* m_frameIntake3, */ m_waitHalfSec5, m_stopIntake3, m_homeIntake);
//    SequentialCommandGroup m_agitateIntake = new SequentialCommandGroup(m_extendIntake2, m_waitHalfSec6, m_partialIntake2, m_stopIntake5, m_waitHalfSec7, m_extendIntake3, m_runIntake3);
    SequentialCommandGroup m_agitateIntake = new SequentialCommandGroup(m_midExtend3, m_waitHalfSec6, m_frameIntake, m_stopIntake5, m_waitHalfSec7, m_midExtend4, m_runIntake3);
    SequentialCommandGroup m_shortAgitateIntake = new SequentialCommandGroup(m_midExtend, m_waitHalfSec8, m_partialIntake3, m_stopIntake6, m_waitHalfSec9, m_midExtend2, m_runIntake4);

    Command agitateCommand = m_agitateIntake;
    Command shortAgitateCommand = m_shortAgitateIntake;
    // Command agitateCommandAuto = agitateCommand.repeatedly().withTimeout(5);

    private Rotation2d getBumpAlignAngle(double currentRot){
        double alignDeg = Math.round((currentRot - 45.0) / 90.0) * 90.0 + 45.0; // Rounds to the nearest 45 degrees
        return new Rotation2d((alignDeg / 180 * Math.PI) + (isBlue() ? 0.0 : Math.PI)); // 180 based on alliance
    }
    
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
        else{
            System.out.println("Alliance Unknown");
        }

        return false;
        // if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
        //     return true;
        // }
        // return false;
        // if (RobotBase.isReal()) return isBlue; // TODO needs physical test
        // return (DriverStationSim.getAllianceStationId().toString().contains("Blue")); // isBlue doesn't work in sim and no direct way to get alliance, so need to check id (ex. Blue1)
    }

    private boolean isInRotation(){
        double rot = drivetrain.getRotationDegrees();
        boolean isTop = Constants.m_geofenceNeutTop.isInZone(drivetrain.getPose());
        boolean isBot = Constants.m_geofenceNeutBottom.isInZone(drivetrain.getPose());
        if (!isTop && !isBot){
            return false;
        }
        double angle1 = 160.0;
        double angle2 = 20.0;
        if (isTop){
            angle1 = -20.0;
            angle2 = -160.0;
        }
        
        return (rot < angle1 && rot > angle2);
    }
}
