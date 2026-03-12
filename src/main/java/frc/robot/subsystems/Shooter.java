package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.ConstantsCANIDS;
import frc.robot.Geofencing;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.Drive;


@Logged
public class Shooter extends SubsystemBase {
    // private Drive m_drive = TunerConstants.createDrivetrain();

    private final TalonFX m_flywheelMotorLead = new TalonFX(ConstantsCANIDS.kFlywheelLeadID);
    private final TalonFX m_flywheelMotorFollow = new TalonFX(ConstantsCANIDS.kFlywheelFollowID);
    private final VelocityVoltage m_vvReq = new VelocityVoltage(0).withSlot(0);

    private InterpolatingDoubleTreeMap RPMtable = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap TOFtable = new InterpolatingDoubleTreeMap();

    private Servo m_servo = new Servo(0);

    private SparkMax m_turretMot = new SparkMax(ConstantsCANIDS.kTurretID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_turretCtlr = m_turretMot.getClosedLoopController();
    private RelativeEncoder m_turretEnc = m_turretMot.getEncoder();
    
    //private SparkMax m_hoodMot = new SparkMax(ConstantsCANIDS.kHoodID, SparkMax.MotorType.kBrushless);
    //private SparkClosedLoopController m_hoodCtlr = m_hoodMot.getClosedLoopController();

    SparkMaxConfig configMax = new SparkMaxConfig();

    // private boolean isShooting = true; // shooter state
    enum ShootingState{noShoot, hubShoot, feedShoot};
    private ShootingState shootingState = ShootingState.noShoot;

    private double m_hubX = 0.0;
    private double yDist = 0.0;
    private double xDist = 0.0;
    public double m_distance = 0.0;
    public double m_prevDistance = 0.0;

    double offsetX = 0.0;
    double offsetY = 0.0;

    public double m_lastP = 0.0;
    public double m_lastD = 0.0;

    private double m_turretAngle = 0.0;
    // private static double m_radToTurns = 12.7 / (Math.PI / 2);
    private static double m_maxTurns = 12.2;
    private static double m_radToTurns = m_maxTurns / (Math.PI / 2);

    private Pose2d m_robotPose = Pose2d.kZero;
    private ChassisSpeeds m_ChassisSpeeds = new ChassisSpeeds();
    private Geofencing m_geofenceNeutZone;
    private boolean m_isBlue = false;
    public boolean m_moveTurret = false;

    public Shooter(){
        SmartDashboard.putNumber("offsetRPM", Constants.rpmBoost);

        SmartDashboard.putNumber("turretRad", 0.0);
        m_turretEnc.setPosition(0.0);
        // Add calibration points (distance in meters -> shooter RPM)
        RPMtable.put(2.6, 1850.0);
        RPMtable.put(3.0, 1900.0);
        RPMtable.put(3.5, 2000.0);
        RPMtable.put(4.0, 2100.0);
        RPMtable.put(4.5, 2200.0);
        RPMtable.put(5.0, 2300.0);
        RPMtable.put(5.5, 2400.0);
        RPMtable.put(6.0, 2500.0);
        //TOF = Time Of Flight
        TOFtable.put(5.68, 1.15);
        TOFtable.put(4.55, 1.13);
        TOFtable.put(3.15, 1.12);
        TOFtable.put(1.88, 1.09);
        TOFtable.put(1.38, 0.9);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // TODO figure out gear ratio
        
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.1;
        slot0.kV = 0.12;
        slot0.kP = 0.11;
        slot0.kI = 0;
        slot0.kD = 0.0;

        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                   .withPeakReverseVoltage(Volts.of(-8));
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_flywheelMotorLead.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = m_flywheelMotorFollow.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        m_flywheelMotorFollow.setControl(new Follower(m_flywheelMotorLead.getDeviceID(), MotorAlignmentValue.Opposed));

        configMax.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                       .p(0.1)
                       .d(0.02);
        m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SmartDashboard.putNumber("turretP", 0.15);
        SmartDashboard.putNumber("turretD", 0.1);
        // configMax.closedLoop.p(0.5);
        // m_hoodMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setHubX(double hubX) { m_hubX = hubX; }
    public void setRobotPose(Pose2d pose) { m_robotPose = pose; }
    public void setRobotSpeed(ChassisSpeeds speeds) { m_ChassisSpeeds = speeds; }
    public void setNeutralZone(Geofencing neutZone) { m_geofenceNeutZone = neutZone; }
    public void setIsBlue(boolean isBlue) { m_isBlue = isBlue; }

    @Override
    public void periodic() {
        if (m_geofenceNeutZone == null){
            return;
        }
        if (m_geofenceNeutZone.isInZone(m_robotPose)){
            shootingState = ShootingState.feedShoot;
        }

        else {
            shootingState = ShootingState.hubShoot;
        }

        if (m_lastP != SmartDashboard.getNumber("turretP", 0.1)){
            m_lastP = SmartDashboard.getNumber("turretP", 0.1);
            configMax.closedLoop.p(m_lastP);
            m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        if (m_lastD != SmartDashboard.getNumber("turretD", 0.05)){
            m_lastD = SmartDashboard.getNumber("turretD", 0.05);
            configMax.closedLoop.d(m_lastD);
            m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        SmartDashboard.putNumber("turretEnc", m_turretEnc.getPosition());

        SmartDashboard.putNumber("ShooterRPM", m_flywheelMotorLead.getVelocity().getValueAsDouble() * 60);
        double targX = Constants.c_hubY;
        double targY = m_hubX;
        if (shootingState == ShootingState.hubShoot){
            targX = Constants.c_hubY;
            targY = m_hubX;   
        }
        else if (shootingState == ShootingState.feedShoot){
            if (m_robotPose.getY() < 3.4){ // close
                targY = 2.0;
            }
            else if (m_robotPose.getY() > 4.6){ // away
                targY = 6.0;  
            }

            if (m_isBlue){
                targX = 2.3;
            }
            else {
                targX = 13.5;
            }
        }
        SmartDashboard.putNumber("targetX", targX);
        SmartDashboard.putNumber("targetY", targY);
        xDist = targX - m_robotPose.getX();   
        yDist = targY - m_robotPose.getY();

        calculateTurretAngle(xDist, yDist);        

        SmartDashboard.putNumber("ShooterDistance", m_distance);
        SmartDashboard.putNumber("TurretDegCalc", m_turretAngle * 180.0 / Math.PI);
        
        boolean isTesting = SmartDashboard.getBoolean("disableShooter", Constants.defaultFlywheel); // for reducing noise during testing
        // SmartDashboard.putBoolean("shootDisableGet", isTesting); // debugging
//        if (!isShooting && !isTesting) {
        if (!isTesting) {
            // setRPMDistance();
            setRPMDistanceAndVelo(m_ChassisSpeeds);
        }
        else {
            stopShooter();
        }
    }
    public void resetPrevDist() { m_prevDistance = 0.0; }

    public double getAngularDisplacement(Pose2d currentPose, Pose2d targetPose, Rotation2d turretAngle){
        currentPose.transformBy(new Transform2d(0.0, 0.0, Rotation2d.kZero)); // offset of robot center to turret center
        double xDisplacement = targetPose.getX() - currentPose.getX();
        double yDisplacement = targetPose.getY() - currentPose.getY();
        return Math.atan2(yDisplacement, xDisplacement) - currentPose.getRotation().getRadians() - turretAngle.getRadians();
    }

    public double getAimingRotations(double angle){
        double rotations = angle;
        return rotations; // TODO: Figure out angle to rotations
    }

    public void setRPM(double rpm){
        if (RobotBase.isReal()) {
            m_flywheelMotorLead.setControl(m_vvReq.withVelocity(rpm / 60.0));
        }
        else {
            m_simAngVel = rpm;
            m_FlywheelSim.setAngularVelocity(rpm * 2.0 * Math.PI / 60.0);
        }
    }

    public void setRPMDistance() {
        // if (!isShooting) { isShooting = true; }
        if (Math.abs(m_distance - m_prevDistance) > 0.3) {
            if (m_turretAngle > 0.0) {
                double offset = Math.PI / 2 - m_turretAngle;
                double rpmBoost = offset * 30;
                m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance) + rpmBoost) / 60.0));
            }
            else {
                m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance)) / 60.0));
            }
            m_prevDistance = m_distance;
        }
    }

    public void setRPMDistanceAndVelo(ChassisSpeeds speeds){
        double offsetDistance = m_distance;
        double offsetRPM = SmartDashboard.getNumber("offsetRPM", Constants.rpmBoost);
        for (int i = 0; i < 20; i++){   // SEC Why does this loop 20 times??
            offsetX = speeds.vxMetersPerSecond * TOFtable.get(offsetDistance);
            offsetY = speeds.vyMetersPerSecond * TOFtable.get(offsetDistance);
            offsetDistance = Math.sqrt(Math.pow(xDist - offsetX, 2) + Math.pow(yDist - offsetY, 2));;
        }
        if (m_isBlue){
            offsetX = -offsetX;
        }
        SmartDashboard.putNumber("SOTF Distance", offsetDistance);
        SmartDashboard.putNumber("SOTFX", offsetX); //SOTF stand for shooting on the fly
        SmartDashboard.putNumber("SOTFY", offsetY);

        if (Math.abs(m_distance - m_prevDistance) > 0.3) {
            if (m_turretAngle > 0.0) {
                double offset = Math.PI / 2 - m_turretAngle;
                double rpmBoost = offset * 30 + offsetRPM;
                m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance) + rpmBoost) / 60.0));
            }
            else {
                m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance) + offsetRPM) / 60.0));
            }
            m_prevDistance = m_distance;
        }
    }

    public void setMotor(double rpm){
        m_flywheelMotorLead.set(rpm);
    }

    public void stopShooter(){
        // if (isShooting) { isShooting = false; }
        if (RobotBase.isReal()){
            m_flywheelMotorLead.stopMotor();
        }
        else {
            m_simAngVel = 0.0;
            m_FlywheelSim.setAngularVelocity(0.0);
        }
    }

    public void setServo(double value){
        m_servo.set(value);
    }

    public void resetTurret() {
        // SmartDashboard.putNumber("turretRad", 0.0);
        m_turretCtlr.setSetpoint(0.0, ControlType.kPosition);
    }

    public void calculateTurretAngle(double x, double y){
        SmartDashboard.putNumber("robotToTargetX", x);
        SmartDashboard.putNumber("robotToTargetY", y);
        SmartDashboard.putNumber("OffsetX", offsetX);
        SmartDashboard.putNumber("OffsetY", offsetY);

        double robotRot = m_robotPose.getRotation().getRadians();
        SmartDashboard.putNumber("turretRobotRot0", robotRot * 180.0 / Math.PI);
        int quad = 1;

        if (x >= 0.0) {
            if (y > 0.0) {
                quad = 2;
            }
            else {
                quad = 1;
            }
        }
        else {
            if (y > 0.0) {
                quad = 3;
            }
            else {
                quad = 4;
            }
        }
        SmartDashboard.putNumber("turretQuad", quad);
        
        m_distance = Math.sqrt(Math.pow(xDist + offsetX, 2) + Math.pow(yDist + offsetY, 2));
        m_turretAngle = Math.atan2(yDist + Constants.m_turretOffsetY + offsetY, xDist + Constants.m_turretOffsetX + offsetX);
        SmartDashboard.putNumber("turretDegCalc0", m_turretAngle * 180.0 / Math.PI);

        if (quad == 1) {
            m_turretAngle = -1.0 * (m_turretAngle - robotRot);
        }
        else if (quad == 4) {
            m_turretAngle = -1.0 * ((m_turretAngle % Math.PI) - robotRot);
        }
        else { // q2 and 3
            m_turretAngle = -1.0 * ((m_turretAngle % Math.PI) + robotRot);
        }

        SmartDashboard.putNumber("turretRobotRot", robotRot * 180.0 / Math.PI);
        SmartDashboard.putNumber("turretRadCalc", m_turretAngle);

        double turns = ((m_turretAngle) * m_radToTurns);
        if (turns > m_maxTurns){
            turns = m_maxTurns;
        }
        else if (turns < -m_maxTurns){
            turns = -m_maxTurns;
        }
        SmartDashboard.putNumber("turretTurnsCalc", turns);
        if (m_moveTurret) {
            m_turretCtlr.setSetpoint(turns, ControlType.kPosition);
        }
    }

//     public void moveHood(double angle){
//         m_hoodCtlr.setSetpoint(angle, ControlType.kPosition);
//     }


/* Testing sim stuff */

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.

    private final DCMotor m_flywheelGearbox = DCMotor.getKrakenX60Foc(1);
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.createFlywheelSystem(
            m_flywheelGearbox, kFlywheelMomentOfInertia, kFlywheelGearing);

    private static final double kFlywheelMomentOfInertia = 0.00032; // kg * m^2
    private static final double kFlywheelGearing = 1.0;

    private final int kEncoderAChannel = 0;
    private final int kEncoderBChannel = 1;

    private final FlywheelSim m_FlywheelSim =
        new FlywheelSim(
            m_flywheelPlant, m_flywheelGearbox
        );

    private double m_simAngVel = 0.0; 
    static int count = 0;
    
    @Override
    public void simulationPeriodic() {
        
        if (!RobotBase.isReal()){
            m_FlywheelSim.setAngularVelocity(m_simAngVel * 2.0 * Math.PI / 60.0);
        }
        m_FlywheelSim.update(0.020);
        // if (count++ % 100 == 0)
        // {
            // SmartDashboard.putNumber("ShooterRPM", m_FlywheelSim.getAngularVelocityRadPerSec() * 60.0 / 2 * Math.PI);
        // }
    }

}