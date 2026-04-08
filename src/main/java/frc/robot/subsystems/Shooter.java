package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ConstantsCANIDS;
import frc.robot.Geofencing;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

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

@Logged
public class Shooter extends SubsystemBase {
    private final TalonFX m_flywheelMotorLead = new TalonFX(ConstantsCANIDS.kFlywheelLeadID);
    private final TalonFX m_flywheelMotorFollow = new TalonFX(ConstantsCANIDS.kFlywheelFollowID);
    private final VelocityVoltage m_vvReq = new VelocityVoltage(0).withSlot(0);

    private InterpolatingDoubleTreeMap RPMtable = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap TOFtable = new InterpolatingDoubleTreeMap();

    private SparkMax m_turretMot = new SparkMax(ConstantsCANIDS.kTurretID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_turretCtlr = m_turretMot.getClosedLoopController();
    private RelativeEncoder m_turretEnc = m_turretMot.getEncoder();
    SparkMaxConfig configMax = new SparkMaxConfig();

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
    private static double m_radToTurns = 12.2 / (Math.PI / 2);

    private Pose2d m_robotPose = Pose2d.kZero;
    private ChassisSpeeds m_ChassisSpeeds = new ChassisSpeeds();
    private Geofencing m_geofenceNeutZone;
    private boolean m_isBlue = false;
    public boolean m_moveTurret = true; // for sim only
    // public boolean m_moveTurret = false;
    private double m_lastTurns = 0;

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
        fdb.SensorToMechanismRatio = 1;
        
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
                       .p(0.15);
                    //    .d(0.02);
        m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SmartDashboard.putNumber("turretP", 0.15);
        SmartDashboard.putNumber("turretD", 0.1);
        SmartDashboard.putNumber("NegXVelOffsetDegrade", Constants.m_defaultNegXVelOffsetDegrade);
        SmartDashboard.putNumber("NegYVelOffsetDegrade", Constants.m_defaultNegYVelOffsetDegrade);
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

        // if (m_lastP != SmartDashboard.getNumber("turretP", 0.1)){
        //     m_lastP = SmartDashboard.getNumber("turretP", 0.1);
        //     configMax.closedLoop.p(m_lastP);
        //     m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // }
        // if (m_lastD != SmartDashboard.getNumber("turretD", 0.05)){
        //     m_lastD = SmartDashboard.getNumber("turretD", 0.05);
        //     configMax.closedLoop.d(m_lastD);
        //     m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // }
        SmartDashboard.putNumber("turretEnc", m_turretEnc.getPosition());

        SmartDashboard.putNumber("ShooterRPM", m_flywheelMotorLead.getVelocity().getValueAsDouble() * 60);
        double targX = m_hubX;
        double targY = Constants.c_hubY;
        if (shootingState == ShootingState.hubShoot){
            targX = m_hubX;   
            targY = Constants.c_hubY;
        }
        else if (shootingState == ShootingState.feedShoot){
            if (m_robotPose.getY() < 4.0){ // close
                targY = 2.0;
            }
            else {
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
        if (!isTesting) {
            setRPMDistanceAndVelo(m_ChassisSpeeds);
        }
        else {
            stopShooter();
        }
    }

    public void resetPrevDist() { m_prevDistance = 0.0; }

    public void setRPM(double rpm){
        //if (RobotBase.isReal()) {
            m_flywheelMotorLead.setControl(m_vvReq.withVelocity(rpm / 60.0));
        //}
        //else {
        //   m_simAngVel = rpm;
        //    m_FlywheelSim.setAngularVelocity(rpm * 2.0 * Math.PI / 60.0);
        //}
    }

    // public void setRPMDistance() {
    //     if (Math.abs(m_distance - m_prevDistance) > 0.3) {
    //         if (m_turretAngle > 0.0) {
    //             double offset = Math.PI / 2 - m_turretAngle;
    //             double rpmBoost = offset * 0;
    //             m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance) + rpmBoost) / 60.0));
    //         }
    //         else {
    //             m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance)) / 60.0));
    //         }
    //         m_prevDistance = m_distance;
    //     }
    // }

    public void setRPMDistanceAndVelo(ChassisSpeeds speeds){
        double offsetDistance = m_distance;
        double offsetRPM = SmartDashboard.getNumber("offsetRPM", Constants.rpmBoost);
        for (int i = 0; i < 20; i++){   // Loop 20 times to let the algorithm converge
            offsetX = speeds.vxMetersPerSecond * TOFtable.get(offsetDistance);
            offsetY = speeds.vyMetersPerSecond * TOFtable.get(offsetDistance);
            if (offsetX < 0.0 && offsetY < 0.0){
                // This one is for backing towards the outpost
                //offsetX *= SmartDashboard.getNumber("NegXVelOffsetDegrade", Constants.m_defaultNegXVelOffsetDegrade);
                offsetY *= SmartDashboard.getNumber("NegYVelOffsetDegrade", Constants.m_defaultNegYVelOffsetDegrade);
            }
            if (offsetY < 0.0){
                // This one is for right to left
                offsetY *= SmartDashboard.getNumber("NegYVelOffsetDegrade", Constants.m_defaultNegYVelOffsetDegrade);
            }
            offsetDistance = Math.sqrt(Math.pow(xDist - offsetX, 2) + Math.pow(yDist - offsetY, 2));
        }
        // if (m_isBlue){
        //     offsetX = -offsetX;
        // }
        SmartDashboard.putNumber("SOTF Distance", offsetDistance);
        SmartDashboard.putNumber("SOTFX", offsetX); //SOTF stand for shooting on the fly
        SmartDashboard.putNumber("SOTFY", offsetY);

        // Making sure the shots dont fall off with repeated pid calls
        if (Math.abs(m_distance - m_prevDistance) > 0.3) {
            m_flywheelMotorLead.setControl(m_vvReq.withVelocity((RPMtable.get(m_distance) + offsetRPM) / 60.0));
            m_prevDistance = m_distance;
        }
    }

    public void setMotor(double rpm){
        m_flywheelMotorLead.set(rpm);
    }

    public void stopShooter(){
        // if (RobotBase.isReal()){
            m_flywheelMotorLead.stopMotor();
        // }
        // else {
        //     m_simAngVel = 0.0;
        //     m_FlywheelSim.setAngularVelocity(0.0);
        // }
    }

    public void resetTurret() {
        m_turretCtlr.setSetpoint(0.0, ControlType.kPosition);
    }

    public void calculateTurretAngle(double x, double y) {
        SmartDashboard.putNumber("robotToTargetX", x);
        SmartDashboard.putNumber("robotToTargetY", y);
        SmartDashboard.putNumber("OffsetX", offsetX);
        SmartDashboard.putNumber("OffsetY", offsetY);

        double robotFieldRot = m_robotPose.getRotation().getRadians();
        double robotRot = robotFieldRot;
        SmartDashboard.putNumber("turretRobotRot0", robotRot * 180.0 / Math.PI);
        
        m_distance = Math.sqrt(Math.pow(xDist - offsetX, 2) + Math.pow(yDist - offsetY, 2));
        double robotToTargetAngle = Math.atan2(yDist + Constants.m_turretOffsetY - offsetY, xDist + Constants.m_turretOffsetX - offsetX);
        m_turretAngle = robotToTargetAngle;
        SmartDashboard.putNumber("turretDegCalc0", m_turretAngle * 180.0 / Math.PI);

        boolean bRedHubBlueFeed =  ((!m_isBlue && shootingState == ShootingState.hubShoot)
                                 || ( m_isBlue && shootingState == ShootingState.feedShoot));

        boolean bBlueHubRedFeed  = (( m_isBlue && shootingState == ShootingState.hubShoot)
                                ||  (!m_isBlue && shootingState == ShootingState.feedShoot));

        if (bRedHubBlueFeed) {
            // Invert axes
            if (robotRot < 0.0) {
                robotRot += Math.PI;
            }
            else if (robotRot > 0.0) {
                robotRot -= Math.PI;
            }

            // Do not need for 360 turret
            // if (m_turretAngle < -Constants.m_turretLimitAngle) {
            //     m_turretAngle += Math.PI;
            // }
            // else if (m_turretAngle > Constants.m_turretLimitAngle) {
            //     m_turretAngle -= Math.PI;
            // }
        }
        // else if (bBlueHubRedFeed) {
        //     // Do not need for 360 turret
        //     if (m_turretAngle - robotRot < -Constants.m_turretLimitAngle) {
        //         m_turretAngle += Math.PI;
        //     }
        //     else if (m_turretAngle - robotRot > Constants.m_turretLimitAngle) {
        //         m_turretAngle -= Math.PI;
        //     }
        // }

        m_turretAngle = -1.0 * (m_turretAngle - robotRot) + Constants.m_turretZeroAngle;

        SmartDashboard.putNumber("turretRobotRot", robotRot * 180.0 / Math.PI);
        SmartDashboard.putNumber("turretRadCalc", m_turretAngle);

        SmartDashboard.putNumber("robotFieldRot", robotFieldRot * 180.0 / Math.PI);
        SmartDashboard.putNumber("robotToTargetAngle", robotToTargetAngle * 180.0 / Math.PI);
        // boolean bTurretFacingZero = (Math.abs(m_turretAngle - robotFieldRot) < Constants.m_turretLimitAngle);
        // boolean bTurrentFacing180 = (Math.abs(m_turretAngle - robotFieldRot) > Constants.m_turretLimitAngle);
        // boolean bDisallowTurniungZero = bTurretFacingZero         // Red Hub and Blue Feed
        //                             && ((!m_isBlue && shootingState == ShootingState.hubShoot)
        //                              || ( m_isBlue && shootingState == ShootingState.feedShoot));

        // boolean bDisallowTurniung180  = bTurrentFacing180         // Blue Hub and Red Feed
        //                              && (( m_isBlue && shootingState == ShootingState.hubShoot)
        //                              ||  (!m_isBlue && shootingState == ShootingState.feedShoot));

        // SmartDashboard.putBoolean("bDisallowTurniungZero", bDisallowTurniungZero);
        // SmartDashboard.putBoolean("bDisallowTurniung180", bDisallowTurniung180);
        // if (m_moveTurret && !(bDisallowTurniungZero || bDisallowTurniung180)) {
        if (m_moveTurret) {
            double turns = ((m_turretAngle) * m_radToTurns);
            if (turns > Constants.m_maxTurnsPos){
                turns = Constants.m_maxTurnsPos;
            }
            else if (turns < Constants.m_maxTurnsNeg){
                turns = Constants.m_maxTurnsNeg;
            }
            m_lastTurns = turns;
            m_turretCtlr.setSetpoint(turns, ControlType.kPosition);
        }
        SmartDashboard.putNumber("turretTurnsCalc", m_lastTurns);
    }

/* Testing sim stuff */

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
/************************************************
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
************************************************/

}