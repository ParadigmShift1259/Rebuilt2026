package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.ConstantsCANIDS;

import static edu.wpi.first.units.Units.*;

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

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;


@Logged
public class Shooter extends SubsystemBase {
    private final TalonFX m_flywheelMotorLead = new TalonFX(ConstantsCANIDS.kFlywheelLeadID);
    private final TalonFX m_flywheelMotorFollow = new TalonFX(ConstantsCANIDS.kFlywheelFollowID);
    private final VelocityVoltage m_vvReq = new VelocityVoltage(0).withSlot(0);

    InterpolatingDoubleTreeMap RPMtable = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap TOFtable = new InterpolatingDoubleTreeMap();

    private Servo m_servo = new Servo(0);

    private SparkMax m_turretMot = new SparkMax(ConstantsCANIDS.kTurretID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_turretCtlr = m_turretMot.getClosedLoopController();
    
    private SparkMax m_hoodMot = new SparkMax(ConstantsCANIDS.kHoodID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_hoodCtlr = m_hoodMot.getClosedLoopController();

    public Shooter(){
        // Add calibration points (distance in meters -> shooter RPM)
        RPMtable.put(2.6, 1850.0);
        RPMtable.put(3.0, 1900.0);
        RPMtable.put(3.5, 2000.0);
        RPMtable.put(4.0, 2100.0);
        RPMtable.put(4.5, 2200.0);
        RPMtable.put(5.0, 2300.0);
        RPMtable.put(5.5, 2400.0);
        RPMtable.put(6.0, 2500.0);

        TOFtable.put(5.68, 1.16);
        TOFtable.put(4.55, 1.12);
        TOFtable.put(3.15, 1.11);
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
        slot0.kD = 0;

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

        SparkMaxConfig configMax = new SparkMaxConfig();
        configMax.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                       .p(0.5);
        // m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        // configMax.closedLoop.p(0.5);
        // m_hoodMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterRPM", m_flywheelMotorLead.getVelocity().getValueAsDouble() * 60);
    }

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

    public void setRPMDistance(double distance){
        m_flywheelMotorLead.setControl(m_vvReq.withVelocity(RPMtable.get(distance)/ 60.0));
    }

    public void setRPMDistanceAndVelo(double distance, ChassisSpeeds speeds){
        double m_distance = distance;
        double offsetX = 0.0;
        double offsetY = 0.0;
        for (int i = 0; i < 20; i++){
            offsetX = speeds.vxMetersPerSecond * TOFtable.get(distance);
            offsetY = speeds.vyMetersPerSecond * TOFtable.get(distance);
            distance = distance + Math.sqrt(Math.pow(offsetX, 2) + Math.pow(offsetX, 2));
        }
        m_flywheelMotorLead.setControl(m_vvReq.withVelocity(RPMtable.get(distance)/ 60.0));
    }

    public void setMotor(double rpm){
        m_flywheelMotorLead.set(rpm);
    }

    public void stopShooter(){
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

//     public void aimTurret(double angle){
//         m_turretCtlr.setSetpoint(getAimingRotations(angle), ControlType.kPosition);
//     }

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