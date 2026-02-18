package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ConstantsCANIDS;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;


@Logged
public class Shooter extends SubsystemBase {
    private final TalonFX m_flywheelMotorLead = new TalonFX(ConstantsCANIDS.kFlywheelLeadID);
    private final TalonFX m_flywheelMotorFollow = new TalonFX(ConstantsCANIDS.kFlywheelFollowID);
    private final VelocityVoltage m_vvReq = new VelocityVoltage(0).withSlot(0);

    private Servo m_servo = new Servo(0);

    private SparkMax m_turretMot = new SparkMax(ConstantsCANIDS.kTurretID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_turretCtlr = m_turretMot.getClosedLoopController();
    
    private SparkMax m_hoodMot = new SparkMax(ConstantsCANIDS.kHoodID, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_hoodCtlr = m_hoodMot.getClosedLoopController();

    public Shooter(){
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
        m_flywheelMotorLead.setControl(m_vvReq.withVelocity(rpm / 60.0));
    }

    public void setMotor(double rpm){
        m_flywheelMotorLead.set(rpm);
    }

    public void stopShooter(){
        m_flywheelMotorLead.stopMotor();
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
}
