package frc.robot.subsystems;

import frc.robot.ConstantsCANIDS;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkMax m_deployMotorLead = new SparkMax(ConstantsCANIDS.kIntakeDeployID1, SparkMax.MotorType.kBrushless);
    private SparkMax m_deployMotorFollow = new SparkMax(ConstantsCANIDS.kIntakeDeployID2, SparkMax.MotorType.kBrushless);
    private RelativeEncoder m_deployEnc = m_deployMotorLead.getEncoder();
    private RelativeEncoder m_followEnc = m_deployMotorFollow.getEncoder();
    private SparkClosedLoopController m_deployClc = m_deployMotorLead.getClosedLoopController();
    private SparkClosedLoopController m_followClc = m_deployMotorFollow.getClosedLoopController();
    private final TalonFX m_rollerMotor = new TalonFX(ConstantsCANIDS.kIntakeRollerID);
    private double m_minRPM = 20000;
    private double m_maxRPM = 0.0;
    private boolean m_isRunning = false;
    public static final double m_home = 0.0;
    public static final double m_frame = 3.5;
    public static final double m_extend = 14.3;

    public Intake() {
        SmartDashboard.putNumber("intakeVoltage", -10);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(true)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
            .pid(0.04, 0.0, 0.0);
        m_deployMotorLead.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        config.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
            .pid(0.04, 0.0, 0.0);
        m_deployMotorFollow.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_deployEnc.setPosition(0.0);
        m_followEnc.setPosition(0.0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // TODO figure out gear ratio
        
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        
        Slot0Configs slot0 = cfg.Slot0;
       SmartDashboard.putNumber("rollMot1RPM", 60 * m_rollerMotor.getVelocity(true).getValueAsDouble());
       SmartDashboard.putNumber("maxRPM", m_maxRPM);
       SmartDashboard.putNumber("minRPM", m_minRPM);
        
       SmartDashboard.putBoolean("isRunning", m_isRunning);
       if (m_isRunning) { // doesn't work for min?
            double rpm = 60 * m_rollerMotor.getVelocity(true).getValueAsDouble();
            if (rpm > m_maxRPM) {
                m_maxRPM = rpm;
            }
            if (rpm < m_minRPM) {
                m_minRPM = rpm;
            }
        }
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_rollerMotor.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

   @Override
    public void periodic() {
        SmartDashboard.putNumber("DeployLeadRotations", m_deployEnc.getPosition());
        SmartDashboard.putNumber("DeployFollowRotations", m_followEnc.getPosition());
        double rpm = 60 * m_rollerMotor.getVelocity(true).getValueAsDouble();
        SmartDashboard.putNumber("rollMot1RPM", rpm);
        SmartDashboard.putNumber("maxRPM", m_maxRPM);
        SmartDashboard.putNumber("minRPM", m_minRPM);
        SmartDashboard.putBoolean("isRunning", m_isRunning);

        if (m_isRunning) { // doesn't work for min?
            if (rpm > m_maxRPM) {
                m_maxRPM = rpm;
            }
            if (rpm < m_minRPM) {
                m_minRPM = rpm;
            }
        }
    }
    
    public void deploy(double pos) {
        m_deployClc.setSetpoint(pos, ControlType.kPosition);
        m_followClc.setSetpoint(pos, ControlType.kPosition);
    }

    public void retract() {
        m_deployClc.setSetpoint(0, ControlType.kPosition);
        m_followClc.setSetpoint(0, ControlType.kPosition);
    }

    public void runIntake() {
        m_isRunning = true;
        m_minRPM = 20000.0;
        m_maxRPM = 0.0;

        double volt = SmartDashboard.getNumber("intakeVoltage", -10);
        m_rollerMotor.setVoltage(-volt);
        
    }
    
    public void stopIntake() {
        m_isRunning = false;
        m_rollerMotor.setVoltage(0);
    }
}
