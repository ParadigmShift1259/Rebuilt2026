package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkMax m_deployMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);
    private RelativeEncoder m_deployEnc = m_deployMotor.getEncoder();
    private SparkClosedLoopController m_deployClc = m_deployMotor.getClosedLoopController();
    private final TalonFX m_rollerMotor = new TalonFX(11);
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_deployMotor.configure(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
        m_deployEnc.setPosition(0.0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // TODO figure out gear ratio
        
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        
        Slot0Configs slot0 = cfg.Slot0;
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

    public void deploy() {
        m_deployClc.setSetpoint(0, ControlType.kPosition); // TODO figure out position
    }

    public void retract() {
        m_deployClc.setSetpoint(0, ControlType.kPosition); // TODO figure out position
    }

    public void runIntake() {
        m_rollerMotor.setControl(m_mmReq.withPosition(10).withSlot(0));
        m_rollerMotor.setPosition(Rotations.of(1)); // TODO figure out rot/sec
    }
    
    public void stopIntake() {
        m_rollerMotor.setControl(m_mmReq.withPosition(0).withSlot(0));
        m_rollerMotor.setPosition(Rotations.of(0));
    }
}
