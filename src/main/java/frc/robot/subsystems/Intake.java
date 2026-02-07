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
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkMax m_deployMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);
    private RelativeEncoder m_deployEnc = m_deployMotor.getEncoder();
    private SparkClosedLoopController m_deployClc = m_deployMotor.getClosedLoopController();
    //private final TalonFX m_rollerMotor = new TalonFX(4);
    private SparkMax m_rollerMotor = new SparkMax(4, SparkMax.MotorType.kBrushless);

    public Intake() {
        SmartDashboard.putNumber("intakeVoltage", 6);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_deployMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_deployEnc.setPosition(0.0);

        config.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void deploy() {
        m_deployClc.setSetpoint(0, ControlType.kPosition); // TODO figure out position
    }

    public void retract() {
        m_deployClc.setSetpoint(0, ControlType.kPosition); // TODO figure out position
    }

    public void runIntake() {
        double volt = SmartDashboard.getNumber("intakeVoltage", 6);
        m_rollerMotor.setVoltage(volt);
        // m_rollerMotor.setPosition(Rotations.of(1)); // TODO figure out rot/sec
    }
    
    public void stopIntake() {
        m_rollerMotor.setVoltage(0);
        // m_rollerMotor.setControl(m_mmReq.withPosition(0).withSlot(0));
        // m_rollerMotor.setPosition(Rotations.of(0));
    }
}
