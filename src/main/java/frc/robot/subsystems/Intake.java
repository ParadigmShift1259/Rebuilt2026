package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    private SparkMax m_deployMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);
    private RelativeEncoder m_deployEnc = m_deployMotor.getEncoder();
    private SparkClosedLoopController m_deployClc = m_deployMotor.getClosedLoopController();

    public Intake(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_deployMotor.configure(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
        m_deployEnc.setPosition(0.0);
    }
}
