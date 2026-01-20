package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Transfer extends SubsystemBase {
    private SparkMax m_rollerMotor = new SparkMax(31, SparkMax.MotorType.kBrushless);
    private SparkMax m_mecanumMotor = new SparkMax(32, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController m_rollerClc = m_rollerMotor.getClosedLoopController();
    private SparkClosedLoopController m_mecanumClc = m_mecanumMotor.getClosedLoopController();

    public Transfer(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_mecanumMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setRollerSpeed(double speed){
        m_rollerMotor.set(speed);
    }

    public void setmecanumSpeed(double speed){
        m_mecanumMotor.set(speed);
    }

    public void stopRoller(){
        m_rollerMotor.stopMotor();
    }

    public void stopMecanum(){
        m_mecanumMotor.stopMotor();
    }
}
