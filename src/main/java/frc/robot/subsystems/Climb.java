package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private SparkFlex m_motor1 = new SparkFlex(11, SparkLowLevel.MotorType.kBrushless);
    private SparkFlex m_motor2 = new SparkFlex(12, SparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder m_enc1 = m_motor1.getEncoder();
    private RelativeEncoder m_enc2 = m_motor2.getEncoder();
    private SparkClosedLoopController m_clc1 = m_motor1.getClosedLoopController();
    private SparkClosedLoopController m_clc2 = m_motor2.getClosedLoopController();
    private enum ClimbLevel {L1, L2, L3}; //LEVEL Priorities
    
    public Climb () {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_motor1.configure(config, SparkFlex.ResetMode.kNoResetSafeParameters, SparkFlex.PersistMode.kNoPersistParameters);
        m_enc1.setPosition(0.0);
        m_enc2.setPosition(0.0);
    }

    public void deploy(ClimbLevel level) {
        m_clc1.setSetpoint(0, ControlType.kPosition);
        m_clc2.setSetpoint(0, ControlType.kPosition);
    }

    public void retract(ClimbLevel level) {
        m_clc1.setSetpoint(0, ControlType.kPosition);
        m_clc2.setSetpoint(0, ControlType.kPosition);
    }

    public void goToPosition(double position) {
        m_clc1.setSetpoint(position, ControlType.kPosition);
        m_clc2.setSetpoint(position, ControlType.kPosition);
    }

    public void goToPositionRel(double position) {
        goToPosition(m_enc1.getPosition() + position);
    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();   
    }
}
