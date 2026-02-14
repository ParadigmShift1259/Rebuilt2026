package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ConstantsCANIDS;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Transfer extends SubsystemBase {
    private SparkMax m_spinDex = new SparkMax(ConstantsCANIDS.kSpindexerID, SparkMax.MotorType.kBrushless);
    private SparkFlex m_feederMot = new SparkFlex(ConstantsCANIDS.kFeederID, SparkFlex.MotorType.kBrushless);
    // private SparkClosedLoopController m_spinDexCtlr = m_spinDex.getClosedLoopController();
    private SparkClosedLoopController m_feederMotCtlr = m_feederMot.getClosedLoopController();

    public Transfer(){
        SparkMaxConfig configMax = new SparkMaxConfig();
        configMax.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0);
        m_spinDex.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkFlexConfig configFlex = new SparkFlexConfig();
        configFlex.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                        .p(0.2);
        m_feederMot.configure(configFlex, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpinDexSpeed(double speed){
        m_spinDex.set(speed);
    }

    public void setFeederSpeed(double speed){
        m_feederMotCtlr.setSetpoint(speed, ControlType.kVelocity);
    }

    public void stopSpinDex(){
        m_spinDex.stopMotor();
    }

    public void stopFeeder(){
        m_feederMot.stopMotor();
    }
}
