package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;

@Logged
public class Transfer extends SubsystemBase {

    private final double defaultSpinSpeed = -0.55;

    private SparkMax m_spinDex = new SparkMax(ConstantsCANIDS.kSpindexerID, SparkMax.MotorType.kBrushless);
    private TalonFX m_kickerMotor = new TalonFX(ConstantsCANIDS.kFeederID);

    public Transfer(){
        SmartDashboard.putNumber("spinSpeed", defaultSpinSpeed);
        SparkMaxConfig configMax = new SparkMaxConfig();
        configMax.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false);
        m_spinDex.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkFlexConfig configFlex = new SparkFlexConfig();
        configFlex.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                        .p(0.2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("SpindexerRPM", m_spinDex.getEncoder().getVelocity());
    }

    public void setSpinDexSpeed(boolean reverse){
        double speed = SmartDashboard.getNumber("spinSpeed", defaultSpinSpeed);
        m_spinDex.set(speed * (reverse ? -1.0 : 1.0));
    }

    public void setFeederSpeed(double speed){
        m_kickerMotor.set(speed);
    }

    public void stopSpinDex(){
        m_spinDex.stopMotor();
    }

    public void stopFeeder(){
        m_kickerMotor.stopMotor();
    }
}
