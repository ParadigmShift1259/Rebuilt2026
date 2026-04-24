package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ConstantsCANIDS;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;

@Logged
public class Transfer extends SubsystemBase {

    private final double defaultSpinSpeed = -0.75;

    private SparkMax m_spinDex = new SparkMax(ConstantsCANIDS.kSpindexerID, SparkMax.MotorType.kBrushless);
    private TalonFX m_kickerMotor = new TalonFX(ConstantsCANIDS.kFeederID);

    public Transfer(){
        SmartDashboard.putNumber("spinSpeed", defaultSpinSpeed);
        SparkMaxConfig configMax = new SparkMaxConfig();
        configMax.idleMode(SparkMaxConfig.IdleMode.kCoast)
                 .inverted(false)
                 .smartCurrentLimit(20, 40);
        m_spinDex.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(50))
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(Amps.of(30))
                .withSupplyCurrentLowerTime(Seconds.of(1))
        );
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_kickerMotor.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure kicker motor. Error: " + status.toString());
        }
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
