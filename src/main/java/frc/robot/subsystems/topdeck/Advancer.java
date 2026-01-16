package frc.robot.subsystems.topdeck;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AdvancerConstants.*;

public class Advancer extends SubsystemBase {
    private final SparkMax AdavancerMotor;

    public Advancer() {
        AdavancerMotor = new SparkMax(ADVANCER_MOTOR_ID, MotorType.kBrushless);
        SparkBaseConfig AdvancerMotorConfig = new SparkMaxConfig();
        AdvancerMotorConfig.smartCurrentLimit(40, 40);
        AdvancerMotorConfig.disableFollowerMode();

        AdvancerMotorConfig.idleMode(IdleMode.kCoast);

        AdvancerMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
        AdvancerMotorConfig.signals.primaryEncoderPositionPeriodMs(5);

        AdavancerMotor.configure(AdvancerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}
