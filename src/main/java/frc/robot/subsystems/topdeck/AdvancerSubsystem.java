package frc.robot.subsystems.topdeck;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AdvancerConstants.*;

public class AdvancerSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry maxspeedEntry = tab
            .add("Max Speed Advancer", 1.0)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .getEntry();
    private final TalonFX AdavancerMotor;
    private final SparkFlex RollerMotor;
    private final SparkMax NeoAdvancerMotor;

    public AdvancerSubsystem() {
        RollerMotor = new SparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
        NeoAdvancerMotor = new SparkMax(NEO_ADVANCER_MOTOR_ID, MotorType.kBrushless);
        AdavancerMotor = new TalonFX(ADVANCER_MOTOR_ID);
        TalonFXConfiguration AdvancerMotorConfig = new TalonFXConfiguration();
        SparkFlexConfig rollerMotorConfig = new SparkFlexConfig();
        SparkMaxConfig AdvancerMotorConfigSpark = new SparkMaxConfig();
        SparkMaxConfig NeoAdvancerMotorConfigSpark = new SparkMaxConfig(); // STUFF CASSIE ADDED

        AdvancerMotorConfigSpark.smartCurrentLimit(30, 30);
        AdvancerMotorConfigSpark.softLimit.forwardSoftLimitEnabled(false);
        AdvancerMotorConfigSpark.softLimit.reverseSoftLimitEnabled(false);
        AdvancerMotorConfigSpark.idleMode(IdleMode.kBrake);

        NeoAdvancerMotor.configure(AdvancerMotorConfigSpark, ResetMode.kNoResetSafeParameters, // STUFF CASSIE REMOVED
                PersistMode.kPersistParameters);

        // NeoAdvancerMotorConfigSpark.smartCurrentLimit(30, 30); // STUFF CASSIE ADDED
        // NeoAdvancerMotorConfigSpark.softLimit.forwardSoftLimitEnabled(false); //
        // STUFF CASSIE ADDED
        // NeoAdvancerMotorConfigSpark.softLimit.reverseSoftLimitEnabled(false); //
        // STUFF CASSIE ADDED
        // NeoAdvancerMotorConfigSpark.idleMode(IdleMode.kBrake); // STUFF CASSIE ADDED

        rollerMotorConfig.disableFollowerMode();
        rollerMotorConfig.smartCurrentLimit(30, 30);
        rollerMotorConfig.idleMode(IdleMode.kBrake);

        AdvancerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        AdvancerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;

        AdvancerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        AdavancerMotor.getConfigurator().apply(AdvancerMotorConfig);

    }

    public void reverse() {
        AdavancerMotor.set(ADVANCER_SPEED);
        NeoAdvancerMotor.set(NEO_ADVANCER_SPEED);
        RollerMotor.set(ADVANCER_ROLLER_SPEED);
    }

    public void advancerOnlyReverse() {
        // AdavancerMotor.set(ADVANCER_SPEED);
        // NeoAdvancerMotor.set(NEO_ADVANCER_SPEED);
    }

    public void stopAdvancer() {
        AdavancerMotor.set(0);
        NeoAdvancerMotor.set(0);
        RollerMotor.set(0);
    }

    public void advance() {
        AdavancerMotor.set(-ADVANCER_SPEED);
        NeoAdvancerMotor.set(-NEO_ADVANCER_SPEED);
        RollerMotor.set(-ADVANCER_ROLLER_SPEED);
    }
}
