package frc.robot.subsystems.topDeck;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AdvancerConstants.*;

public class AdvancerSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry maxspeedEntry = tab
            .add("Max Speed Advancer", 1)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .getEntry();
    private final SparkMax AdavancerMotor;

    public AdvancerSubsystem() {
        AdavancerMotor = new SparkMax(ADVANCER_MOTOR_ID, MotorType.kBrushless);
        SparkBaseConfig AdvancerMotorConfig = new SparkMaxConfig();
        AdvancerMotorConfig.smartCurrentLimit(40, 40);
        AdvancerMotorConfig.disableFollowerMode();

        AdvancerMotorConfig.idleMode(IdleMode.kBrake);

        AdvancerMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
        AdvancerMotorConfig.signals.primaryEncoderPositionPeriodMs(5);

        AdavancerMotor.configure(AdvancerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reverse() {
        AdavancerMotor.set(-ADVANCER_SPEED);
    }

    public void stopAdvancer() {
        AdavancerMotor.stopMotor();
    }

    public void advance() {
        AdavancerMotor.set(ADVANCER_SPEED);
        AdavancerMotor.set(ADVANCER_SPEED);
    }
}
