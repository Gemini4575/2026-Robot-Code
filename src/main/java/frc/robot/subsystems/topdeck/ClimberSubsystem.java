package frc.robot.subsystems.topdeck;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CimberConstants.*;

public class ClimberSubsystem extends SubsystemBase{
    private ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    private GenericEntry climberPositionEntry = climberTab.add("CLimber Motor Position", 0)
        .getEntry();
    private GenericEntry climberSetPointEntry = climberTab.add("Climber Reached Set Point", false)
    .getEntry();
    private final SparkMax climberMotor;
    public ClimberSubsystem() {
        climberMotor = new SparkMax(CLIMBER_MOTOR_CANID, MotorType.kBrushless);
        climberMotor.getEncoder().setPosition(0);
        SparkBaseConfig CLimberMotorConfig = new SparkMaxConfig();
        CLimberMotorConfig.smartCurrentLimit(40, 40);
        CLimberMotorConfig.disableFollowerMode();

        CLimberMotorConfig.idleMode(IdleMode.kBrake);

        CLimberMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
        CLimberMotorConfig.signals.primaryEncoderPositionPeriodMs(5);

        climberMotor.configure(CLimberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void JoystickControl(double joy){
        climberMotor.set(joy);
    }

    private boolean IMoveDownToClimbI(){
        climberMotor.set(1);

        return climberMotor.getEncoder().getPosition() >= Climber_Down_SetPoint;
    }

    public boolean MoveDownToClimb(){
        if(IMoveDownToClimbI()){
            stop();
            return true;
        }
        return false;
    }

    public void stop(){
        climberMotor.stopMotor();
    }

    @Override
    public void periodic(){
        climberPositionEntry.setDouble(climberMotor.getEncoder().getPosition());
        climberSetPointEntry.setBoolean(climberMotor.getEncoder().getPosition() >= Climber_Down_SetPoint);
    }
}
