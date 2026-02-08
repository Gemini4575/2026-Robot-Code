package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topDeck.IntakeSubystem;

public class Intake extends Command{
    private IntakeSubystem i;
    private BooleanSupplier Intake;
    private BooleanSupplier Outake;
    private BooleanSupplier Extend;
    private BooleanSupplier Retract;
    
    public Intake(IntakeSubystem i, BooleanSupplier Intake, BooleanSupplier Outake, BooleanSupplier Extend, BooleanSupplier Retract) {
        this.i = i;
        this.Intake = Intake;
        this.Outake = Outake;
        this.Extend = Extend;
        this.Retract = Retract;
        addRequirements(i);
    }

    @Override
    public void execute() {
        if (Intake.getAsBoolean()) {
            i.Intake();
        }
        else if (Outake.getAsBoolean()) {
            i.Outake();
        }
        else if (Extend.getAsBoolean()) {
            i.extendIntake();
        }
        else if (Retract.getAsBoolean()) {
            i.retractIntake();
        }
        else {
            i.stopIntake();
            i.stopSliders();
        }
    }

}