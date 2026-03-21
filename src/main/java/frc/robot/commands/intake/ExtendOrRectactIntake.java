package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.IntakeSubystem;

public class ExtendOrRectactIntake extends Command {
    private final IntakeSubystem i;
    private final BooleanSupplier extend;
    private final BooleanSupplier retract;
    private final BooleanSupplier intake;

    public ExtendOrRectactIntake(IntakeSubystem ii, BooleanSupplier extend, BooleanSupplier retract,
            BooleanSupplier intake) {
        i = ii;
        addRequirements(i);
        this.extend = extend;
        this.retract = retract;
        this.intake = intake;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (extend.getAsBoolean()) {
            i.MoveDownToIntake();
        } else if (retract.getAsBoolean()) {
            i.MoveUpToStore();
        } else if (intake.getAsBoolean()) {
            i.Intake();
        } else {
            i.stopIntake();
        }
    }
}
