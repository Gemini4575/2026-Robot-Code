package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.IntakeSubystem;

public class ExtendOrRectactIntake extends Command {
    private final IntakeSubystem i;
    private final AdvancerSubsystem a;
    private final BooleanSupplier extend;
    private final BooleanSupplier retract;
    private final BooleanSupplier intake;

    public ExtendOrRectactIntake(IntakeSubystem ii, AdvancerSubsystem a, BooleanSupplier extend,
            BooleanSupplier retract,
            BooleanSupplier intake) {
        i = ii;
        this.a = a;
        addRequirements(i);
        addRequirements(a);
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
            a.advancerOnlyReverse();
            i.Intake();
        } else if (retract.getAsBoolean()) {
            i.MoveUpToStore();
        } else if (intake.getAsBoolean()) {
            i.Outake();
        } else {
            i.stopIntake();
            a.stopAdvancer();
        }

    }
}
