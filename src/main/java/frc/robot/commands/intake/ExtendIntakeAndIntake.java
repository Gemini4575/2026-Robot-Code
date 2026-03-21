package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.IntakeSubystem;

public class ExtendIntakeAndIntake extends Command {
    private final IntakeSubystem i;

    public ExtendIntakeAndIntake(IntakeSubystem ii) {
        i = ii;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        i.MoveDownToIntake();
    }

    @Override
    public void execute() {
        i.Intake();
    }

    @Override
    public void end(boolean iii) {
        i.stopIntake();
    }
}
