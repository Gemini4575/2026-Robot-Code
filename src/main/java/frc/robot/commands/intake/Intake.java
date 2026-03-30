package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.IntakeSubystem;

public class Intake extends Command {
    private IntakeSubystem i;

    public Intake(IntakeSubystem i) {
        this.i = i;
        addRequirements(i);
    }

    @Override
    public void execute() {
        i.Intake();
    }

    @Override
    public void end(boolean interrupted) {
        i.stopIntake();
    }

}