package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topDeck.IntakeSubystem;

public class Intake extends Command{
    private IntakeSubystem i;
    
    public Intake(IntakeSubystem i) {
        this.i = i;
        addRequirements(i);
    }

    @Override
    public void execute() {
        i.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        i.stopIntake();
    }
}