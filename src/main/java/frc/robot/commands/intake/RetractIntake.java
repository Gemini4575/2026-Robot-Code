package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.IntakeSubystem;

public class RetractIntake extends Command {
    private IntakeSubystem climber;

    public RetractIntake(IntakeSubystem climberSubsystem) {
        this.climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climber.MoveUpToStore();
    }

    @Override
    public boolean isFinished() {
        return climber.MoveUpToStore();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

}
