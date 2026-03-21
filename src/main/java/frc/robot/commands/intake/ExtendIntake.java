package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ClimberSubsystem;
import frc.robot.subsystems.topdeck.IntakeSubystem;

public class ExtendIntake extends Command {
    private IntakeSubystem climber;

    public ExtendIntake(IntakeSubystem climberSubsystem) {
        this.climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climber.MoveDownToIntake();
    }

    @Override
    public boolean isFinished() {
        return climber.MoveDownToIntake();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

}
