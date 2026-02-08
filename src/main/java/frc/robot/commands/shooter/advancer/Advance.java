package frc.robot.commands.shooter.advancer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topDeck.AdvancerSubsystem;

public class Advance extends Command {
    private final AdvancerSubsystem advancer;

    public Advance(AdvancerSubsystem advancerSubsystem) {
        this.advancer = advancerSubsystem;
        addRequirements(advancerSubsystem);
    }

    @Override
    public void execute() {
        advancer.advance();
    }

    @Override
    public boolean isFinished() {
        advancer.stopAdvancer();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        advancer.stopAdvancer();
    }

    @Override
    public void initialize() {
    }

}
