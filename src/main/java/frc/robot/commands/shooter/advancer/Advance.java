package frc.robot.commands.shooter.advancer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;

public class Advance extends Command {
    private final Timer timer = new Timer();
    private final AdvancerSubsystem advancer;

    public Advance(AdvancerSubsystem advancerSubsystem) {
        this.advancer = advancerSubsystem;
        addRequirements(advancerSubsystem);
    }
    boolean thing = false;
    @Override
    public void execute() {
        if(timer.hasElapsed(0.5) && thing){
            advancer.stopAdvancer();
            timer.reset();
        } else {
            advancer.advance();
        }
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
        timer.reset();
        timer.start();
    }

}
