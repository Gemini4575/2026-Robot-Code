package frc.robot.commands.auto.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class ShootFromAutoMiddle extends Command {
    private ShooterSubsystem shooter;
    private AdvancerSubsystem advancer;
    private Timer timer = new Timer();

    public ShootFromAutoMiddle(ShooterSubsystem shooter, AdvancerSubsystem advancer) {
        this.shooter = shooter;
        this.advancer = advancer;
        addRequirements(shooter, advancer);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.runShooterAtVelocity(Constants.ShooterRPMConstants.MIDDLE_AUTO_SHOOT);
    }

    boolean firstRun = false;

    @Override
    public void execute() {
        if (timer.hasElapsed(1)) {
            advancer.advance();
            firstRun = true;
        } else {
            advancer.stopAdvancer();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(5)) {
            shooter.stopShooter();
            advancer.stopAdvancer();
            firstRun = false;
            return true;
        }
        return false;
    }

}
