package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ClimberSubsystem;

public class ClimbTelop extends Command {
    private ClimberSubsystem c;
    private final BooleanSupplier climberButton;

    public ClimbTelop(ClimberSubsystem c, BooleanSupplier climberButton) {
        this.c = c;
        addRequirements(c);
        this.climberButton = climberButton;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (climberButton.getAsBoolean()) {
            if (c.MoveDownToClimbCheck()) {
                c.MoveUpToClimb();
            } else {
                c.MoveDownToClimb();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        c.stop();
    }
}
