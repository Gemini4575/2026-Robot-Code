package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ClimberSubsystem;

public class ClimbTelop extends Command {
    private ClimberSubsystem c;
    private final BooleanSupplier climberDown;
    private final BooleanSupplier climberUp;

    public ClimbTelop(ClimberSubsystem c, BooleanSupplier climberDown, BooleanSupplier climberUp) {
        this.c = c;
        addRequirements(c);
        this.climberDown = climberDown;
        this.climberUp = climberUp;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (climberDown.getAsBoolean()) {
            c.MoveDownToClimb();
        } else if (climberUp.getAsBoolean()) {
            c.MoveUpToClimb();
        } else {
            c.MoveTo0();
        }
    }

    @Override
    public void end(boolean interrupted) {
        c.stop();
    }
}
