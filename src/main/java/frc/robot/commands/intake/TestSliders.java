package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topDeck.IntakeSubystem;

public class TestSliders extends Command{
    private IntakeSubystem i;
    private DoubleSupplier joystickValue;

    public TestSliders(IntakeSubystem i, DoubleSupplier joystickValue) {
        this.i = i;
        this.joystickValue = joystickValue;
        addRequirements(i);
    }

    @Override
    public void execute() {
        i.testSliders(joystickValue.getAsDouble());
    }

     @Override
     public void end(boolean interrupted) {
         i.testSliders(0);
     }
}
