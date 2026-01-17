package frc.robot.subsystems.topDeck;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreak extends SubsystemBase {
    private final DigitalInput beam1;

    public BeamBreak() {
        beam1 = new DigitalInput(0);
    }

    public boolean get() {
        return beam1.get();
    }
}
