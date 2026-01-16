package frc.robot.commands.smartDashBoard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.model.MetricData;
import frc.robot.model.MetricName;
import frc.robot.service.MetricService;

public class SendNote extends Command {
    private final String note;

    public SendNote(String note) {
        this.note = note;
    }

    @Override
    public void initialize() {
        // Send the note to the SmartDashboard
    }

    @Override
    public void execute() {
        System.out.println("Sending note: " + note);
        MetricService.publish(MetricName.MESSAGE, "hiihidhfidifhi");
        System.out.println("after Sending note: " + note);
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes immediately after sending the note
    }

}
