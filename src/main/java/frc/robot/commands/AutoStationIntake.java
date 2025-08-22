package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class AutoStationIntake extends Command {

    private final Intake m_intake;
    private final LED l_led;
    private final double pose = Constants.IntakeConstants.upPosition;

    public AutoStationIntake(Intake m_intake, LED l_led) {
        this.m_intake = m_intake;
        this.l_led = l_led;

        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        l_led.ationIntake();
        m_intake.setPivot(pose, m_intake.getPosition() <= pose ? 0 : 1);
        m_intake.setIntake(Constants.IntakeConstants.stationVelocity, false);
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
        l_led.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_intake.getIntakeCurrent() > 30;
    }
}
