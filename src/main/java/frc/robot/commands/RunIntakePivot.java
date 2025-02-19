package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakePivot extends Command {

    private final Intake m_intake;
    private final double pos;
    private final int slot;

    public RunIntakePivot(Intake m_intake, double pos, int slot) {
        this.m_intake = m_intake;
        this.pos = pos;
        this.slot = slot;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setPivot(pos, slot);
    }
}
