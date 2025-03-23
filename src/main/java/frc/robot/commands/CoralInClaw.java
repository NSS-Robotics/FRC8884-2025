package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CoralInClaw extends Command {

    private final Claw m_claw;

    public CoralInClaw(Claw m_claw) {
        this.m_claw = m_claw;
    }

    // Called when the command is initially scheduled.

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        System.out.println(
            "truedddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_claw.gamePieceDetected();
    }
}
