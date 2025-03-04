package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DownClimb extends Command {

    private final Climber m_climber;
    private BooleanSupplier operatorSaysYes; 

    public DownClimb(Climber m_climber, BooleanSupplier operatorSaysYes) {
        this.m_climber = m_climber;
        this.operatorSaysYes = operatorSaysYes;

        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(operatorSaysYes.getAsBoolean()) {
            m_climber.setClimber(Constants.ClimberConstants.climbRot, 1);
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
