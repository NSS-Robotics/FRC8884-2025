package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import java.lang.constant.Constable;

public class AutoStationIntake extends Command {

    private final Intake m_intake;
    private final RobotContainer robotContainer;
    private final double pose = Constants.IntakeConstants.upPosition;

    public AutoStationIntake(RobotContainer robotContainer, Intake m_intake) {
        this.robotContainer = robotContainer;
        this.m_intake = m_intake;

        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setPivot(pose, m_intake.getPosition() <= pose ? 0 : 1);
        m_intake.setIntake(Constants.IntakeConstants.stationVelocity, false);
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_intake.getIntakeCurrent() > 30;
    }
}
