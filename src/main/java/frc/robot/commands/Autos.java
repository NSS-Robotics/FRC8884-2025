package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public final class Autos {

    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(Swerve subsystem) {
        return Commands.sequence(new InstantCommand());
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
