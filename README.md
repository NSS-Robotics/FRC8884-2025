![Lines of code](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/NSS-Robotics/FRC8884-2025/badge&color=blue&label=Lines%20of%20code)

This is the Knight Owls' code for the 2025 season and one of the very few repos with a README.

Though if you thought the README would be useful, I'm (not really) sorry to disappoint you.

Okay fine, it can be somewhat useful.

## Structure

- `.gradle/`: No clue. Whatever stuff gradle needs, I guess.
- `.vscode/`: VSCode config? I don't know; I don't use VSCode.
- `.wpilib/`: Again, don't know. WPILib files, probably (mind blowing, right?).
- `build/`: Cached build files, maybe? I don't really know.
- `gradle/`: Is `.gradle` not enough? More gradle stuff, I'd assume.
- `src/main/`: Robot source code.
    - `deploy/`: No clue.
    - `java/`: Code directory
        - `frc/`: Our robot is packaged `frc.robot`, and utilies are `frc.lib`.
            - `lib/`: Libraries and utilities
                - `swerve/`: Swerve things, like SwerveModules and SwerveModuleConstants
            - `robot/`: Code for the robot
                - `commands/`: I think this is command for autos, but I'm not sure.
                - `subsystems/`: Subsystems, like Swerve, Pivot, etc.
- `vendordeps/`: Dependency files, like CTRE, Phoenix, etc.