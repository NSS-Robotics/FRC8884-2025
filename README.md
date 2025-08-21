![Lines of code](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/NSS-Robotics/FRC8884-2025/badge&color=blue&label=Lines%20of%20code)
![Code quality: Garbage](https://img.shields.io/badge/code_quality-garbage-red.svg)

This is the Knight Owls' code for the 2025 season and one of the very few repos with a README.

This branch is only to be used when the robot is being driven by one person.
Made specially for Head Start to High School 2025.

Though if you thought the README would be useful, I'm (not really) sorry to disappoint you.

Okay fine, it can be somewhat useful.

## Structure

- `.vscode/`: VSCode config? I don't know; I don't use VSCode.
- `.wpilib/`: Again, don't know. WPILib files, probably (mind blowing, right?).
- `build/`: Cached build files, maybe? I don't really know.
- `gradle/`: Gradle stuff, I'd assume.
- `src/main/`: Robot source code.
    - `deploy/`: Files that get deployed to the robot.
    - `java/`: Code directory
        - `frc/`: Our robot is packaged `frc.robot`, and utilies are `frc.lib`.
            - `lib/`: Libraries and utilities
                - `swerve/`: Swerve things, like SwerveModules and SwerveModuleConstants
            - `robot/`: Code for the robot
                - `commands/`: I think this is command for autos, but I'm not sure.
                - `subsystems/`: Subsystems, like Swerve, Pivot, etc.
- `vendordeps/`: Dependency files, like CTRE, Phoenix, etc.

## Problems

- You can't do station intake (unless we bind the backpedals but I think we're using those too)
- It's kinda hell to do everything yourself
- Auto align only works for April Tag 7 (left & right poles) because I was lazy and we didn't need to align to all the poles.
