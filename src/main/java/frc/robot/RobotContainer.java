// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MazeRunner;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    public final Drivetrain drivetrain = Constants.Drive.DRIVETRAIN;

    // end configure yeet mode

    private final Field2d f = new Field2d();
    private final CommandXboxController joystick = new CommandXboxController(0);
//    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);
    private final Trigger FMSconnect = new Trigger(DriverStation::isEnabled);

    private enum ClimberPosition {
        Start,
        MovingUp,
        Up,
        MovingDown,
        Down
    }

    private ClimberPosition climberPosition = ClimberPosition.Start;

    public RobotContainer() {
        configureBindings();
        SmartDashboard.putData(this.f);
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand(String autoName) {
        return null;
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void periodic() {
        f.setRobotPose(drivetrain.getState().Pose);
    }

    private final Pose2d startBlueCenter = new Pose2d(1.4, 5.56, Rotation2d.fromDegrees(0.0));
    private final Pose2d startRedCenter = new Pose2d(15.1, 5.56, Rotation2d.fromDegrees(180.0));

    private final Pose2d startBlueAmp = new Pose2d(0.731, 6.696, Rotation2d.fromDegrees(60.0));
    private final Pose2d startRedAmp = new Pose2d(15.8, 6.696, Rotation2d.fromDegrees(120.0));

    private final Pose2d startBlueSource = new Pose2d(0.731, 4.435, Rotation2d.fromDegrees(-60.0));
    private final Pose2d startRedSource = new Pose2d(15.8, 4.435, Rotation2d.fromDegrees(-120.0));

    public void setStartingPose(String spot) {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0.0));
                if (spot.equals("amp")) {
                    drivetrain.resetPose(startBlueAmp);
                } else if (spot.equals("source")) {
                    drivetrain.resetPose(startBlueSource);
                } else {
                    drivetrain.resetPose(startBlueCenter);
                }
            } else {
                drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180.0));
                if (spot.equals("amp")) {
                    drivetrain.resetPose(startRedAmp);
                } else if (spot.equals("source")) {
                    drivetrain.resetPose(startRedSource);
                } else {
                    drivetrain.resetPose(startRedCenter);
                }
            }
        }
    }
}
