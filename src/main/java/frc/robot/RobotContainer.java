// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FollowPath;
import frc.robot.commands.RunAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.DriveMode;
import frc.robot.sysid.SysIdRoutines;

public class RobotContainer {

    public final Drivetrain drivetrain = Constants.Drive.DRIVETRAIN;
    public final Intake intake = Intake.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Cantdle lamp = Cantdle.getInstance();
    private final TalonFX climber = new TalonFX(Constants.Ports.CLIMBER, Constants.Ports.CTRE_CANBUS);

    // configure yeet mode by commenting one or the other

    private final Yeeter thrower = Yeeter.getInstance();
    // private final Thrower thrower = Thrower.getInstance();

    // end configure yeet mode

    private final Field2d f = new Field2d();
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);
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
        this.drivetrain.setDefaultCommand(drivetrain.teleopDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX).ignoringDisable(true));

        joystick.start().onTrue(drivetrain.runOnce(() -> {
            Pose2d currentPose = this.drivetrain.getState().Pose;
            if (DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue)
                    .equals(DriverStation.Alliance.Blue)) {
                this.drivetrain.seedFieldRelative(new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(0)));
            } else {
                this.drivetrain.seedFieldRelative(new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(180.0)));
            }
        }));

        this.joystick.a().onTrue(
                this.drivetrain.setDriveMode(DriveMode.UNDERHAND_SPEAKER_DRIVE)
                        .alongWith(this.thrower.prepareSpeaker())
                        .alongWith(this.intake.off())
                        .alongWith(this.arm.trackAngle(CameraVision::getUnderhandAngle)));
        // .alongWith(this.arm.setTuned()));

        this.joystick.y().onTrue(
                this.arm.setOverhand()
                        .alongWith(this.thrower.prepareSpeaker())
                        .alongWith(this.intake.off())
                        .alongWith(this.drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC_DRIVE)));

        this.joystick.b().onTrue(
                this.drivetrain.setDriveMode(DriveMode.AMP_DRIVE)
                        .alongWith(this.arm.setAmp())
                        .alongWith(this.thrower.prepareAmp())
                        .alongWith(this.intake.off()));

        this.joystick.x().onTrue(
                this.arm.setStow()
                        .alongWith(this.intake.off())
                        .alongWith(this.thrower.off())
                        .alongWith(this.drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC_DRIVE)));

        // Intake Note
        this.joystick.rightBumper()
                .onTrue(
                        this.intake.intakeNote()
                                .alongWith(this.arm.setIntake())
                                .alongWith(this.thrower.setIntake())
                                .alongWith(this.drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC_DRIVE)))
                .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.hold()));

        // Eject stuck Note
        this.joystick.leftBumper()
                .onTrue(this.thrower.eject()
                        .alongWith(this.intake.reverseIntake())
                        .alongWith(this.drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC_DRIVE)))
                .onFalse(this.thrower.off());

        // Throw note
        this.joystick.rightTrigger()
                .onTrue(
                        this.thrower.launch()
                                .andThen(
                                        new ConditionalCommand(
                                                this.arm.setStowSlow(),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(0.5),
                                                        this.arm.setStow()),
                                                arm::isAmping))
                                .andThen(this.thrower.off())
                                .andThen(this.drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC_DRIVE)));

        // driver feedback
        this.thrower.hasNote
                .onTrue(new InstantCommand(() -> this.joystick.getHID().setRumble(RumbleType.kBothRumble, 0.6))
                        .alongWith(this.lamp.flashColor(Cantdle.ORANGE, 1.0)))
                .onFalse(new InstantCommand(() -> this.joystick.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
        this.thrower.ready.onTrue(this.lamp.flashColor(Cantdle.GREEN, 0.5));
        FMSconnect.onTrue(this.lamp.flashColor(Cantdle.PURPLE, 2.5));

        // this.joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // this.joystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // this.joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // this.joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // this.joystick.a().whileTrue(thrower.YeeterRoutine.dynamic(Direction.kForward));
        // this.joystick.b().whileTrue(thrower.YeeterRoutine.dynamic(Direction.kReverse));
        // this.joystick.x().whileTrue(thrower.YeeterRoutine.quasistatic(Direction.kForward));
        // this.joystick.y().whileTrue(thrower.YeeterRoutine.quasistatic(Direction.kReverse));

        // this.joystick.a().whileTrue(SysIdRoutines.armTorqueCurrentRoutine.dynamic(Direction.kForward));
        // this.joystick.b().whileTrue(SysIdRoutines.armTorqueCurrentRoutine.dynamic(Direction.kReverse));
        // this.joystick.x().whileTrue(SysIdRoutines.armTorqueCurrentRoutine.quasistatic(Direction.kForward));
        // this.joystick.y().whileTrue(SysIdRoutines.armTorqueCurrentRoutine.quasistatic(Direction.kReverse));


        this.climber.setInverted(true);
        this.climber.setNeutralMode(NeutralModeValue.Brake);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand(String autoName) {
        RunAuto auto = null;

        switch (autoName){
            case "test":
            case "cheese":
            case "sauce":
            case "simple":
                auto = new RunAuto(autoName, drivetrain, 1, true, this.f);
                auto.setNamedEvent("intake", () -> this.intake.intakeNote()
                        .alongWith(this.arm.setIntake())
                        .alongWith(this.thrower.setIntake()));
                auto.setNamedEvent("intakeOff", () -> this.arm.setStow()
                        .alongWith(this.intake.off())
                        .alongWith(this.thrower.off()));
                auto.setNamedEvent("prepSpeaker", () -> this.intake.off()
                        .alongWith(this.thrower.prepareSpeaker())
                        .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND + 0.02)));
                auto.setNamedEvent("shoot", this.thrower::launch);
                break;
            default:
                this.f.getObject("traj").setPoses();
                this.f.getObject("trajPoses").setPoses();
        }

        return auto;
    }

    public Command getAutonomousCommand() {
        FollowPath hmm = new FollowPath("cheese", this.drivetrain, true);
        return new SequentialCommandGroup(
                this.intake.off().alongWith(this.thrower.prepareSpeaker())
                        .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND)),
                new WaitCommand(0.5),
                this.thrower.launch(),
                new WaitCommand(0.25),
                this.arm.setStow().alongWith(this.intake.off()),
                new WaitCommand(0.25),
                new ParallelCommandGroup(
                        hmm,
                        new SequentialCommandGroup(
                                this.intake.intakeNote().alongWith(this.thrower.setIntake()),
                                new WaitCommand(2.2),
                                this.intake.off().alongWith(this.thrower.prepareSpeaker())
                                        .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND + 0.02)),
                                new WaitCommand(0.85),
                                this.thrower.launch(), // throw 1
                                new WaitCommand(0.1),
                                this.intake.intakeNote().alongWith(this.thrower.setIntake()).alongWith(this.arm.setStow()),
                                new WaitCommand(1.45),
                                this.intake.off().alongWith(this.thrower.prepareSpeaker())
                                        .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND + 0.02)),
                                new WaitCommand(0.7),
                                this.thrower.launch(), // throw 2
                                new WaitCommand(0.2),
                                this.intake.intakeNote().alongWith(this.thrower.setIntake()).alongWith(this.arm.setStow()),
                                new WaitCommand(1.7),
                                this.intake.off().alongWith(this.thrower.prepareSpeaker())
                                        .alongWith(this.arm.setAngle(0.14)),
                                new WaitCommand(0.7),
                                this.thrower.launch(), // throw 3
                                new WaitCommand(0.2),
                                this.thrower.off().alongWith(this.arm.setStow()))));

        // return this.intake.off().alongWith(this.thrower.prepareSpeaker())
        // .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND))
        // .andThen(new WaitCommand(0.5))
        // .andThen(this.thrower.launch())
        // .andThen(new WaitCommand(0.25))
        // .andThen(this.arm.setStow().alongWith(this.intake.off()))
        // .andThen(new WaitCommand(0.25))
        // .andThen(new FollowPath("cheese", drivetrain, true)

        // .alongWith(new WaitCommand(0)
        // .andThen(this.intake.intakeNote().alongWith(this.thrower.setIntake()))).asProxy()

        // .alongWith(new WaitCommand(2.17)
        // .andThen(this.intake.off().alongWith(this.thrower.prepareSpeaker())
        // .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND +
        // 0.125)))).asProxy()

        // // throw number 2
        // .alongWith(new WaitCommand(2.9)
        // .andThen(this.thrower.launch())).asProxy()

        // .alongWith(new WaitCommand(3.4)
        // .andThen(this.intake.intakeNote().alongWith(this.thrower.setIntake()).alongWith(this.arm.setStow()))).asProxy()

        // .alongWith(new WaitCommand(4.3)
        // .andThen(this.intake.off().alongWith(this.thrower.prepareSpeaker())
        // .alongWith(this.arm.setAngle(Constants.Arm.Positions.OVERHAND +
        // 0.125)))).asProxy()

        // // throw number 3
        // .alongWith(new WaitCommand(4.7)
        // .andThen(this.thrower.launch())).asProxy()

        // .alongWith(new WaitCommand(5.4)
        // .andThen(this.intake.intakeNote().alongWith(this.thrower.setIntake()).alongWith(this.arm.setStow()))).asProxy()

        // .alongWith(new WaitCommand(6.5)
        // .andThen(this.intake.off().alongWith(this.thrower.prepareSpeaker())
        // .alongWith(this.arm.setAngle(0.25 * Math.PI + 0.1)))).asProxy()

        // // throw number 4
        // .alongWith(new WaitCommand(7.3)
        // .andThen(this.thrower.launch())).asProxy()

        // .alongWith(new WaitCommand(8.0)
        // .andThen(this.thrower.off().alongWith(this.arm.setStow()))).asProxy());
        // return new SequentialCommandGroup(
        // this.arm.setOverhand()
        // .alongWith(this.thrower.prepareSpeaker())
        // .alongWith(this.intake.off()),
        // new WaitCommand(1.0),
        // this.thrower.launch(),
        // new WaitCommand(1.0),
        // this.arm.setIntake().alongWith(this.thrower.off()));
    }

    public void periodic() {
        SmartDashboard.putNumber("front cam angle to goal", LimeLight.front.getLatest().tx);
        SmartDashboard.putNumber("angle target", this.drivetrain.targetHeading().getDegrees());

        LimeLight.front.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        LimeLight.back.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());

        // try {
        // boolean reject = false;
        // LimelightHelpers.PoseEstimate llPose =
        // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

        // if (llPose != null) {
        // reject |= (Math.abs(this.drivetrain.getPigeon2().getRate()) >= 360.0);
        // reject |= llPose.avgTagArea <= 0.2;
        // // reject |= llPose.avgTagDist >= 4.0;

        // if (!reject) {
        // this.drivetrain.addVisionMeasurement(llPose.pose, llPose.timestampSeconds,
        // VecBuilder.fill(5.0, 5.0, 999.0));
        // }
        // }

        // } catch (Exception e) {
        // // oops???
        // }

        switch (this.climberPosition) {
            case MovingUp:
                this.climber.set(Constants.Climber.UP_VELOCITY);
                if (this.climber.getPosition().getValueAsDouble() >= Constants.Climber.UP_POSITION) {
                    this.climberPosition = ClimberPosition.Up;
                }
                break;
            case MovingDown:
                this.climber.set(this.joystick.getLeftTriggerAxis());
                if (this.climber.getPosition().getValueAsDouble() >= Constants.Climber.DOWN_POSITION) {
                    this.climberPosition = ClimberPosition.Down;
                }
                break;
            case Start:
                this.climber.set(0.0);
                if (this.joystick.getHID().getXButton() && this.joystick.getHID().getLeftTriggerAxis() >= Constants.Climber.ERROR) {
                    this.climberPosition = ClimberPosition.MovingUp;
                }
                break;
            case Up:
                this.climber.set(0.0);
                if (this.joystick.getHID().getXButton() && this.joystick.getHID().getLeftTriggerAxis() >= Constants.Climber.ERROR) {
                    this.climberPosition = ClimberPosition.MovingDown;
                }
                break;
            case Down:
                this.climber.set(0.0);
                break;
        }
        SmartDashboard.putString("Climber Target", this.climberPosition.toString());

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
                    drivetrain.seedFieldRelative(startBlueAmp);
                } else if (spot.equals("source")) {
                    drivetrain.seedFieldRelative(startBlueSource);
                } else {
                    drivetrain.seedFieldRelative(startBlueCenter);
                }
            } else {
                drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180.0));
                if (spot.equals("amp")) {
                    drivetrain.seedFieldRelative(startRedAmp);
                } else if (spot.equals("source")) {
                    drivetrain.seedFieldRelative(startRedSource);
                } else {
                    drivetrain.seedFieldRelative(startRedCenter);
                }
            }
        }
    }
}
