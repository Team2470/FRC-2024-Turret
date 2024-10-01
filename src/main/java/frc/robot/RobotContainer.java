// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class RobotContainer {

  double leftTrigger;
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = new Intake();
  private final Turret turret = new Turret();
  
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    //
    // Intake
    //
    joystick.rightBumper().whileTrue(intake.inCommand());
    
    joystick.leftBumper().whileTrue(intake.outCommand());

    //
    // Turret
    //
    joystick.povLeft().whileTrue(turret.openloopCommand(4)); //Counter Clockwise
    joystick.povRight().whileTrue(turret.openloopCommand(-4)); //Clockwise


    //
    // Drive Train
    //

    final var xFilter = new SlewRateLimiter(5);
    final var yFilter = new SlewRateLimiter(5);
    final var rotateFilter = new SlewRateLimiter(5);

    BooleanSupplier slowModeSupplier = () -> joystick.getHID().getXButton();

    DoubleSupplier rotationSupplier = () -> {
        double leftTrigger = joystick.getHID().getLeftTriggerAxis();
        double rightTrigger = joystick.getHID().getRightTriggerAxis();

        double rotate = 0.0;
        if (leftTrigger < rightTrigger) {
          rotate = -rightTrigger;
        } else {
          rotate = leftTrigger;
        }

        return rotateFilter.calculate(rotate) * MaxAngularRate;
    };

    Supplier<Translation2d> translationSupplier = () -> {
        // Read gamepad joystick state, and apply slew rate limiters
        Translation2d move = new Translation2d(
          // X Move Velocity - Forward
          MathUtil.applyDeadband(xFilter.calculate(-joystick.getHID().getLeftY()),.05),
          // Y Move Velocity - Strafe
          MathUtil.applyDeadband(yFilter.calculate(-joystick.getHID().getLeftX()),.05)
        );
        
        return new Translation2d(
            // Scale the speed of the robot by using a quadratic input curve.
            // and convert the joystick values -1.0-1.0 to Meters Per Second
            Math.pow(move.getNorm(), 2) * MaxSpeed, 
            // Get the direction the joystick is pointing 
            move.getAngle()
        );
    };

    // Field-centric by default
    final var fieldCentric = new SwerveRequest.FieldCentric();
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(()->{
            var translation = translationSupplier.get();


            double xMove = translation.getX();
            double yMove = translation.getY();
            double rotate = rotationSupplier.getAsDouble();

            if(slowModeSupplier.getAsBoolean()) {
              xMove *= 0.5;
              yMove *= 0.5;
              rotate *= 0.25;
            } else{
              xMove *= 1.0;
              yMove *= 1.0;
              rotate *= 0.5;
            }

            return fieldCentric
              .withVelocityX(xMove)
              .withVelocityY(yMove)
              .withRotationalRate(rotate);
        })
    );

    // Robot Centric when pressing A
    final var robotCentric = new SwerveRequest.RobotCentric();
    joystick.a().whileTrue(drivetrain.applyRequest(()->{
        var translation = translationSupplier.get();

        double xMove = translation.getX();
        double yMove = translation.getY();
        double rotate = rotationSupplier.getAsDouble();

        if(slowModeSupplier.getAsBoolean()) {
          xMove *= 0.5;
          yMove *= 0.5;
          rotate *= 0.25;
        } else{
          xMove *= 1.0;
          yMove *= 1.0;
          rotate *= 0.5;
        }

        return robotCentric
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withRotationalRate(rotate);
    }));

    // Align to Amp by pressing left bumper
    final var alignToAmp = new SwerveRequest.FieldCentricFacingAngle();
    alignToAmp.HeadingController.setPID(0,0,0);
    alignToAmp.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    // This makes angles specified below to be in referece to the driver. This means
    // angles will get automatically flipped if we are Red or Blue alliance.
    // If set to ForwardReference.RedAlliance then the angles will not be flipped.
    // TODO what is zero degrees?
    alignToAmp.ForwardReference = ForwardReference.RedAlliance; 
    joystick.leftBumper().whileTrue(drivetrain.applyRequest(()->{
        var translation = translationSupplier.get();

        double xMove = translation.getX();
        double yMove = translation.getY();

        if(slowModeSupplier.getAsBoolean()) {
          xMove *= 0.5;
          yMove *= 0.5;
        } else{
          xMove *= 1.0;
          yMove *= 1.0;
        }

        return alignToAmp
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withTargetDirection(Rotation2d.fromDegrees(90)
        );
    }));

    // Align to Source by pressing right bumper
    final var alignToSource = new SwerveRequest.FieldCentricFacingAngle();
    // This makes angles specified below to be in referece to the driver. This means
    // angles will get automatically flipped if we are Red or Blue alliance.
    // If set to ForwardReference.RedAlliance then the angles will not be flipped.
    // TODO what is zero degrees?
    alignToAmp.ForwardReference = ForwardReference.OperatorPerspective; 
    joystick.rightBumper().whileTrue(drivetrain.applyRequest(()->{
        var translation = translationSupplier.get();

        double xMove = translation.getX();
        double yMove = translation.getY();

        if(slowModeSupplier.getAsBoolean()) {
          xMove *= 0.5;
          yMove *= 0.5;
        } else{
          xMove *= 1.0;
          yMove *= 1.0;
        }

        // The angle we need to face needs to change depending on which alliance we are
        // on.
        double angle = -120.0;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
          angle = -60.0;
        }

        return alignToSource.withDeadband(0.05)
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withTargetDirection(Rotation2d.fromDegrees(angle)
        );
    }));

    // X-Stop
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // Reset the field-centric heading on start press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
