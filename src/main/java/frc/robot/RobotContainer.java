package frc.robot;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Shooter m_Shooter = new Shooter();
    private final Intake m_Intake = new Intake();
    private final ShooterAdjuster m_ShooterAdjuster = new ShooterAdjuster();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    CommandXboxController driverXbox = new CommandXboxController(0);
    CommandXboxController operatorXbox = new CommandXboxController(1);
    // CommandJoystick pretendJoystic = new CommandJoystick(2);
    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int headingXAxis = XboxController.Axis.kRightX.value;
    private final int headingYAxis = XboxController.Axis.kRightY.value;

    private final Trigger robotCentric = driverXbox.leftBumper();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final Field2d field;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerveAngle(
                        s_Swerve,
                        () -> driverXbox.getRawAxis(translationAxis),
                        () -> driverXbox.getRawAxis(strafeAxis),
                        () -> -driverXbox.getRawAxis(headingXAxis),
                        () -> -driverXbox.getRawAxis(headingYAxis),
                        () -> robotCentric.getAsBoolean()));

        SmartDashboard.putNumber("Amp speed", .25);
        // pretendJoystic.button(1).onTrue(getAutonomousCommand());
        // registers Named Commands
        NamedCommands.registerCommand("IntakeOff", new IntakeOff(m_Intake));
        NamedCommands.registerCommand("IntakeOn", new IntakeOn(m_Intake, false));
        NamedCommands.registerCommand("PrepareToShoot", new PrepareToShoot(m_Shooter));
        NamedCommands.registerCommand("Shoot", new Shoot(m_Shooter, 12, 12));
        NamedCommands.registerCommand("Shoot Auto", new ShootAuto(m_Shooter));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Configure the button bindings
        configureButtonBindings();

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        // driverXbox.back().onTrue(new
        // InstantComma''''''''''''''''''''''''''''''''''''''''''''''''''nd(drivebase::addFakeVisionReading));

        // Intake Commands
        operatorXbox.start().whileTrue(new IntakeFromShooter(m_Shooter))
                .onTrue(new InstantCommand(() -> {
                    m_ShooterAdjuster.goToZero();
                }, m_ShooterAdjuster));

        driverXbox.rightBumper().whileTrue(new IntakeOn(m_Intake, true))
                .whileTrue(new InstantCommand(() -> {
                    m_Shooter.setBottomMotorVolts(.05 * 12.0);
                }, m_Shooter))
                .onTrue(new InstantCommand(() -> {
                    m_ShooterAdjuster.goToZero();
                }, m_ShooterAdjuster));

        operatorXbox.leftTrigger().whileTrue(new IntakeReverse(m_Intake));

        // operatorXbox.rightBumper().whileTrue(new InstantCommand(() ->
        // {m_ShooterAdjuster.setSpeed(operatorXbox.getRightY() * .5);},
        // m_ShooterAdjuster));

        // Shooter Commands

        DoubleSupplier mySupplier = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                // TODO Auto-generated method stub
                return SmartDashboard.getNumber("Amp speed", .25) * 12.0;
            }

        };

        driverXbox.rightTrigger().whileTrue(new Shoot(m_Shooter, 11
    , 11));//both were 11
        driverXbox.leftTrigger().whileTrue(new InstantCommand(() -> {
            m_Shooter.ampShot();
        }, m_Shooter))
                .onFalse(new InstantCommand(() -> {
                    m_Shooter.shooterOff();
                }, m_Shooter));

        operatorXbox.rightTrigger().onTrue(new PrepareToShoot(m_Shooter));

        driverXbox.leftBumper().onTrue(new InstantCommand(() -> {
            m_ShooterAdjuster.setPosition(Constants.ShooterAngles.UNDER_STAGE);
        }, m_ShooterAdjuster));
        operatorXbox.y().onTrue(new InstantCommand(() -> {
            m_ShooterAdjuster.goToZero();
        }, m_ShooterAdjuster));
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            m_ShooterAdjuster.setPosition(Constants.ShooterAngles.AMP);
        }, m_ShooterAdjuster));

        operatorXbox.povUp()
                .onTrue((new InstantCommand(() -> m_ShooterAdjuster.setPosition(Constants.ShooterAngles.PEDESTAL))));
        // operatorXbox.povLeft().onTrue((new
        // InstantCommand(()->m_ShooterAdjuster.setPosition(Constants.ShooterAngles.AMP_ZONE))));

        // operatorXbox.povUp().whileTrue(new ShooterUp(m_ShooterAdjuster));
        // operatorXbox.povDown().whileTrue(new ShooterDown(m_ShooterAdjuster));
       // operatorXbox.a().onTrue((new
        // InstantCommand(()->m_ShooterAdjuster.setPosition(.90)))); //Drive Commands
        /* Driver Buttons */
        //driverXbox.start().onTrue(new InstantCommand(() -> s_Swerve.resetGyro()));

        Optional<Alliance> alliance = DriverStation.getAlliance();
        driverXbox.y().onTrue(new AnglePreset(s_Swerve, 180, 0)); //DEFAULT
        driverXbox.a().onTrue(new AnglePreset(s_Swerve, 180 - 47, 47)); //AMP ZONE
        driverXbox.b().onTrue(new AnglePreset(s_Swerve, -90, -26)); //RED:AMP BLUE:PODIUM
        driverXbox.x().onTrue(new AnglePreset(s_Swerve, 180 + 26, -90)); //RED:PODIUM BLUE:AMP
        // if (/* alliance.isPresent() && alliance.get() == Alliance.Red */false) {
        //     driverXbox.y().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(180)))); // DEFAULT
        //     driverXbox.a().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(180 - 47)))); // AMP ZONE
        //     driverXbox.b().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(-90)))); // AMP
        //     driverXbox.x().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(180 + 26)))); // PODIUM

        // } else {
        //     driverXbox.y().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(0)))); // DEFAULT
        //     driverXbox.a().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(47)))); // AMP ZONE
        //     driverXbox.x().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(-90)))); // AMP
        //     driverXbox.b().onTrue((new InstantCommand(() -> s_Swerve.setDesiredAngle(-26)))); // PODIUM
        // }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return drivebase.sysIdDriveMotorCommand();
        // An example command will be run in autonomousas
        // return new PathPlannerAuto("4-Auto");
        // return new PathPlannerAuto("Straight Auto");
        // return new ShootAuto(m_Shooter);
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            s_Swerve.setDesiredAngle(180); // DEFAULT
        } else {
            s_Swerve.setDesiredAngle(0); // DEFAULT
        }
        // drivebase.setDefaultCommand();
    }
}
