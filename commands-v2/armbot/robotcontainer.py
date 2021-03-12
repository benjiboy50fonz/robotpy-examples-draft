from commands2 import RunCommand, InstantCommand
from commands2.button import JoystickButton

from wpilib import XboxController
from wpilib.interfaces import GenericHID

from subsystems.drivetrain import DriveSubsystem
from subsystems.arm import ArmSubsystem

import constants


class RobotContainer:

    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self):
        self.driverController = XboxController(constants.kDriverControllerPort)

        self.robotDrive = DriveSubsystem()
        self.robotArm = ArmSubsystem()

        self.configureButtonBindings()

        self.robotDrive.setDefaultCommand(
            RunCommand(
                lambda: self.robotDrive.arcadeDrive(
                    # Inverse the value so joystick forward is robot forward.
                    -self.driverController.getRawAxis(1),
                    # Multiply by 65% to make it more controllable.
                    self.driverController.getRawAxis(2) * 0.65,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self):
        """
        Configure the joystick buttons for the driver's controller.
        """

        # Don't assign any to variables because we won't use them.

        # Go to 2 radians (according to the encoder) when the A button is pressed.
        (
            JoystickButton(self.driverController, XboxController.Button.kA.value)
            .whenPressed(lambda: self.robotArm.setGoal(2))
            .whenPressed(lambda: self.robotArm.enable())
        )

        # Tell the arm to return to home position when the B button is pressed.
        (
            JoystickButton(self.driverController, XboxController.Button.kB.value)
            .whenPressed(lambda: self.robotArm.setGoal(constants.kArmOffsetRads))
            .whenPressed(lambda: self.robotArm.enable())
        )

        # Stops the arm when the Y button is pressed.
        (
            JoystickButton(
                self.driverController, XboxController.Button.kY.value
            ).whenPressed(lambda: self.robotArm.disable())
        )

        (
            JoystickButton(
                self.driverController, XboxController.Button.kBumperRight.value
            )
            .whenPressed(lambda: self.robotDrive.setMaxOutput(0.5))
            .whenReleased(lambda: self.robotDrive.setMaxOutput(1))
        )

    def disablePIDSubsystem(self):
        """
        Disable the arm and stop it from whatever it's
        doing.
        """

        self.robotArm.disable()

    def getAutonomousCommand(self):
        """Return an empty command."""

        return InstantCommand()
