import wpilib.simulation

from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface):
        """
        :param physics_controller: `pyfrc.physics.core.PhysicsInterface` object
                                   to communicate simulation effects to
        """

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(1)
        self.r_motor = wpilib.simulation.PWMSim(2)

        # NavX (SPI interface)
        self.navx = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        self.physics_controller = physics_controller

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = wpilib.simulation.DifferentialDrivetrainSim(
            plant=LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3),
            trackWidth=0.8,
            driveMotor=DCMotor.NEO(2), # 2 NEO drivetrain.
            gearingRatio=10.71, # Common ToughBox Mini gear ratio.
            wheelRadius=0.0762, # 6" wheel's radius in meters.
        )
        # fmt: on

    def update_sim(self, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        # Use a simple percentage drive.
        self.drivetrain.setInputs(l_motor, -r_motor)
        self.drivetrain.update(tm_diff)

        # Update the gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but
        #    the returned pose is positive counter-clockwise
        self.navx_yaw.set(-self.drivetrain.getPose().rotation().degrees())

        # Update the robot's position on Field2D
        self.physics_controller.field.setRobotPose(self.drivetrain.getPose())
