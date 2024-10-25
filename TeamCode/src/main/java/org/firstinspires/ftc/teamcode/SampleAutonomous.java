/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Autonomous Test")

public class SampleAutonomous extends LinearOpMode
{
    // get an instance of the "Robot" class.
    final private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true, true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");

        waitForStart();

        robot.resetHeading();  // Reset heading to set a baseline for Auto
        robot.activate();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            robot.drive(48, 0.60, 0.25);
            //robot.drive(  -12, 0.20, 0.25);
            robot.turnTo(90, 0.60, 0.25);
            //robot.drive(  -12, 0.20, 0.25);
            //robot.turnTo(180, 0.20, 0.25);

            robot.drive(25, 0.60, 0.25);
            robot.strafe(45, 0.60, 0.25);
        }

        telemetry.update();
        robot.incrementOpModeCounter();
    }
}
