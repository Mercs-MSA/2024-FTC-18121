/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Four Sample to Net Facing Net")

public class AutoFourSampleToNet extends LinearOpMode
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
            for (int i = 0; i < 100; i++) {
                robot.shoulderWinchRobot(0);
            }
            robot.drive(5, 0.60, 0.1);
            robot.drive(-5, 0.60, 0.1);
            robot.strafe(51, 0.60, 0.1);
            robot.drive(7, 0.60, 0.1);
            // TODO: Try using between 7 - 9 inches for this one, see what works
            robot.strafe(-46, 0.60, 0.1);
            robot.strafe(46, 0.60, 0.1);
            robot.drive(7, 0.60, 0.1);
            robot.strafe(-46, 0.60, 0.1);
            robot.strafe(46, 0.60, 0.1);
            robot.drive(7, 0.60, 0.1);
            robot.strafe(-46, 0.60, 0.1);
            robot.strafe(51, 0.60, 0.1);
            robot.drive(-18, 0.60, 0.1);
            robot.turnTo(180, 0.60, 0.1);
            for (int i = 0; i < 100; i++) {
                robot.shoulderAttachHangingHook(0);
            }
            robot.drive(18, 0.60, 0.1);
        }
        telemetry.update();
        robot.incrementOpModeCounter();
    }
}
