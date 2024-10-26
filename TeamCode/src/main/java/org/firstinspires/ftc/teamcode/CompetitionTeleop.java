package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Competition")
public class CompetitionTeleop extends LinearOpMode {

    final double SAFE_DRIVE_SPEED   = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.7 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    boolean ResetArm = false;

    // get an instance of the "Robot" class.
    final private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    public void runOpMode() {

        robot.initialize(true, true);

        robot.readSensors();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        robot.activate();

        while (opModeIsActive()) {
            robot.readSensors();


//            if (!ResetArm) {
//                for (int i = 0; i < 100; i++) {
//                    robot.shoulderResetTeleop(0);
//                }
//                robot.shoulderResetEncoder();
//                ResetArm = true;
//            }

            // Allow the driver to reset the gyro by pressing both small gamepad buttons
            if (gamepad1.start && gamepad1.back) {
                robot.resetHeading();
                robot.resetOdometry();
            }

            if (gamepad2.left_trigger > 0.2) {
                telemetry.addData("Intake?", "Out");
                robot.intakeOutward();
            }
            else if (gamepad2.right_trigger > 0.2) {
                telemetry.addData("Intake?", "In");
                robot.intakeInward();
            }
            else {
                telemetry.addData("Intake?", "Off");
                robot.intakeStop();
            }

            if(gamepad2.right_bumper){
                robot.shoulderCollect(gamepad2.left_stick_y);
                robot.wristOut();
                robot.intakeInward();
            }
            else if (gamepad2.left_bumper){
                robot.shoulderClearBarrier(gamepad2.left_stick_y);
            }
            else if (gamepad2.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                robot.shoulderScoreSampleInLow(gamepad2.left_stick_y);
                robot.wristOut();
            }
            else if (gamepad2.dpad_left) {
                /* This turns off the intake, folds in the wrist, and moves the arm
                back to folded inside the robot. This is also the starting configuration */
                robot.shoulderCollapsedIntoRobot(gamepad2.left_stick_y);
                robot.intakeStop();
                robot.wristIn();
            }
            else if (gamepad2.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                robot.shoulderScoreSpecimen(gamepad2.left_stick_y);
                robot.wristIn();
            }
            else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                robot.shoulderAttachHangingHook(gamepad2.left_stick_y);
                robot.intakeStop();
                robot.wristIn();
            }
            else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                robot.shoulderWinchRobot(gamepad2.left_stick_y);
                robot.intakeStop();
                robot.wristIn();
            }



            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

            // read joystick values and scale according to limits set at top of this file
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick

            //  OR... For special conditions, Use the DPAD to make pure orthoginal motions
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 2.0;
            }

            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);

            telemetry.update();
            robot.incrementOpModeCounter();
        }
    }
}