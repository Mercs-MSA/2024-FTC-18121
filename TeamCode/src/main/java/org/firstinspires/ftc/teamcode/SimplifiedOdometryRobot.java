/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

// TODO CHECK THE BOTTOM OF THE CODE FOR WHAT TO WORK ON
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class SimplifiedOdometryRobot {
    // Adjust these numbers to suit your robot.
    private static final double DRIVE_GAIN          = 0.05;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 3;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.05;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 2.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.01;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 0.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    private static final double OTOS_LINEAR_SCALAR  = 1.0113728;
    private static final double OTOS_ANGULAR_SCALAR = 0.9961217;

    private static final double OTOS_MOUNTING_OFFSET_X = -3.3125;
    private static final double OTOS_MOUNTING_OFFSET_Y = 1.75;
    private static final double OTOS_MOUNTING_OFFSET_Z = 0;

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
To find this, we first need to consider the total gear reduction powering our arm.
First, we have an external 20t:100t (5:1) reduction created by two spur gears.
But we also have an internal gear reduction in our motor.
The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
reduction of ~50.9:1. (more precisely it is 250047/4913:1)
We can multiply these two ratios together to get our final reduction of ~254.47:1.
The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    public static final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 265 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_AUTON_CLIMB           = 135 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    final double ARM_TEST_POSITION         = 20  * ARM_TICKS_PER_DEGREE;
    final double ARM_RESET_TELEOP          = -115* ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    public boolean WRIST_IS_IN = true;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor = 0;

    // Public Members
    public double drivenDistance     = 0; // scaled axial distance (+ = forward)
    public double strafedDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU

    // ---  Private Members

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    private ProportionalControl driveController;
    private ProportionalControl strafeController;
    private ProportionalControl yawController;

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    private Servo wrist;

    private CRServo intake;

    public DcMotor shoulder;

    private final LinearOpMode myOpMode;
    private SparkFunOTOS myOtos;
    private Datalog datalog;
    private VoltageSensor battery;

    private final ElapsedTime holdTimer              = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private SparkFunOTOS.Pose2D currentRobotPosition = new SparkFunOTOS.Pose2D(0, 0, 0); // Unmodified axial odometer count
    private SparkFunOTOS.Pose2D pathStartPoint       = new SparkFunOTOS.Pose2D(0,0,0); // Used to offset axial odometer
    private double rawHeading         = 0; // Unmodified heading (degrees)
    private double headingOffset      = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry     = false;

    private final boolean otosEnabled = true;

    private boolean driveInReverse    = false;
    private boolean strafeInReverse   = false;

    private boolean logData           = false;
    private int datalogCounter        = 0;

    // Robot Constructor
    public SimplifiedOdometryRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry, boolean logData)
    {
        // Set the desired telemetry/logger states
        this.showTelemetry = showTelemetry;
        this.logData = logData;

        // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
        this.driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false, this.myOpMode, "drive");
        this.strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false, this.myOpMode, "strafe");
        this.yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true, this.myOpMode, "rotate");

        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        leftFrontDrive  = setupDriveMotor("frontLeft", DcMotor.Direction.FORWARD);
        rightFrontDrive = setupDriveMotor("frontRight", DcMotor.Direction.REVERSE);
        leftBackDrive  = setupDriveMotor( "backLeft", DcMotor.Direction.FORWARD);
        rightBackDrive = setupDriveMotor( "backRight",DcMotor.Direction.REVERSE);

        wrist = myOpMode.hardwareMap.get(Servo.class,"wrist");
        intake = myOpMode.hardwareMap.get(CRServo.class, "hand");
        shoulder = myOpMode.hardwareMap.get(DcMotor.class, "shoulder");

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) shoulder).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        // THIS CODE IS NOT LEGAL FOR COMPETITION! ROBOT CANNOT MOVE BETWEEN AUTO & TELEOP
        //shoulder.setTargetPosition(0);
        //shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Make sure that the intake is off, and the wrist is folded in. */
        // THIS CODE IS NOT LEGAL FOR COMPETITION! ROBOT CANNOT MOVE BETWEEN AUTO & TELEOP
        //intake.setPower(INTAKE_OFF);
        //wrist.setPosition(WRIST_FOLDED_IN);

        battery = myOpMode.hardwareMap.voltageSensor.get("Control Hub");

        if (otosEnabled) {
            myOtos = setupSparkfunOTOS("sensor_otos");
        }

        // zero out all the odometry readings.
        resetOdometry();

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize the datalog
        if (this.logData) {
            datalog = new Datalog("datalog_01");

            // You do not need to fill every field of the datalog
            // every time you call writeLine(); those fields will simply
            // contain the last value.
            datalog.opModeStatus.set("INIT");
            datalog.battery.set(battery.getVoltage());
            datalog.writeLine();
        }
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    private SparkFunOTOS setupSparkfunOTOS(String deviceName) {
        SparkFunOTOS sensor = myOpMode.hardwareMap.get(SparkFunOTOS.class, deviceName);

        sensor.setLinearUnit(DistanceUnit.INCH);
        sensor.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(OTOS_MOUNTING_OFFSET_X, OTOS_MOUNTING_OFFSET_Y, OTOS_MOUNTING_OFFSET_Z);
        sensor.setOffset(offset);

        sensor.setLinearScalar(OTOS_LINEAR_SCALAR);
        sensor.setAngularScalar(OTOS_ANGULAR_SCALAR);

        sensor.calibrateImu();
        sensor.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        sensor.setPosition(currentPosition);

        return sensor;
    }

    public void activate() {
        if (this.logData) {
            datalog.opModeStatus.set("RUNNING");
        }
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {
        if (!otosEnabled) { return true; }

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        SparkFunOTOS.Pose2D vel  = myOtos.getVelocity();

        currentRobotPosition =  myOtos.getPosition();
        drivenDistance = distanceBetweenPoints(pathStartPoint, currentRobotPosition);
        // TODO Figure out how compare slopes between target path and current path. Store in StrafedDistance

        rawHeading = pos.h;
        heading     = rawHeading - headingOffset;
        turnRate    = vel.h;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", rawHeading - headingOffset, turnRate);
            myOpMode.telemetry.addData("Dist Driven To Target", drivenDistance);
            myOpMode.telemetry.addData("Path Start", "%5.2f %5.2f %5.2f", pathStartPoint.x, pathStartPoint.y, pathStartPoint.h);
            myOpMode.telemetry.addData("Current Pos", "%5.2f %5.2f %5.2f", currentRobotPosition.x, currentRobotPosition.y, currentRobotPosition.h);
        }

        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    //  ########################  Mid level control functions.  #############################3#


    /**
     * @param distance This is the distance inputted that you want to travel in
     */
    public SparkFunOTOS.Pose2D findTargetPosition(double distance) {
        double mathDistanceY = distance * Math.sin(Math.toRadians(getHeading()));
        double mathDistanceX = distance * Math.cos(Math.toRadians(getHeading()));

        return new SparkFunOTOS.Pose2D(mathDistanceX + currentRobotPosition.x, mathDistanceY + currentRobotPosition.y, getHeading());
    }

    public double distanceBetweenPoints(SparkFunOTOS.Pose2D pointA, SparkFunOTOS.Pose2D pointB) {
        return Math.sqrt(Math.pow((pointB.x - pointA.x), 2) + Math.pow((pointB.y - pointA.y), 2));
    }

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param targetDistance  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double targetDistance, double power, double holdTime) {
        resetOdometry();

        // if our target distance is negative, we're trying to drive backwards. But we're about to calculate
        // distance driven, which will always be a positive number. So we need to track our intention to drive backwards
        if (targetDistance < 0) {
            driveInReverse = true;
        }

        SparkFunOTOS.Pose2D targetPosition = findTargetPosition(targetDistance);
        // TODO This is where we can use targetPosition and currentPosition to find slope of target path. We can use slope in ReadSensors()

        driveController.reset(Math.abs(targetDistance), power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(drivenDistance), strafeController.getOutput(strafedDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.telemetry.update();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param targetDistance  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double targetDistance, double power, double holdTime) {
        resetOdometry();

        // if our target distance is positive, we're trying to drive "backwards" because the robot defaults
        // to left-strafe as positive, but the OTOS wants right-strafe to be positive. We're about to calculate
        // distance driven, which will always be a positive number. So we need to track our intention to drive "backwards"
        if (targetDistance > 0) {
            strafeInReverse = true;
        }

        SparkFunOTOS.Pose2D targetPosition = findTargetPosition(targetDistance);

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(Math.abs(targetDistance), power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(strafedDistance), strafeController.getOutput(drivenDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.telemetry.update();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.telemetry.update();
            myOpMode.sleep(10);
        }
        stopRobot();
    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){
        // If our intention was to drive/strafe in reverse, then we need to invert the power signal being sent from our controller
        if (driveInReverse) { drive *= -1; }
        if (strafeInReverse) { strafe *= -1; }

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            /* send telemetry to the driver of the arm's current position and target position */
            myOpMode.telemetry.addData("armTarget: ", shoulder.getTargetPosition());
            myOpMode.telemetry.addData("arm Encoder: ", shoulder.getCurrentPosition());

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) shoulder).isOverCurrent()){
                myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
        }

        if (logData) {
            datalog.loopCounter.set(datalogCounter);
            datalog.battery.set(battery.getVoltage());
            datalog.leftFrontEncoder.set(lF);
            datalog.rightFrontEncoder.set(rF);
            datalog.leftBackEncoder.set(lB);
            datalog.rightBackEncoder.set(rB);
            datalog.writeLine();
        }
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);

        // whenever the robot stops we should reset our drive/strafe direction intention
        driveInReverse = false;
        strafeInReverse = false;
    }

    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        readSensors();

        pathStartPoint = currentRobotPosition;

        drivenDistance = 0.0;
        driveController.reset(0);

        strafedDistance = 0.0;
        strafeController.reset(0);
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    public void incrementOpModeCounter() {
        this.datalogCounter++;
    }

    public void intakeInward() {
        intake.setPower(INTAKE_COLLECT);
    }
    public void intakeOutward() {
        intake.setPower(INTAKE_DEPOSIT);
    }
    public void intakeStop() {
        intake.setPower(INTAKE_OFF);
    }
    public void wristIn() {
        wrist.setPosition(WRIST_FOLDED_IN);
        WRIST_IS_IN = true;
    }
    public void wristOut() {
        wrist.setPosition(WRIST_FOLDED_OUT);
        WRIST_IS_IN = false;
    }

    public void shoulderMovement() {
                /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        shoulder.setTargetPosition((int) (armPosition + (FUDGE_FACTOR * armPositionFudgeFactor)));

        ((DcMotorEx) shoulder).setVelocity(2100);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /* This is the intaking/collecting arm position */
    public void shoulderCollect() {
        armPosition = ARM_COLLECT;
    }
    /* This is about 20° up from the collecting position to clear the barrier
                 Note here that we don't set the wrist position or the intake power when we
                 select this "mode", this means that the intake and wrist will continue what
                 they were doing before we clicked left bumper. */
    public void shoulderClearBarrier() {
        armPosition = ARM_CLEAR_BARRIER;
    }
    public void shoulderScoreSampleInLow() {
        armPosition = ARM_SCORE_SAMPLE_IN_LOW;
    }
    public void shoulderCollapsedIntoRobot() {
        armPosition = ARM_COLLAPSED_INTO_ROBOT;
    }
    public void shoulderScoreSpecimen() {
        armPosition = ARM_SCORE_SPECIMEN;
    }
    public void shoulderAttachHangingHook() {
        armPosition = ARM_ATTACH_HANGING_HOOK;
    }
    public void shoulderWinchRobot() {
        armPosition = ARM_WINCH_ROBOT;
    }
    public void shoulderAutonClimb() {
        armPosition = ARM_AUTON_CLIMB;
    }
    public void shoulderTestPosition() {
        armPosition = ARM_TEST_POSITION;
    }
    public void shoulderResetTeleop() {
        armPosition = ARM_RESET_TELEOP;
    }
    public void shoulderResetEncoder() {
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    String label;
    LinearOpMode myOpMode;
    ElapsedTime cycleTime = new ElapsedTime();

//    boolean checkpointLarge;
//    boolean checkpointSmall;

    public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular, LinearOpMode opmode, String label) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        this.myOpMode = opmode;
        this.label = label;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;
        
        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

         inPosition = (Math.abs(error) < tolerance);
//        checkpointLarge = (Math.abs(error) < tolerance);
//        checkpointSmall = (Math.abs(error) < tolerance / 2);
//        inPosition = (Math.abs(error) < tolerance / 4);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

//        if (checkpointLarge) {
//            output = output / 2;
//        }
//        if (checkpointSmall) {
//            output = output / 2;
//        }
        lastOutput = output;
        cycleTime.reset();
        myOpMode.telemetry.addData("Ctrl", "%s %5.2f %5.2f %5.2f %5.2f", this.label, error, output, setPoint, input);
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint the new point to set
     * @param powerLimit amount to limit power
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint the new point to set
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
//        checkpointLarge = false;
//        checkpointSmall = false;
    }
}

/*
 * This class encapsulates all the fields that will go into the datalog.
 */
class Datalog
{
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
    public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");

    public Datalogger.GenericField leftFrontEncoder = new Datalogger.GenericField("LF Enc");
    public Datalogger.GenericField rightFrontEncoder = new Datalogger.GenericField("RF Enc");
    public Datalogger.GenericField leftBackEncoder = new Datalogger.GenericField("LB Enc");
    public Datalogger.GenericField rightBackEncoder = new Datalogger.GenericField("RB Enc");
    public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

    public Datalog(String name)
    {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        opModeStatus,
                        loopCounter,
                        leftFrontEncoder,
                        rightFrontEncoder,
                        leftBackEncoder,
                        rightBackEncoder,
                        battery
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}