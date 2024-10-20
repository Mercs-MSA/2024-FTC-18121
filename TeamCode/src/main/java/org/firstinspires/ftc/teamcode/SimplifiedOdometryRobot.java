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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class SimplifiedOdometryRobot {
    // Adjust these numbers to suit your robot.
    private static final double DRIVE_GAIN          = 0.05;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 1.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 10;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.05;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 10;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.003;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 0.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 16;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    private static final double OTOS_LINEAR_SCALAR  = 1.0113728;
    private static final double OTOS_ANGULAR_SCALAR = 0.9961217;

    private static final double OTOS_MOUNTING_OFFSET_X = -3.3125;
    private static final double OTOS_MOUNTING_OFFSET_Y = 1.75;
    private static final double OTOS_MOUNTING_OFFSET_Z = 0;

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

    private final boolean otosEnabled = false;

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
            myOpMode.telemetry.addData("Target Offset", "%5.2f %5.2f %5.2f", pathStartPoint.x, pathStartPoint.y, pathStartPoint.h);
            myOpMode.telemetry.addData("Target Raw", "%5.2f %5.2f %5.2f", currentRobotPosition.x, currentRobotPosition.y, currentRobotPosition.h);
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

        driveController.reset(targetDistance, power);   // achieve desired drive distance
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

        // if our target distance is negative, we're trying to drive backwards. But we're about to calculate
        // distance driven, which will always be a positive number. So we need to track our intention to drive backwards
        if (targetDistance < 0) {
            strafeInReverse = true;
        }

        SparkFunOTOS.Pose2D targetPosition = findTargetPosition(targetDistance);

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(targetDistance, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(drivenDistance), strafeController.getOutput(strafedDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
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

    boolean checkpointLarge;
    boolean checkpointSmall;

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

        // inPosition = (Math.abs(error) < tolerance);
        checkpointLarge = (Math.abs(error) < tolerance);
        checkpointSmall = (Math.abs(error) < tolerance / 2);
        inPosition = (Math.abs(error) < tolerance / 4);

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

        if (checkpointLarge) {
            output = output / 2;
        }
        if (checkpointSmall) {
            output = output / 2;
        }
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
        checkpointLarge = false;
        checkpointSmall = false;
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