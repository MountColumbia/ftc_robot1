/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


//For the IMU Sensor
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



/**
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 */
@Disabled
//@Autonomous(name="Gordon: Autonomous 2", group="Pushbot")

public class GordonAutonomous2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.3;

    static final int        UNKNOWN_COLOR = 0 ;
    static final int        BLUE = 1 ;
    static final int        RED = 2 ;

    // The IMU sensor object
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        setupIMU ();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int pos = 5;

        int our_color = RED ;
    if (pos == 3 || pos == 4)
        our_color = BLUE ;

    claws_grab_glyph ();
    jewel_sequence (our_color);

        if (pos == 1) {
            //pos 1 (red top)
            drive (FORWARD_SPEED, 2.0);
            turn(-90);
            drive (FORWARD_SPEED, 0.25);
        } else if (pos == 2) {
        //pos 2 (red bottom)
            drive (FORWARD_SPEED, 1.1);
            turn(90); //left
            drive (FORWARD_SPEED, 0.7);
            turn(0);//right
            drive (FORWARD_SPEED, 0.6);
        } else if (pos == 3) {
            //pos 3 (blue top)
            drive (-FORWARD_SPEED, 2.0);
            turn(-90);
            drive (FORWARD_SPEED, 0.25);
        } else if (pos ==4) {
        //pos 4 (blue bottom)
            drive (-FORWARD_SPEED, 1.1);
            turn(90); //left
            drive (FORWARD_SPEED, 0.7);
            turn(180);//right
            drive (FORWARD_SPEED, 0.6);
    }

    claws_release_glyph ();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    void setupIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    double getHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deg1 = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) ;
        deg1 = AngleUnit.DEGREES.normalize(deg1);
        return deg1;
    }

    void turn (double new_heading) {
        double cur_head = getHeading();
        double speed_dir = 1;
        if ((new_heading - cur_head) <= 180)
            speed_dir = -1;
       turn_phase (new_heading, speed_dir * 0.3, 10);
       turn_phase (new_heading, speed_dir * 0.2, 1);
    }

    void turn_phase (double new_heading, double turn_speed, double accepted_diff) {
        robot.leftDrive.setPower(-turn_speed);
        robot.rightDrive.setPower(turn_speed);
        while (opModeIsActive()) {
            double heading = getHeading();
            double diff = heading - new_heading ;

            if (diff >= -accepted_diff && diff <= accepted_diff)
                break;


            telemetry.addData("Heading", "%f", heading);
            telemetry.addData("bearing","%f", new_heading);
            telemetry.addData("diff","%f", diff);
            telemetry.addData("accpted_diff","%f", accepted_diff);
            telemetry.addData("turn_speed","%f", turn_speed);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    void drive (double speed, double time) {
        robot.leftDrive.setPower(speed);
        robot.rightDrive.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("speed", "%f", speed);
            telemetry.addData("Time", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    void just_wait (double timeout) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeout)) {
        }
    }

    void move_flicker(double new_position) {
        robot.flickrServo.setPosition(new_position);
        // There is no way to read the actual servo position,
        // So give it 2 seconds to complete the move.
        just_wait (2);
    }

    /* Return: 2 if RED detected
               1 if BLUE detected
               0 if unable to determine color */
    int read_flicker_color () {
        // Ugly hack: give the color sensor one more second
        // to read stable values (assuming the flickr arm stopped swining).
        // A better way (in the future) would be to read the actual
        // values and see when they stabilize.
        just_wait (1);

        int red = robot.color_sensor.red();
        int blue = robot.color_sensor.blue();

        if (red>2 && blue>2 && (red*2/3 > blue))
            return RED;
        if (red>2 && blue>2 && (blue*3/4 > red))
            return BLUE;

        return UNKNOWN_COLOR;
    }


   void flickr_drive (Boolean move_forward) {
       double speed = 0.2 ;
       if (!move_forward)
          speed = speed * -1 ;
        drive (speed, 0.5);
        //Don't bother returning to the original position
        //drive (-speed,0.5);
   }


    void claws_grab_glyph() {
    // close claws on pre-positioned glyph
    robot.leftClaw.setPosition (0.7);
    robot.rightClaw.setPosition (0.2);
    robot.leftArm.setPower(0.2);
    just_wait(1);
    robot.leftArm.setPower(0);
    }

    void claws_release_glyph () {
    robot.leftClaw.setPosition (0.5);
    robot.rightClaw.setPosition (0.5);
    }


    void jewel_sequence (int our_color) {
    // flickr + colorsensor + jewel sequence
    move_flicker (1); //flickr down
    int detected_color = read_flicker_color();
    if (detected_color != UNKNOWN_COLOR) {
        Boolean drive_forward = (our_color != detected_color) ;
        flickr_drive (drive_forward);
    }
    move_flicker (0.25); // flickr up
    }
}
