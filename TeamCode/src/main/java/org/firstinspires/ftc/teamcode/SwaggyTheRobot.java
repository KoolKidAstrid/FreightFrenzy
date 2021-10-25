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

// test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//vision import
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
//dont mind me just importing some imu stuff
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//blinkin import
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

// This is not an OpMode.  It is a class that holds all the boring stuff

public class SwaggyTheRobot {


    public DcMotor[] LeftMotors = new DcMotor[2];
    public DcMotor[] RightMotors = new DcMotor[2];
    public DcMotor[] AllMotors = new DcMotor[4];


    BNO055IMU imu;

    // Motors
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    public DcMotor DK = null;

    // just gonna define some variables for encoders real quick dont mind me
    static final double mmPerInch               = 25.4f;    // this is jus math tho
    static final double countsPerRevolution     = 383.6f;   // Gobilda Yellowjacket 435
    static final double wheelDiameterMM         = 96;      // For figuring circumference
    static final double WheelDiameterIn         = wheelDiameterMM / mmPerInch;
    static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);

    double autoSpeedMult = 0.5;


    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public SwaggyTheRobot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {

        // imu stuff from last year cause i dont want to rebuild this from the ground up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "IMU");

        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");

        DK = OpModeReference.hardwareMap.get(DcMotor.class, "DK");

        // motor arrays
        // left
        LeftMotors[0] = FL;
        LeftMotors[1] = BL;
        // right
        RightMotors[0] = FR;
        RightMotors[1] = BR;
        // all
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = BL;
        AllMotors[3] = BR;

        for (DcMotor l : LeftMotors)
            l.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor r : RightMotors)
            r.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : AllMotors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // initialize the IMU
        imu.initialize(parameters);
    }


    // This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
// This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
    public void turn(double targetAngleDifference, double power) {

        // before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        // just some boolean variables to tell if we've stepped motor power down
        // might actually want more than two steps
        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        // if target angle is Negative, we're turning RIGHT
        if (targetAngleDifference < 0) {
            // turning right, so we want all right motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(-power * autoSpeedMult);
            for (DcMotor m : LeftMotors)
                m.setPower(power * autoSpeedMult);
            // sleep a tenth of a second
            // WARNING - not sure why this is needed - but sometimes right turns didn't work without
            OpModeReference.sleep(100);

            // we're turning right, so our target angle difference will be negative (ex: -90)
            // so GetAngleDifference will go from 0 to -90
            // keep turning while difference is greater than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power * autoSpeedMult /2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power * autoSpeedMult /2);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power * autoSpeedMult);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power * autoSpeedMult);
                    firstStepDownComplete = true;
                }

                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
//                OpModeReference.telemetry.addData("LeftMotorPower", ML.getPower());
//                OpModeReference.telemetry.addData("RightMotorPower", MR.getPower());
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {
            // turning left so want all left motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(power);
            for (DcMotor m : LeftMotors)
                m.setPower(-power);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power * autoSpeedMult /2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power * autoSpeedMult /2);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power * autoSpeedMult);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power * autoSpeedMult);
                    firstStepDownComplete = true;
                }
                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorPower", FL.getPower());
                OpModeReference.telemetry.addData("RightMotorPower", FR.getPower());
                OpModeReference.telemetry.update();
            }
        } else {
            // is zero - not turning - just return
            return;
        }

        // turn all motors off
        stopDriving();
    }

    // this is a method to get the current heading/z angle from the IMU
    // WE WANT THE Z ANGLE :)
    // AxesOrder.XYZ means we want thirdAngle
    // AxesOrder.ZYX would mean we want firstAngle
    public double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    public double GetAngleDifference(double startAngle) {
        double angleDifference = GetCurrentZAngle() - startAngle;

        // handle going past the 0 or 180 barriers
        // where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }

    //Autonomous Methods:

    public void stopDriving (){
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }


    public void drive(double inches, double speed) {

        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            //int targetTicks = (int) (2 * inches * countsPerInch);
            int targetTicks = (int) (inches * countsPerInch);

            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION

            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed/2);

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("FR current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FL current", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("BL current", BL.getCurrentPosition());
                OpModeReference.telemetry.addData("BR current", BR.getCurrentPosition());

                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setDriveMotorPower(double methodSpeed, double stepMult) {
        for (DcMotor m : AllMotors)
            m.setPower(methodSpeed * stepMult);
    }

    public void autoDucks(double seconds, double power) {
        DK.setPower(power);
        OpModeReference.sleep(Math.round(seconds*1000));
        DK.setPower(0);
    }

    public void ducks (boolean in, boolean out) {
        if (in && !out)
            DK.setPower(1);
        if (!in && out)
            DK.setPower(-1);
        if (!in && !out)
            DK.setPower(0);
    }

    public void driverControl () {

        double drive = OpModeReference.gamepad1.left_stick_y;
        double turn = OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper)
            movingSpeed = 0.1;
        else if (OpModeReference.gamepad1.right_bumper)
            movingSpeed = 1;
        else
            movingSpeed = 0.5;

        for (DcMotor l : LeftMotors)
            l.setPower(movingSpeed * (drive + turn));
        for (DcMotor r : RightMotors)
            r.setPower(movingSpeed * (drive - turn));
//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

}
