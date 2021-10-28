package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto BLue", group="Linear Opmode")
public class AutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwaggyTheRobot swaggy = new SwaggyTheRobot(this);
        swaggy.initialize();
        waitForStart();

        swaggy.drive(-27, 0.5);
        swaggy.turn(-90, 0.35);
        swaggy.drive(-20, 0.5);
        swaggy.turn(-75, 0.35);
        swaggy.drive(-23, 0.5);
        for (DcMotor m : swaggy.AllMotors) {
            m.setPower(-0.05);
        }
        swaggy.autoDucks(3, 0.5);
        for (DcMotor m : swaggy.AllMotors) {
            m.setPower(0);
        }
        swaggy.drive(18, 0.5);


    }
}
