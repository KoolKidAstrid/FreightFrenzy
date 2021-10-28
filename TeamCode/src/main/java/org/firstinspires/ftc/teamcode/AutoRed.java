package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Red", group="Linear Opmode")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwaggyTheRobot swaggy = new SwaggyTheRobot(this);
        swaggy.initialize();
        waitForStart();

        swaggy.drive(-27, 0.5);
        swaggy.turn(115, 0.35);
        swaggy.drive(-42, 0.5);
        for (DcMotor m : swaggy.AllMotors) {
            m.setPower(-0.05);
        }
        swaggy.autoDucks(3, -0.5);
        for (DcMotor m : swaggy.AllMotors) {
            m.setPower(0);
        }

    }
}
