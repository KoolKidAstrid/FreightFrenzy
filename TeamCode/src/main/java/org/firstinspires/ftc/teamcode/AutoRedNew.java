package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto Red New", group="Linear Opmode")
public class AutoRedNew extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

        boolean done = false;

        while (opModeIsActive() && !done) {

//        drive to wobble thing
            nyx.drive(-12, 0.5);

            int down = 1050;
//        raise head to wobble thing top layer
//        while (nyx.ARM2.getCurrentPosition() > down + 10 || nyx.ARM2.getCurrentPosition() < down - 10)
//            nyx.setArm2((float) down/1725f);
//        nyx.ARM2.setPower(0);

            nyx.setArmAuto(down);

//        deposit block
            nyx.intake(-1);
            sleep(3000);
            nyx.intake(0);

//        lower head
//        while (nyx.ARM2.getCurrentPosition() > 50 || nyx.ARM2.getCurrentPosition() < -50)
//            nyx.setArm2(0f/1725f);
//        nyx.ARM2.setPower(0);

            nyx.setArmAuto(0);

//        drive backwards
            nyx.drive(9, 0.5);

//        turn to carousel
            nyx.newTurn(-90);

//        drive to carousel
            nyx.drive(21, 0.5);

            nyx.newTurn(-45);

            nyx.drive(30,0.5);

            nyx.newTurn(135);

            nyx.drive(17, 0.5);
            nyx.drive(3, 0.25);

//        spin :)
            nyx.autoDucks(5, -0.25);

//        park
            nyx.drive(-17, 0.5);

            done = true;
        }

    }
}

