package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Warehouse", group="Linear Opmode")
public class AutoBlueWarehouse extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();


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

        nyx.drive(-2, 0.25);

//        turn to warehouse
        nyx.newTurn(90);

//        drive to warehouse
        nyx.drive(-72, 1);

    }
}
