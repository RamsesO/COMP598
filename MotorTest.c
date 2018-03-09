/*
motorA is the main chassis/tank. This is the transport. Encoded. + mean left. - means right.
motorB is the height controller/ pulley. Encoded. + means ????. - means ???.
motorC is the claw controller. + means open. - means close.
*/

int motorSpeed = 20;
int clawSpeed = 50;

void clawRoutineTest(){
        //Diable all driving
        setMotorSpeed(motorA, 0);

        //Lower claw and grab item
        //Increase degrees of rotation to increase distance
        //Use the circumfrence of wheel * # of rotations.
        moveMotorTarget(motorB, 720, clawSpeed);
        delay(3000);
        moveMotorTarget(motorC, 720, -clawSpeed);
        delay(3000);

        //Raise claw and drop item
        //Increase degrees of rotation to increase distance
        //Use the circumfrence of wheel * # of rotations.
        moveMotorTarget(motorB, 720, -clawSpeed);
        delay(3000);
        moveMotorTarget(motorC, 720, clawSpeed);
        delay(3000);

}

void returnToBase(){


        int stepsTaken = 1;

        while(stepsTaken != 0){
                //Get degrees traveled
                stepsTaken = getMotorEncoder(motorA);

                //Go left if we traveled right.
                //Go right if we traveled left
                if(stepsTaken > 0){
                        moveMotorTarget(motorA, stepsTaken, -motorSpeed);

                } else {
                        moveMotorTarget(motorA, stepsTaken, motorSpeed);
                }

        }

}

task main()
{
        resetMotorEncoder(motorA);
        while (true) {

                /*
                If the center button is pushed stop moving and enter the
                crane lifting routina. IT will lower the crane X degrees
                where 360 degrees = one full rotation.
                */
                if(getButtonPress(buttonEnter)){
                        //into claw routine
                        //Replace this with non test version in future
                        //IE just close claw. return to base. then open claw
                        clawRoutineTest();
                        returnToBase();
                        //We can only drive forward if
                        //the center button has not been pressed
                        } else {

                        //If the left button on the brick is pressed
                        //Drive left
                        if(getButtonPress(buttonLeft)){
                                setMotorSpeed(motorA, -motorSpeed);

                                //If the right button on the brick is pressed
                                // drive right
                                } else if(getButtonPress(buttonRight)){
                                setMotorSpeed(motorA, motorSpeed);

                                //Stop otherwise
                                } else {
                                setMotorSpeed(motorA, 0);
                        }

                }

        }

}