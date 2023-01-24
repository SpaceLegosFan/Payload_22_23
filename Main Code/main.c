// highest level code for stepper motor

int main()
{
    // set up the motor ...duh
    setup();

    // loop sensor data until deployment is needed
    loop();

    deployHor();
    orient();
    deployVert();

    // TODO: call RF functions
}
