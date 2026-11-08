#include "auton.hpp"
#include "auton_selector.hpp"

void auton_steps() {
    close_claw();
    start_at(0, 0, 180);
    spin_motor(auton_toggle_degrees(), 200);
    move_to(0, 15.5, 100, true);

    robot::chassis().turnToHeading(90, 500, {}, false);
    move_to(20,16.5,40);
    lift_to(100,500,true);
    open_claw();
        /*
    move_to(0,15.5,100,true);
    robot::chassis().turnToHeading(90,3000);
    lift_to(240,1500);
    move_to(17.5,15.5 ,50);
    pros::delay(500); // Wait 0.1 seconds
    lift_to(0,1000);
    open_claw();
    move_to(0,17,50,true);
    robot::chassis().turnToHeading(50,1500, {}, false);
    pickup_cup(30,5000);
    robot::chassis().turnToHeading(153,1000);
    lift_to(360,1000,true);
    move_to(18,10,40,false,2000);
    lift_to(100,500,false);
    open_claw();
    
    move_to(25,-5 ,40);
    move_to(23,-5,20);
    close_claw();
    robot::chassis().swingToHeading(
    90, lemlib::DriveSide::RIGHT, 2000, {}, false);
    robot::chassis().turnToHeading(45,2000);
    lift_to(360,1500,true);
    move_to(20,17,100,false);
    open_claw();
    
    
    move_to(21,37,20,false,1000);
    close_claw();
    */
}
