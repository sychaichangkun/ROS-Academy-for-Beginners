   #include <action_demo/DoDishesAction.h> // Note: "Action" is appended
   #include <actionlib/client/simple_action_client.h>

   typedef actionlib::SimpleActionClient<action_demo::DoDishesAction> Client;

   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "dishes_Client");
     Client client("dishes", true); // true -> don't need ros::spin()
    client.waitForServer();
   action_demo::DoDishesGoal goal;
   // Fill in goal here
    client.sendGoal(goal);
   client.waitForResult(ros::Duration(5.0));
   if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
   }
