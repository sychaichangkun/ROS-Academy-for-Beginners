   #include <action_demo/DoDishesAction.h>  // Note: "Action" is appended
   #include <actionlib/server/simple_action_server.h>

   typedef actionlib::SimpleActionServer<action_demo::DoDishesAction> Server;

   void execute(const action_demo::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
   {
      // Do lots of awesome groundbreaking robot stuff here
     as->setSucceeded();
   }

  int main(int argc, char** argv)
   {
     ros::init(argc, argv, "dishes_Server");
    ros::NodeHandle n;
     Server server(n, "dishes", boost::bind(&execute, _1, &server), false);
    server.start();
     ros::spin();
    return 0;
   }
