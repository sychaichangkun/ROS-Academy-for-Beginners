##  Nodelet Everything

#  Nodelet Everything

节点对于从ROS中挤出更多的性能至关重要，特别是对于视觉，控制或其他高频和/或带宽应用。为什么不使一切都成为一个节点？在这个ROS教程中，您将学习如何将节点结合到代码中。

## 开始

编写节点友好的代码，从一开始就很容易。最好的事情就是建立一个可以独立运行的nodelet库和一个包装节点的可执行文件。您会注意到，由于pluginlib，您甚至不需要将您的节点可执行文件与CMakeLists中的nodelet库链接起来。请参阅下面的参考代码，大部分来自[https://github.com/clearpathrobotics/zbar_ros](https://github.com/clearpathrobotics/zbar_ros)。名称等都使用@（...）方案进行模板化。

## 但是 为什么？

主要优点是节点之间的自动零拷贝传输（在一个节点管理器中）。这意味着，由硬件驱动程序创建的点云不需要在命中代码之前进行复制或序列化，假设您将节点注入相机的管理器，从而节省您的时间和麻烦。

您获得节点的所有模块化，以及具有一个单一过程的所有效率。这使得节点比裸插件更灵活（通过pluginlib） - 您可以隐式地利用发生的任何进程内通信。

## 注意事项

零拷贝传输工作的要求是您使用ConstPtr回调进行订阅，发布后不要修改发布方的消息，请参阅下面的节点代码。

最早你可以得到一个NodeHandle在onInit（）方法中。不要尝试在构造函数中做任何ROS。如果一个节点掉线，整个管理器就会关闭。检查异常。

要遵守节点API，您不应手动管理线程。但是你总是使用ros :: Timer回调，就像你应该是反正的，对吧？来自Nodelet API的NodeHandles的回调由一个共享的线程池管理，这样就更有效率了。

ROS_DEBUG和朋友不再工作 - 使用等效的NODELET_DEBUG。遗憾的是，这排除了在节点和节点之间使用调试消息共享代码实现，这就是为什么我们在下面使用动态包装节点的原因。

您实际上并不需要包装节点 - 您可以使用nodelet独立的pkg / nodelet运行nodelet。但是这样做不能使用rosrun pkg节点，这是非常用户友好的。

现在我已经大致对你讲清楚了，请阅读其余的nodelet文档：

* [http://wiki.ros.org/nodelet](http://wiki.ros.org/nodelet)
* [http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing)

## 现在 code:


任何使用empy的人都可以使用以下@（...）格式。只需用您选择的字符串替换每个标签！

**Node Code**

	#include  “ros / ros.h”
	#include  “nodelet / loader.h”

	int main(int argc, char **argv){
		ros::init(argc, argv, "@(node)");
		nodelet::Loader nodelet;
		nodelet::M_string remap(ros::names::getRemappings());
		nodelet::V_string nargv;
		std::string nodelet_name = ros::this_node::getName();
		nodelet.load(nodelet_name, "@(package)/@(nodelet)", remap, nargv);
		ros::spin();
		return 0;
		}


**Nodelet COde**

	#include“ros / ros.h”
	#include“nodelet / nodelet.h” 
	namespace @(namespace)
	{

		class @(NodeletClass) : public nodelet::Nodelet
		 {
		 public:
		@(NodeletClass)();

		private:
		virtual void onInit(){
		nh = getNodeHandle();
		private_nh = getPrivateNodeHandle();
		timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(& @(NodeletClass)::timerCb, this, _1));
		sub_ = nh.subscribe("incoming_chatter", 10, boost::bind(& @(NodeletClass)::messageCb, this, _1));
		pub_ = private_nh.advertise<std_msgs::String>("outgoing_chatter", 10);
		};

		void timerCb(const ros::TimerEvent& event){
		// Using timers is the preferred 'ROS way' to manual threading
		NODELET_INFO_STREAM("The time is now " << event.current_real);
		}

		// must use a ConstPtr callback to use zero-copy transport
		void messageCb(const std_msgs::StringConstPtr message){

		// can republish the old message no problem, since we're not modifying it
		pub_.publish(message);

		std_msgs::String new_message;
		new_message.data = message.data + " fizz buzz";
		pub_.publish(new_message);

		// we can't modify any messages after they've been published, unless we want our subscribers to get VERY confused
		// new_message.data = "can't do this!";
		 }
		ros::Subscriber sub_;
		ros::Publisher pub_;
		ros::Timer timer_;
		};

		} // namespace @(namespace)

		PLUGINLIB_DECLARE_CLASS(@(package), @(NodeletClass), @(namespace)::@(NamespaceClass), nodelet::Nodelet);

---