# ROS 101 :一个实际例子
## 实际例子
现在你已经了解了基础知识，下面就介绍一下实际的例子。跟随着我们，看看我们如何真正“做”所有这些事情...

1. 首先，你需要运行Ubuntu，并安装ROS。为方便起见，您可以在这里下载我们易于使用的图像：[这里下载](https://s3.amazonaws.com/CPR_PUBLIC/LEARN_ROS/ROS_Edu.zip)。登录（用户名）：user，密码：learn 。
2. 获取VMWare Player，并使用上面的虚拟磁盘。如果您不想使用提供的图像，请按照本教程（安装Ubuntu 14.04之后）: [这里链接](wiki.ros.org/indigo/Installation/Ubuntu)。在其余的教程中，我们将参考ROS备忘单：[点击下载](http://bit.ly/1RCVMaB)。
3. 打开一个新的终端窗口（Ctrl + Alt + T）。在新终端中，键入roscore（然后按回车）这应该产生类似于以下内容。

![](https://i.loli.net/2017/08/16/59939cff59012.jpg)

刚刚完成的是如上所述启动ROS Master。我们现在可以尝试一些ROS命令。

打开一个新的**终端**，并输入**rostopic**。这将为您提供rostopic命令可以执行的所有选项的列表。

现在，我们对rostopic list 感兴趣。输入**rostopic list**。（并按enter键）。这应该给你一个窗口，如下所示：

![](https://i.loli.net/2017/08/16/59939d8774109.png)

上面列出的两个条目是ROS的内置方式来报告和聚合系统中的调试消息。我们想要做的是发布和订阅消息。

您可以再次打开一个新的终端，或在同一个终端窗口中打开一个新的选项卡（Ctrl + Shift + T）。在新终端中键入（注意：如果您尝试将其粘贴到终端中，则此命令将无法正常工作）：
**`rostopic pub / hello std_msgs / String“Hello Robot”`**

![](https://i.loli.net/2017/08/16/59939dfab7226.png)

我们来分解这个命令的部分。

* **rostopic pub** - 这命令ROS发布一个新主题。

* **/ hello** - 这是新主题的名称。（可以做任何你想要的）

* **std_msgs / String** - 这是主题类型。我们想发表一个字符串主题。在我们上面的概述示例中，它是一种图像数据类型。

* **“Hello Robot”** - 这是主题包含的实际数据。IE的消息本身。

回到以前的终端，我们可以再次执行rostopic列表。

我们现在有一个新的主题列出！我们也可以回应该主题，通过输入**rostopic echo / hello**来查看消息。

![](https://i.loli.net/2017/08/16/59939e6f6683c.png)

我们现在已经用消息成功发布了一个主题，并收到了这个消息。键入**Ctrl + C**停止回显/ hello主题。我们还可以查看发布消息的节点。键入**rosnode list**。你会得到一个类似于下面的列表。（左侧节点旁边的确切数字可能不同）

![](https://i.loli.net/2017/08/16/59939ece7ee45.png)

因为我们要求 rostopic 向我们发布/ hello主题，所以ROS已经开始创建了一个节点。我们可以通过键入rosnode info /rostopic/_.....**（无论什么数字）**来查看细节

**提示**：在ROS中，通常在Linux中，每当您开始输入内容时，您可以按Tab键自动完成。如果有多个条目，请双击Tab获取列表。在上面的例子中，我输入的是rosnode info / rost（TAB）

![](https://i.loli.net/2017/08/16/59939f56b8da3.jpg)

我们可以通过键入**`rostopic info / hello`**来获取我们的主题相同的信息

![](https://i.loli.net/2017/08/16/59939fb8c4c38.jpg)

您会注意到，“发布者：”下列出的节点是我们请求的信息相同的节点。

到目前为止，我们已经学习了涵盖ROS的基本原理，以及如何使用Rostopic，rosnode 等。

下一次，我们将编写一个简短的示例程序，并尝试一下。

---




