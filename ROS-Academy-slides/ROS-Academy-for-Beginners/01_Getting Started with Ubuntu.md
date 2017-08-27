# ROS 101 :Ubuntu入门
## 初学者>>Ubuntu入门
### UBUNTU入门
要了解如何使用ROS，我们必须学习Ubuntu的基础知识！Ubuntu是一个基于Linux的开源操作系统。开发的大部分将使用终端进行。您可以通过单击终端图标或是在开始菜单中键入“终端”来打开终端窗口。

![图1](https://i.loli.net/2017/08/15/5992a7489d923.jpg)

要使用终端，只需要在窗口中输入命令并且按“进入”即可。在终端工作时，您当前所在的目录成为工作目录，并且显示在终端行的开头。下面有一些简单的常用命令：

| 命令                | 描述                         |
| --------------------|:----------------------------:|
| LS                  | 列出文件和文件夹    |
| cd 'folder'         | 将工作目录更改为 'folder'     |
| pwd                 | 打印当前工作目录               |
|cp'src''dest'        | 将'src'复制到'dest'          |
|sudo     |以root用户身份执行命令|
|mkdir 'directory'  |在名为'directory'的工作目录中创建一个目录|
|gedit或nano 'file' |打开一个文本编辑器来编辑<文件>|

Ubuntu程序从存储库安装。允许通过apt系统非常容易的安装程序和应用程序。要安装程序，只需要键入：**`sudo apt-get install`**

ROS包将被命名为ros。例如，要安装Clearpath Robotics Indigo Husky软件包，您可以键入：**`sudo apt-get install ros-indigo-husky-desktop`**

Ubuntu的一个非常方便的功能是选项卡自动完成！在输入命令时，按Tab键完成命令的其余部分。如果可能需要多个命令可用于完成您的行，请双击选项卡键列出可能的选项。

您现在应该拥有在您机器上安装ROS所需的工具！详情可以登录[ROS Wiki](http://wiki.ros.org/)

