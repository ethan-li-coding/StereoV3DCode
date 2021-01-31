// StereoV3DCode.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <string>

#include "Camera.h"

std::string introduction = "> 亲爱的同学们，我们的世界是3D世界，我们的双眼能够观测三维信息，帮助我们感知距离，导航避障，从而翱翔于天地之间。而当今世界是智能化的世界，我们的科学家们探索各种机器智能技术，让机器能够拥有人类的三维感知能力，并希望在速度和精度上超越人类，比如自动驾驶导航中的定位导航，无人机的自动避障，测量仪中的三维扫描等，都是高智机器智能技术在3D视觉上的具体实现。\n\n立体视觉是三维重建领域的重要方向，它模拟人眼结构用双相机模拟双目，以透视投影、三角测量为基础，通过逻辑复杂的同名点搜索算法，恢复场景中的三维信息。它的应用十分之广泛，自动驾驶、导航避障、文物重建、人脸识别等诸多高科技应用都有它关键的身影。\n\n本课程将带大家由浅入深的了解立体视觉的理论与实践知识。我们会从坐标系讲到相机标定，从被动式立体讲到主动式立体，甚至可能从深度恢复讲到网格构建与处理，感兴趣的同学们，来和我一起探索立体视觉的魅力吧！\n\n本课程是电子资源，所以行文并不会有太多条条框框的约束，但会以逻辑清晰、浅显易懂为目标，水平有限，若有不足之处，还请不吝赐教！\n\n个人微信：EthanYs6，加我申请进技术交流群 StereoV3D，一起技术畅聊。\n\nCSDN搜索：Ethan Li 李迎松，查看网页版课程。\n\n随课代码，将上传至github上，地址：https://github.com/ethan-li-coding/StereoV3DCode";
int main()
{
	std::cout << introduction << std::endl;
	
    return 0;
}

