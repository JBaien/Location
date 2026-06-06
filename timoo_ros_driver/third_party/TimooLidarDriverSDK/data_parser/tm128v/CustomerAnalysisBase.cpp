#include "CustomerAnalysisBase.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>

namespace timoo {
namespace driver {

CustomerAnalysisBase::CustomerAnalysisBase()
{
#ifdef USE_VECTOR

	// 缓存第一重回波
	//EchoPoints.push_back(std::vector<OnePointInfo>());
	// 缓存第二重回波
	//EchoPoints.push_back(std::vector<OnePointInfo>());
#else
	// 缓存第一重回波
	EchoPoints[0] = new OnePointInfo[LIDAR_LINE_MAX * ONELINE_POINTNUM];
	// 缓存第二重回波
	EchoPoints[1] = new OnePointInfo[LIDAR_LINE_MAX * ONELINE_POINTNUM];
	// 缓存第一重回波 有效值索引
	EchoPointsIndex[0] = new size_t[LIDAR_LINE_MAX * HANGLE_MAX];
	// 缓存第二重回波 有效值索引
	EchoPointsIndex[1] = new size_t[LIDAR_LINE_MAX * HANGLE_MAX];
#endif // USE_VECTOR


}
CustomerAnalysisBase::~CustomerAnalysisBase()
{
#ifdef USE_VECTOR
#else
	delete[] EchoPoints[0];
	delete[] EchoPoints[1];
	delete[] EchoPoints;

	delete[] EchoPointsIndex[0];
	delete[] EchoPointsIndex[1];
	delete[] EchoPointsIndex;
#endif // USE_VECTOR


}

// 解析数据协议
void CustomerAnalysisBase::analysis(std::vector<uint8>& buf, std::vector<base::TimooPoint>& block_points, bool& is_last_packet)
{
	if (size == buf.size())
	{

		if ((buf[0] != 0xff) || (buf[1] != 0xcc)) {
			static int buf_count = 0;
			std::cout << buf_count++ << "--------------------------------analysis buf (hex): 0x";
			std::cout << std::hex << static_cast<int>(buf[0]);
			std::cout << std::hex << static_cast<int>(buf[1]);
			std::cout << std::dec << std::endl;  // 恢复十进制输出
			std::cout << "invalid udp data packet" << std::endl;
		}



		DataMainPre* data_main_pre = (DataMainPre*)buf.data();

		// 检测是否还是当前圈
		if (data_main_pre->data_head_info.CircleNumber == nextCircleNo)
		{
			// 调用协议解析函数 解析当前数据中的所有点的数据
			getPoints(buf, EchoPoints, EchoPointsSize, block_points); // 调用相应的解析接口
		}
		else
		{
			// 圈号不一样的表示 现在是新的一圈的数据 此时应该发布上一圈的所有点

			is_last_packet = true;
			// 获取应当接受的一圈点云的总点数
			size_t TotalPoint = data_main_pre->data_head_info.TotalPackages * data_main_pre->data_block_info.EachValidDataBlockNum * data_main_pre->data_block_info.DataBlockRowsNum * data_main_pre->data_block_info.DataBlockColsNum;
			// 限制收满一圈的点云才进行点云发布 最多允许丢失一个方位角的数据

			if ((EchoPointsSize.firstSize >= (TotalPoint - 192)) || (EchoPointsSize.LastSize >= (TotalPoint - 192)))
			{
				// // 输出一圈的点云
				// // 调用回调输出点云
				// //[TODO....]


				// // 例子：
				// int size = 0;
				// int nozerosize = 0;
				// std::string filename = "./data" + std::to_string(data_main_pre->data_head_info.CircleNumber) + ".txt";
				// std::ofstream outFile(filename);
				// // 检查文件是否成功打开
				// if (!outFile.is_open()) {
				// 	std::cerr << "无法打开文件" << std::endl;
				// 	return;
				// }
				// for (int line = 1; line <= 192; line++)
				// {
				// 	for (int angle = 0; angle <= 12030; angle += 15)
				// 	{
				// 		OnePointInfo& tem = getData(1, line, angle);
				// 		if (tem.line != 0) size++;// 如果获取的点的线号为0 则表示这个点是无效的点	
				// 		if (tem.distance_mm != 0) nozerosize++;
				// 		outFile << 
				// 			tem.index << "," <<
				// 			tem.x << "," << 
				// 			tem.y << "," << 
				// 			tem.z << "," << 
				// 			tem.distance_mm << "," <<
				// 			tem.distance_m << "," <<
				// 			(int)tem.reflectivity
				// 			<< std::endl;
				// 	}
				// }
				// outFile.close();
				// assert(size == EchoPointsSize.firstSize);

				// std::cout << "Publish point cloud..." << std::endl;
				

			}
			
			nextCircleNo = data_main_pre->data_head_info.CircleNumber;
			// 当前 收到的数据进行解析
			// 清空上一圈的点云
			clear();
			
			getPoints(buf, EchoPoints, EchoPointsSize, block_points);
			
		}
	}
}



} // namespace driver
} // namespace timoo

