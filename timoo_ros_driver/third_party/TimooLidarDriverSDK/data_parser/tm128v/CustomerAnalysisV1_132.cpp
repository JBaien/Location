#include "CustomerAnalysisV1_132.h"
#include <cmath>   

namespace timoo {
namespace driver {
// using namespace ScanV1_132;

static int getRegion_760A(int line) {
	if (line >= 1 && line <= 24)
		return 8;
	if (line >= 25 && line <= 48)
		return 7;
	if (line >= 49 && line <= 72)
		return 6;
	if (line >= 73 && line <= 96)
		return 5;
	if (line >= 97 && line <= 120)
		return 4;
	if (line >= 121 && line <= 144)
		return 3;
	if (line >= 145 && line <= 168)
		return 2;
	if (line >= 169 && line <= 192)
		return 1;
	return 0;
}

// ���ݵ��ת�ټ�������ĽǶ�ƫ��
static int get_time_sequence_Offset_h_angle_760A(int speed, int line)
{
	int Region = getRegion_760A(line);
	if (Region == 1 || Region == 7)
		return 6000 * speed * (7 * 3) / 1000000;
	if (Region == 3 || Region == 5)
		return 6000 * speed * (14 * 3) / 1000000;
	if (Region == 4 || Region == 6)
		return 6000 * speed * (12 * 3) / 1000000;
	if (Region == 2 || Region == 8)
		return 6000 * speed * (19 * 3) / 1000000;
	return 0;
}



CustomerAnalysisV1_132::CustomerAnalysisV1_132()
	:CustomerAnalysisBase()
{
	size = sizeof(ScanResearchV1_132);
}
CustomerAnalysisV1_132::~CustomerAnalysisV1_132()
{

}

// ʹ�� std::vector �ή�ͳ������� ���齫out_Points�ĳ� ��ǰ���ٵ� 192 * 960 ��С��һά���� ֱ�ӽ�id��Ϊ���� ���Դ�����������  
#ifdef USE_VECTOR
void CustomerAnalysisV1_132::getPoints(std::vector<uint8>& buf, std::vector<std::vector<OnePointInfo>>& out_Points)
#else
void CustomerAnalysisV1_132::getPoints(std::vector<uint8>& buf, OnePointInfo**& out_Points, PointSizeCn& out_size, 
					   					std::vector<base::TimooPoint>& block_points)
#endif //  USE_VECTOR
{
	
	ScanResearchV1_132* scan = (ScanResearchV1_132*)buf.data();
	// ��ȡˮƽ�Ƕȷֱ���
	int hr = scan->data_block_info.HorizontalAngleResolution; // ����100��
	// ��ȡ��ֱ�Ƕȷֱ���
	int vr = scan->data_block_info.VerticalAngleResolution;
	// ��ȡ����ֱ���
	int disr = scan->data_block_info.RangeResolution;
	// ÿ����Ч���ݿ�����
	int block_num = scan->data_block_info.EachDataBlockNum;
	// ÿ�����ݿ�ͨ������
	int rows_num = scan->data_block_info.DataBlockRowsNum;
	// ÿ�����ݿ�ˮƽ�ǵ�����
	int cols_num = scan->data_block_info.DataBlockColsNum;
	// ���ӱ��
	int mirror_no = scan->data_block_info.MirrorNo;
	//scan->data_block_info.MirrorNo = 2;

    if ((buf[0] != 0xff) || (buf[1] != 0xcc)) {
		std::cout << "invalid udp data packet" << std::endl;
		return;
	}

	if (rows_num != 192) { return; }
	
	if (cols_num != 1) { return; }


	// ���ݵ�ǰЭ����ܰ�����ȡ�״�ƫ�ƶ��� ʹ�õ������״��ӳ��м�
	int EchoMultiplicity = scan->data_head_info.EchoMultiplicity == 0 ? 1 : scan->data_head_info.EchoMultiplicity;
	// ���㵱ǰ���Ƶ��ܽǶ� Ӧ���յ����ܰ��� * ÿ������Ч���ݿ����� / �ز��� * ˮƽ�Ƕȷֱ���
	int total_angle = scan->data_head_info.TotalPackages * scan->data_block_info.EachValidDataBlockNum / EchoMultiplicity * hr;
	int office_yaw = total_angle / 1000 / 2; // ������Ҫ����ƫ�ƵĽǶ� һ�������Ϊ60��


	// ������ǰ���ǵ�һ�ػز��㻹�ǵڶ��ػز���
	int EchoType = 1; // 1:��һ�ػز� 2:�ڶ��ػز�
	// ����Э������Ч���ݿ��������б���
	for (int i = 0; i < block_num; i++)
	{
		// ˮƽ�Ƕȵ���id
		int h_angle_id = (scan->data_head_info.PackageSerialNumber - 1) * scan->data_head_info.EchoMultiplicity + i;
		if (scan->data_head_info.EchoMultiplicity == 2)
		{
			h_angle_id = (scan->data_head_info.PackageSerialNumber - 1);
			if (scan->data_block_info.EachDataBlockNum == 1)
			{
				h_angle_id = h_angle_id / 2;
				// ��������ж��ǵ�һ�ػ��ǵڶ��� ���Ϊ����Ϊ��һ�� ���ż��Ϊ�ڶ���
				if (scan->data_head_info.PackageSerialNumber % 2 == 0)
				{
					EchoType = 2;
				}
				else
				{
					EchoType = 1;
				}
			}
			else
			{
				if ((i + 1) % 2 == 0)
					EchoType = 2;
			}
		}

		// ˮƽ�Ƕȼ��� id * �ֱ��� / 10.0f
		int src_h_angle = h_angle_id * hr / 10.0f; // ��ֵΪ 0.01 * 100
		// ˮƽ�� 
		float h_angle = src_h_angle / 100.0f;
		// ��ȡЭ���еĴ�ֱ�ǶȺ�ˮƽ������ ��ʵ�ʴ���������ԶΪ0��
		float v_compensate_angle = scan->data_subject[i].VerticalCompensationAngle / 1000.0f;
		float h_compensate_angle = scan->data_subject[i].HorizontalCompensationAngle / 1000.0f;


		for (int j = 0; j < rows_num; j++)
		{

			// ����Ψһid��
			uint32 id = create_id(j, h_angle_id);//Id + k;
			bool old = false;
#ifdef USE_VECTOR
			OnePointInfo tem;
#else
			
	// std::cout << "v-2  "  << rows_num << std::endl;
			// if (h_angle_id > 0) {
			// std::cout << "EchoType: " << EchoType << ", j:" << j << ", h_angle_id:" << h_angle_id 
			// 		  << ", src_h_angle:" << src_h_angle << ", old:" << old << std::endl;
			// }
			OnePointInfo& tem = create_unsafe(EchoType - 1, j, h_angle_id, src_h_angle, old);
			
	// std::cout << "v-3" << std::endl;
#endif // !USE_VECTOR

			tem.index = id;
			// ��ȡ�ߺ�
			tem.line = j + 1;
			// ��ȡ��id
			tem.line_id = h_angle_id;
			// ��ȡȦ��
			//tem.circle_no = scan->data_head_info.CircleNumber;
			// ��ȡת����
			tem.Mirror = mirror_no;
			// ��ȡͨ����
			//tem.Channal = tem.line;
			// ��ȡ�ز�����
			tem.EchoType = EchoType;
			// ��ȡʱ���
			tem.Time = (std::uint64_t)scan->clock_info.TimeStampSec * (std::uint64_t)1000 + (std::uint64_t)scan->clock_info.TimeStampNanoSec / (std::uint64_t)1000000;
			// ��ȡԴ��ֱ��
			//tem.SrcVAngle = tem.device_model != 1 ? VerticalChannels[tem.line - 1] : Single459_VerticalChannels[tem.line - 1];
			// ��ȡԴˮƽ��
			tem.SrcHAngle = src_h_angle;
			// ��ȡ��ֱ��
			tem.PointVAngle = Single459_VerticalChannels[tem.line - 1];
			tem.PointVAngle += v_compensate_angle;
			// ��ȡˮƽ��  ����ʱ����ת��ƫ��
			float Offset_h_angle = get_time_sequence_Offset_h_angle_760A(scan->data_block_info.MotorSpeed / 100, tem.line);
			
	// std::cout << "v-3" << std::endl;
			tem.PointHAngle = h_angle + Offset_h_angle / 1000.0f; // ����ˮƽ�Ƕ�
			// ��ȡ����
			tem.distance_mm = scan->data_subject[i].dataUnit[j].Distance * disr;
			tem.distance_m = tem.distance_mm / 1000.0f;
			// ��ȡ������
			tem.reflectivity = scan->data_subject[i].dataUnit[j].refelect;
			// ��ȡ���ת��
			tem.motor_speed = scan->data_block_info.MotorSpeed;
			// ����������Ǻ���ֵ��ǰ�洢���������Է�����ڵ����˲������¼���XYZ �����ظ�����
			float cosRidCos = cos(DegreeToRadian * tem.PointVAngle) * cos(DegreeToRadian * (tem.PointHAngle - office_yaw));
			float cosRideSin = cos(DegreeToRadian * tem.PointVAngle) * sin(DegreeToRadian * (tem.PointHAngle - office_yaw));
			float sinRidThousandth = sin(DegreeToRadian * tem.PointVAngle);
			tem.CosRidCos = cosRidCos;
			tem.CosRideSin = cosRideSin;
			tem.SinRidThousandth = sinRidThousandth;
			tem.x = tem.distance_m * cosRideSin;
			tem.y = tem.distance_m * cosRidCos;
			tem.z = tem.distance_m * sinRidThousandth;

			tem.confidence = scan->data_subject[i].dataUnit[j].other;

			base::TimooPoint timoo_point;
			timoo_point.x = tem.x;
			timoo_point.y = tem.y;
			timoo_point.z = tem.z;
			timoo_point.ring_id = tem.line;
			timoo_point.intensity = tem.reflectivity;
			timoo_point.distance = tem.distance_m;
			timoo_point.angle = tem.PointVAngle;
			timoo_point.azimuth = tem.SrcHAngle;
			timoo_point.time = tem.Time;
			// block_points.push_back(timoo_point);
			if (tem.line > 127) {
				if (tem.line % 2 == 0) {block_points.push_back(timoo_point); }
			} else if (tem.line < 65) {
				if (tem.line % 2 == 1) { block_points.push_back(timoo_point); }
			} else {
				block_points.push_back(timoo_point);
			}

#ifdef USE_VECTOR
			// ���ݻز����ʹ�ŵ���Ӧ�Ļ�����
			if (EchoType == 1)
				out_Points[0].push_back(tem);
			else
				out_Points[0].push_back(tem);
#else
			if (!old)
			{
				if (EchoType == 1)
					out_size.firstSize++;
				else
					out_size.LastSize++;
			}
#endif // USE_VECTOR


		}
	}
}

} // namespace driver
} // namespace timoo

