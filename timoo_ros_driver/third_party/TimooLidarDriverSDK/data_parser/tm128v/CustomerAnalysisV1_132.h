#pragma once
#include "CustomerAnalysisBase.h"

namespace timoo {
namespace driver {


class Laser760;
class OnePointInfo;




#pragma pack(push, 1)
	struct DataUnit
	{
		uint16 Distance;
		// ������
		uint8 refelect;
		// ������Ϣ
		uint8 other;
	};
#pragma pack(pop)

#pragma pack(push, 1)
	struct DataSubject
	{
		// ��ʼ��־
		uint8 StartFlag;
		// ��ֱ������
		int16 VerticalCompensationAngle;
		// ˮƽ������
		int16 HorizontalCompensationAngle;
		// ���ݵ�Ԫ
		DataUnit dataUnit[192];
	};
#pragma pack(pop)

#pragma pack(push, 1)
	struct ScanResearchV1_132
	{
		// ��ͷ��Ϣ
		PackageHead package_head;
		// ����ͷ��Ϣ
		DataHeadInfo data_head_info;
		// ���ݿ���Ϣ
		DataBlockInfo data_block_info;
		// ʱ����Ϣ
		ClockInfo clock_info;
		// Ԥ��
		uint8 reserve[16];
		// ��Ϣ��־
		InfoFlag info_flag;
		// ��������
		DataSubject data_subject[1];
		// ֡β
		uint8 tall[2];
	};
#pragma pack(pop)

	class CustomerAnalysisV1_132 : public CustomerAnalysisBase
	{
	private:

	public:
		CustomerAnalysisV1_132();
		~CustomerAnalysisV1_132();
#ifdef USE_VECTOR
		void getPoints(std::vector<uint8>& buf, std::vector<std::vector<OnePointInfo>>& out_Points) override;
#else
		void getPoints(std::vector<uint8>& buf, OnePointInfo**& out_Points, PointSizeCn& out_size, 
					   std::vector<base::TimooPoint>& block_points) override;
#endif // USE_VECTOR
	};


} // namespace driver
} // namespace timoo

