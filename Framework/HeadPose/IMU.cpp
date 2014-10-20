#include "IMU.h"


IMU_THREAD::IMU_THREAD(concurrency::ITarget<std::tuple<QuaternionValue,EulerAnglesStruct>> &target) :
	m_Sensor_Data(target)
{
	m_abort=false;
	m_continue=true;
	sensor=CSerial();
	sensor.Open(4,115200);
	dataProcessor=XbimuReceiver();
	a=new unsigned char;
	data = new QuaternionValue();
	data->Qw = 1;
	data->Qx = 0;
	data->Qy = 0;
	data->Qz = 0;
	quaternion = new Quaternion(1, 0, 0, 0);
	std::tuple<QuaternionValue, EulerAnglesStruct> new_data(*data, quaternion->getEulerAngles());
	
	asend(m_Sensor_Data, new_data);
}


IMU_THREAD::~IMU_THREAD(void)
{
	sensor.Close();

}

void IMU_THREAD::run()
{
	
	while (m_abort!=true )
	{
		if (m_continue)
		{
			int num =sensor.ReadData(a,1);
			//std::cout<<num;
			dataProcessor.processNewCharASCII(*a);
			if(dataProcessor.isQuaternionGetReady())
			{
				data=new QuaternionValue();
				
				dataProcessor.getQuatValues(Qw,Qx,Qy,Qz);

				quaternion=new Quaternion(Qw,Qx,Qy,Qz);
				
				//quaternion->getEulerAngles();
				data->Qw=dataProcessor.getQuaternion().w* 0.0001f;
				data->Qx=dataProcessor.getQuaternion().x* 0.0001f;
				data->Qy=dataProcessor.getQuaternion().y* 0.0001f;
				data->Qz=dataProcessor.getQuaternion().z* 0.0001f;
				std::tuple<QuaternionValue,EulerAnglesStruct> new_data(*data,quaternion->getEulerAngles());
				asend(m_Sensor_Data,new_data);
				//emit Sensor_Data_Available(*data,quaternion->getEulerAngles());
				delete quaternion;
				delete data;
			}  
		}
	}


}

void IMU_THREAD::endThreadExec()
{
	m_abort=false;
}
void IMU_THREAD::getNext(void)
{
	m_continue=true;
}
