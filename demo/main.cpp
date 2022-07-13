/**
 * CN:
 * 主程序入口
 * 1.将SDK文件夹拷贝到自己的工程源码的当前目录
 * 2.引用standard_interface.h头文件
 *
 * EN:
 * main program entry
 * 1. Copy the files of the specified platform and the common basic files (data.h, parse.cpp.user.cpp) under the SDK to the current directory of your own project source code
 * 2. Reference data.h,udp_linux.h header file
 */

#include <stdio.h>
#include <string.h>
#include "../sdk/standard_interface.h"

//传入回调指针的方式打印
void CallBackMsg(int msgtype, void* param)
{
	//实时雷达数据返回
	if (msgtype == 1)
	{
		PointData* pointdata = (PointData*)param;
		for (int i = 0; i < pointdata->N; i++)
		{
			INFO_PR("%.5f\t%.3f\t%d\n", pointdata->points[i].angle, pointdata->points[i].distance, pointdata->points[i].confidence);
		}
	}
	//获取雷达时间戳打印信息
	else if (msgtype == 4)
	{
		DevTimestamp* devtimestamp = (DevTimestamp*)param;
		INFO_PR("timestamp:lidar_ip:%s lidar_port:%d time:%d delay:%d\n", devtimestamp->ip, devtimestamp->port, devtimestamp->timestamp, devtimestamp->delay);
	}
}


//打印
void PrintMsg(int msgtype, void *param)
{

	//获取硬件设备的配信息   当前串口版本仅返回序列号，其他信息为空
	if (msgtype == 2)
	{
		EEpromV101 *eepromv101 = (EEpromV101 *)param;
		//类型，编号，序列号
		char tmp_sn[20] = {0};
		memcpy(tmp_sn, eepromv101->dev_sn, sizeof(eepromv101->dev_sn) - 1);
		char tmp_type[16] = {0};
		memcpy(tmp_type, eepromv101->dev_type, sizeof(eepromv101->dev_type) - 1);

		INFO_PR("dev info: 设备编号:%d\t 序列号:%s\t 类型:%s\n", eepromv101->dev_id, tmp_sn, tmp_type);
		// ip地址 子网掩码 网关地址 默认目标IP  默认目标udp端口号  默认UDP对外服务端口号
		char tmp_IPv4[16] = {0};
		char tmp_mask[16] = {0};
		char tmp_gateway[16] = {0};
		char tmp_srv_ip[16] = {0};

		sprintf(tmp_IPv4, "%d.%d.%d.%d", eepromv101->IPv4[0], eepromv101->IPv4[1], eepromv101->IPv4[2], eepromv101->IPv4[3]);
		sprintf(tmp_mask, "%d.%d.%d.%d", eepromv101->mask[0], eepromv101->mask[1], eepromv101->mask[2], eepromv101->mask[3]);
		sprintf(tmp_gateway, "%d.%d.%d.%d", eepromv101->gateway[0], eepromv101->gateway[1], eepromv101->gateway[2], eepromv101->gateway[3]);
		sprintf(tmp_srv_ip, "%d.%d.%d.%d", eepromv101->srv_ip[0], eepromv101->srv_ip[1], eepromv101->srv_ip[2], eepromv101->srv_ip[3]);

		INFO_PR("dev info: ip地址:%s 子网掩码:%s 网关地址:%s 默认目标IP:%s  默认目标udp端口号:%d   默认UDP对外服务端口号:%d\n",
				tmp_IPv4, tmp_mask, tmp_gateway, tmp_srv_ip, eepromv101->srv_port, eepromv101->local_port);

		/*char tmp_ranger_bias[8] = {0};
		memcpy(tmp_ranger_bias, eepromv101->ranger_bias, sizeof(eepromv101->ranger_bias) - 1);*/
		//转速 ,电机启动参数,FIR滤波阶数，圈数，分辨率，开机自动上传，固定上传，数据点平滑，去拖点，记录校正系数，网络心跳，记录IO口极性
		INFO_PR("dev info: 转速:%d 电机启动参数:%d FIR滤波阶数:%d 圈数:%d  分辨率:%d   开机自动上传:%d 固定上传:%d  数据点平滑:%d 去拖点:%d   网络心跳:%d  记录IO口极性:%d\n",
				eepromv101->RPM, eepromv101->RPM_pulse, eepromv101->fir_filter, eepromv101->cir, eepromv101->with_resample, eepromv101->auto_start,
				eepromv101->target_fixed, eepromv101->with_smooth, eepromv101->with_filter, eepromv101->net_watchdog, eepromv101->pnp_flags);
	}
	//设置硬件配置数据，如果成功，则打印OK    S_PACK
	else if (msgtype == 3)
	{
		DevData *devdata = (DevData *)param;
		for (int i = 0; i < sizeof(devdata->set) - 1; i++)
		{
			char buf[3] = {0};
			if (devdata->set[i] == '1')
			{
				memcpy(buf, devdata->result + 2 * i, 2);
				INFO_PR("DEV SET  index:%d  result:%s\n", i, buf);
			}
		}
	}
}

//ip以及雷达地址拼接
void strCOnvert(char*ip,int port,char*result)
{
    std::string s = ip;
    char tmp[]="000.000.000.000 00000";
    int a[4] = { 0 };
    for (int i = 0; i < 4; i++)
    {
        int tmp=s.find('.', 0);
        std::string str = s.substr(0, tmp);
        a[i] = atoi(str.c_str());
        s = s.substr(tmp+1);
    }
    sprintf(result,"%03d.%03d.%03d.%03d %05d",a[0], a[1], a[2], a[3], port);
}
int main(int argc, char **argv)
{
	/*
	 * 1,params init
	 */
	if (argc != 2 && argc != 15)
	{
		DEBUG_PR("Incorrect number of parameters  %d\n usage : ./demo  ../../config/xxx.txt  or  ./demo    arg1 arg2  ..\n", argc);
		return ARG_ERROR_NUM;
	}
	RunConfig cfg;
	memset(&cfg, 0, sizeof(RunConfig));
	//传入配置文件的方式
	if (argc == 2)
	{
		const char *cfg_file_name = argv[1];
		if (!read_config(cfg_file_name, cfg))
		{
			DEBUG_PR("config file is not exist:%s\n", cfg_file_name);
			return ARG_ERROR_FILE_EXIST;
		}
	}
	//手动传入参的方式
	else if (argc > 2)
	{
		strcpy(cfg.type, argv[1]);
		strcpy(cfg.port, argv[2]);
		cfg.baud_rate = atoi(argv[3]);
		strcpy(cfg.lidar_ip, argv[4]);
		cfg.lidar_port = atoi(argv[5]);
		cfg.local_port = atoi(argv[6]);
		cfg.data_bytes = atoi(argv[7]);
		cfg.unit_is_mm = atoi(argv[8]);
		cfg.with_confidence = atoi(argv[9]);
		cfg.with_chk = atoi(argv[10]);
		cfg.with_smooth = atoi(argv[11]);
		cfg.with_deshadow = atoi(argv[12]);
		cfg.resample = atoi(argv[13]);
		cfg.rpm = atoi(argv[14]);
		cfg.output_scan = atoi(argv[15]);
		cfg.output_360 = atoi(argv[16]);
		cfg.from_zero = atoi(argv[17]);
		strcpy(cfg.output_file, argv[18]);
		cfg.is_group_listener = atoi(argv[19]);
		strcpy(cfg.group_ip, argv[20]);
		cfg.service_port = atoi(argv[21]);
		cfg.is_open_service = atoi(argv[22]);
	}

	//启动雷达设备 返回值非0表示失败  0表示可以正常接收包数据
	int res = openDev(cfg);
	if (res != 0)
	{
		return res;
	}
	if (cfg.is_open_service)
		OpenLocalService(cfg);

	//传入打印函数指针
	cfg.callback = CallBackMsg;
	printf("Please control it through a browser and enter the default address: http://localhost:8888\n");
	while (1)
	{

	}

	//while (1)
	//{
	//	//设置运行前的参数
	//	//仅网络款存在以下功能，串口版本不存在
	//	if (strcmp(cfg.type, "udp")==0)
	//	{
	//		//正常模式  收发一体，不需要设置组播模式请屏蔽is_group_listener=1 or 2
	//		//设置雷达的全局参数，成功则打印OK，失败则打印NG;这里为全部的接口，一般不需要用到
	//		if (cfg.is_group_listener == 0)
	//		{				
	//			DevData tmpData;
	//			memset(&tmpData, 0, sizeof(DevData));
	//			// tmpData.RPM = 600;
	//			// tmpData.ERR = -88;
	//			//strcpy(tmpData.UDP, "192.168.000.110 255.255.255.000 192.168.000.001 06543"); //修改雷达IP地址 子网掩码 网关(三位补0) 端口号(五位补0)
	//			
	//			//char DST[23]={0};
	//			//转成雷达标准格式
	//			//strCOnvert(cfg.group_ip,cfg.local_port,DST);
	//			// strcpy(tmpData.DST, DST);								  //接收主机的地址以及端口号   组播模式地址
	//			
	//			// strcpy(tmpData.NSP, "LSS-40S-B20E441");										  //设备型号
	//			// strcpy(tmpData.UID, "LH4902210610006");//设备序列号
	//			// tmpData.FIR=3;//滤波圈数
	//			// tmpData.PUL=2000;//电机启动脉冲
	//			// tmpData.VER=1000;//版本号
	//			// tmpData.PNP=1;	//IO类型
	//			// tmpData.SMT=1;//数据平滑
	//			// tmpData.DSW=1;//去拖点
	//			// tmpData.DID=9;//编号
	//			// tmpData.ATS=1;//开机自动上传
	//			// tmpData.TFX=0;//固定上传
	//			// tmpData.PST=3;//数据上传类型
	//			// memset(tmpData.set, 0, sizeof(tmpData.set));

	//			//需要调用到那些功能，自行去掉注释并且设置参数，具体的取值范围查看data.h文件的结构体

	//			// memcpy(tmpData.set, "1", 1);	 // RPM设置
	//			// memcpy(tmpData.set + 1, "1", 1); //偏差设置
	//			//memcpy(tmpData.set + 2, "1", 1); // UDP设置
	//			// memcpy(tmpData.set + 3, "1", 1); //接收端设置   特殊说明:需要与固定上传一起使用，否则可能不生效
	//			// memcpy(tmpData.set + 4, "1", 1); //设备型号开关
	//			// memcpy(tmpData.set + 5, "1", 1); //设备序列号开关
	//			// memcpy(tmpData.set + 6, "1", 1); //滤波圈数
	//			// memcpy(tmpData.set + 7, "1", 1); //电机启动脉冲
	//			// memcpy(tmpData.set + 8, "1", 1); //版本号
	//			// memcpy(tmpData.set + 9, "1", 1); //IO类型
	//			// memcpy(tmpData.set + 10, "1", 1); //数据平滑
	//			// memcpy(tmpData.set + 11, "1", 1); //去拖点
	//			// memcpy(tmpData.set + 12, "1", 1); //编号
	//			// memcpy(tmpData.set + 13, "1", 1); //开机自动上传
	//			// memcpy(tmpData.set + 14, "1", 0); //固定上传
	//			// memcpy(tmpData.set + 15, "1", 1); //数据上传类型
	//			// int res = SetDevInfo_extre(cfg.thread_ID[1], tmpData);
	//			// if (res == 0)
	//			// {
	//			// 	PrintMsg(3,&tmpData);
	//			// }
	//		}
	//		//组播模式发送端发送模式
	//		if (cfg.is_group_listener == 2)
	//		{
	//			DevData tmpData;
	//			char tmp[23]={0};
	//			//转成雷达标准格式
	//			strCOnvert(cfg.group_ip,cfg.local_port,tmp);
	//			strcpy(tmpData.DST, tmp);//组播地址加端口
	//			tmpData.TFX = 1; //固定上传
	//			memcpy(tmpData.set + 3, "1", 1);//设置固定上传的ip 网关 端口
	//			memcpy(tmpData.set + 14, "1", 1); //固定上传
	//			SetDevInfo_extre(cfg.thread_ID[1], tmpData);
	//		}
	//		//组播模式接收端，无需任何操作,配置文件调整该参数即可
	//		if (cfg.is_group_listener == 1)
	//		{
	//		}

	//		//设置时间戳打印
	//		//GetLidarTimestamp(cfg.thread_ID[1], true);
	//	}
	//	//获取设备配置信息
	//	 //EEpromV101 data;
	//	 //if (GetDevInfo(cfg.thread_ID[1], data) == 0)
	//	 //{
	//	 //	PrintMsg(2, &data);
	//	 //}
	//	//打印雷达数据，传入打印数据的函数指针   根据CallBackMsg来打印
	//	//getLidarData(cfg.thread_ID[1],true);
	//	while (1)
	//	{
	//		printf("\n=======================================\n");
	//		printf("input 'start' to start LiDAR\n");
	//		printf("input 'stop' to stop LiDAR\n");
	//		printf("input 'restart' to  restart LiDAR\n");
	//		printf("input 'fix' to fix distance LiDAR\n");
	//		printf("input 'exit' to disconnect\n");
	//		printf("=======================================\n");
	//		char line[80];
	//		fgets(line, 80, stdin);
	//		//启动雷达
	//		if (strncmp(line, "start", 5) == 0)
	//		{
	//			ControlDrv(cfg.thread_ID[1],"LSTARH");
	//			continue;
	//		}
	//		//停止雷达
	//		else if (strncmp(line, "stop", 4) == 0)
	//		{
	//			ControlDrv(cfg.thread_ID[1], "LSTOPH");
	//			continue;
	//		}
	//		//重启雷达
	//		else if (strncmp(line, "restart", 7) == 0)
	//		{
	//			ControlDrv(cfg.thread_ID[1], "LRESTH");
	//			continue;
	//		}
	//		//固定测距
	//		else if (strncmp(line, "fix", 3) == 0)
	//		{
	//			ControlDrv(cfg.thread_ID[1], "LMEASH");
	//			continue;
	//		}
	//		//退出
	//		else if (strncmp(line, "exit", 4) == 0)
	//		{
	//			break;
	//		}
	//	}
	//	//断开与雷达的连接
	//	StopDrv(&cfg);
	//	break;
	//}

	return 0;
}
