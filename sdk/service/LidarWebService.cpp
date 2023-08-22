#include "LidarCheckService.h"
#include "LidarWebService.h"
#include "standard_interface.h"
static const char *s_debug_level = "2";
static const char *s_root_dir = "web";
static const char *s_enable_hexdump = "no";
static const char *s_ssi_pattern = "*";
LidarWebService::LidarWebService(int port)
{
	m_port = port;
}

LidarWebService::~LidarWebService()
{
}

void LidarWebService::OpenLocalService(int lidarID)
{	
	char address[64] = {0};
	sprintf(address, "http://0.0.0.0:%d", m_port);
	struct mg_mgr mgr; // Event manager
	mg_log_set("2");   // Set to 3 to enable debug
	mg_mgr_init(&mgr); // Initialise event manager

	mg_http_listen(&mgr, address, fn, &lidarID); // Create HTTP listener
	for (;;)
	{
		mg_mgr_poll(&mgr, 100); // Infinite event loop	
	}
	mg_mgr_free(&mgr);
	
}

void LidarWebService::CloseLocalService()
{
}
static char *jsonValue(const char *result, const char *message, cJSON *array)
{
	if (array == NULL)
		array = cJSON_CreateArray();
	cJSON *root = cJSON_CreateObject();
	cJSON *item = cJSON_CreateString(result);
	cJSON_AddItemToObject(root, "result", item);
	item = cJSON_CreateString(message);
	cJSON_AddItemToObject(root, "message", item);

	cJSON_AddItemToObject(root, "data", array);
	char *out = cJSON_Print(root);
	cJSON_Delete(root);
	return out;
}

static void EEpromV101ToStr(EEpromV101 *eepromv101, char *version, char *result)
{
	cJSON *root = cJSON_CreateObject();
	// 类型，编号，序列号
	char tmp_sn[20] = {0};
	memcpy(tmp_sn, eepromv101->dev_sn, sizeof(eepromv101->dev_sn) - 1);
	char tmp_type[16] = {0};
	memcpy(tmp_type, eepromv101->dev_type, sizeof(eepromv101->dev_type) - 1);
	//printf("qqq %s\n",tmp_type);
	cJSON *item = cJSON_CreateNumber(eepromv101->dev_id);
	cJSON_AddItemToObject(root, "DID", item);
	item = cJSON_CreateString(tmp_sn);
	cJSON_AddItemToObject(root, "UID", item);
	item = cJSON_CreateString(tmp_type);
	cJSON_AddItemToObject(root, "NSP", item);
	/*printf("dev info: 设备编号:%d\t 序列号:%s\t 类型:%s\n", eepromv101->dev_id, tmp_sn, tmp_type);*/
	// ip地址 子网掩码 网关地址 默认目标IP  默认目标udp端口号  默认UDP对外服务端口号
	char tmp_IPv4[16] = {0};
	char tmp_mask[16] = {0};
	char tmp_gateway[16] = {0};
	char tmp_srv_ip[16] = {0};

	sprintf(tmp_IPv4, "%d.%d.%d.%d", eepromv101->IPv4[0], eepromv101->IPv4[1], eepromv101->IPv4[2], eepromv101->IPv4[3]);
	sprintf(tmp_mask, "%d.%d.%d.%d", eepromv101->mask[0], eepromv101->mask[1], eepromv101->mask[2], eepromv101->mask[3]);
	sprintf(tmp_gateway, "%d.%d.%d.%d", eepromv101->gateway[0], eepromv101->gateway[1], eepromv101->gateway[2], eepromv101->gateway[3]);
	sprintf(tmp_srv_ip, "%d.%d.%d.%d", eepromv101->srv_ip[0], eepromv101->srv_ip[1], eepromv101->srv_ip[2], eepromv101->srv_ip[3]);

	// printf("dev info: ip地址:%s 子网掩码:%s 网关地址:%s 默认目标IP:%s  默认目标udp端口号:%d   默认UDP对外服务端口号:%d\n",
	//	tmp_IPv4, tmp_mask, tmp_gateway, tmp_srv_ip, eepromv101->srv_port, eepromv101->local_port);
	item = cJSON_CreateString(tmp_IPv4);
	cJSON_AddItemToObject(root, "IPv4", item);
	item = cJSON_CreateString(tmp_mask);
	cJSON_AddItemToObject(root, "mask", item);
	item = cJSON_CreateString(tmp_gateway);
	cJSON_AddItemToObject(root, "gateway", item);
	item = cJSON_CreateString(tmp_srv_ip);
	cJSON_AddItemToObject(root, "srv_ip", item);
	item = cJSON_CreateNumber(eepromv101->srv_port);
	cJSON_AddItemToObject(root, "srv_port", item);
	item = cJSON_CreateNumber(eepromv101->local_port);
	cJSON_AddItemToObject(root, "local_port", item);

	/*char tmp_ranger_bias[8] = { 0 };
	memcpy(tmp_ranger_bias, eepromv101->ranger_bias, sizeof(eepromv101->ranger_bias) - 1);*/

	int tmp_ranger_bias[6] = {0};
	for (int i = 0; i < 4; i++)
	{
		tmp_ranger_bias[i] = eepromv101->ranger_bias[i * 2 + 1];
		if (eepromv101->ranger_bias[i * 2])
			tmp_ranger_bias[i] *= -1;
	}
	// 转速 ,电机启动参数,FIR滤波阶数，圈数，分辨率，开机自动上传，固定上传，数据点平滑，去拖点，记录校正系数，网络心跳，记录IO口极性
	//  sprintf(result,"dev info: 转速:%d 电机启动参数:%d FIR滤波阶数:%d 圈数:%d  分辨率:%d   开机自动上传:%d 固定上传:%d  数据点平滑:%d 去拖点:%d  记录校正系数:%s  网络心跳:%d  记录IO口极性:%d\n",
	//	eepromv101->RPM, eepromv101->RPM_pulse, eepromv101->fir_filter, eepromv101->cir, eepromv101->with_resample, eepromv101->auto_start,
	//	eepromv101->target_fixed, eepromv101->with_smooth, eepromv101->with_filter, tmp_ranger_bias, eepromv101->net_watchdog, eepromv101->pnp_flags);
	item = cJSON_CreateNumber(eepromv101->RPM);
	cJSON_AddItemToObject(root, "RPM", item);

	item = cJSON_CreateNumber(eepromv101->RPM_pulse);
	cJSON_AddItemToObject(root, "PUL", item);

	item = cJSON_CreateNumber(eepromv101->fir_filter);
	cJSON_AddItemToObject(root, "FIR", item);

	item = cJSON_CreateNumber(eepromv101->cir);
	cJSON_AddItemToObject(root, "cir", item);

	item = cJSON_CreateNumber(eepromv101->with_resample);
	cJSON_AddItemToObject(root, "with_resample", item);

	item = cJSON_CreateNumber(eepromv101->auto_start);
	cJSON_AddItemToObject(root, "ATS", item);

	item = cJSON_CreateNumber(eepromv101->target_fixed);
	cJSON_AddItemToObject(root, "TFX", item);

	item = cJSON_CreateNumber(eepromv101->with_smooth);
	cJSON_AddItemToObject(root, "SMT", item);

	item = cJSON_CreateNumber(eepromv101->with_filter);
	cJSON_AddItemToObject(root, "DSW", item);

	item = cJSON_CreateIntArray(tmp_ranger_bias, 6);
	cJSON_AddItemToObject(root, "ERR", item);

	item = cJSON_CreateNumber(eepromv101->net_watchdog);
	cJSON_AddItemToObject(root, "net_watchdog", item);

	item = cJSON_CreateNumber(eepromv101->pnp_flags);
	cJSON_AddItemToObject(root, "PNP", item);

	item = cJSON_CreateNumber(eepromv101->deshadow);
	cJSON_AddItemToObject(root, "AF", item);

	item = cJSON_CreateNumber(eepromv101->should_post);
	cJSON_AddItemToObject(root, "PST", item);

	item = cJSON_CreateString(version);
	cJSON_AddItemToObject(root, "version", item);
	char *out = jsonValue("SUCCESS", "", root);
	sprintf(result, "%s", out);
	free(out);
}
void DevDataToStr(DevData *devdata, int index, char *value)
{
	memcpy(devdata->set + index, "1", 1);
	switch (index)
	{
	case 0:
		devdata->RPM = atoi(value);
		break;
	case 1:
		devdata->ERR = atoi(value);
		break;
	case 2:
		memcpy(&devdata->UDP, value, sizeof(devdata->UDP));
		break;
	case 3:
		memcpy(&devdata->DST, value, sizeof(devdata->DST));
		break;
	case 4:
		memcpy(&devdata->NSP, value, sizeof(devdata->NSP));
		break;
	case 5:
		memcpy(&devdata->UID, value, sizeof(devdata->UID));
		break;
	case 6:
		devdata->FIR = atoi(value);
		break;
	case 7:
		devdata->PUL = atoi(value);
		break;
	case 8:
		devdata->VER = atoi(value);
		break;
	case 9:
		devdata->PNP = atoi(value);
		break;
	case 10:
		devdata->SMT = atoi(value);
		break;
	case 11:
		devdata->DSW = atoi(value);
		break;
	case 12:
		devdata->DID = atoi(value);
		break;
	case 13:
		devdata->ATS = atoi(value);
		break;
	case 14:
		devdata->TFX = atoi(value);
		break;
	case 15:
		devdata->PST = atoi(value);
	case 16:
		devdata->AF = atoi(value);
		break;
	}
}
static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
	int id = *(int *)fn_data;
	RunConfig *runcfg = BlueSeaLidarSDK::getInstance()->getLidar(id);
	if (ev == MG_EV_HTTP_MSG)
	{
		struct mg_http_message *hm = (struct mg_http_message *)ev_data;
		if (mg_http_match_uri(hm, "/api/stats"))
		{
			cJSON *points = cJSON_CreateArray();
			cJSON *user;
			cJSON *item = cJSON_CreateNumber(1);
			for (struct mg_connection *t = c->mgr->conns; t != NULL; t = t->next)
			{
				char loc[40], rem[40];
				user = cJSON_CreateObject();
				item = cJSON_CreateNumber(t->id);
				cJSON_AddItemToObject(user, "ID", item);

				item = cJSON_CreateString(t->is_udp ? "UDP" : "TCP");
				cJSON_AddItemToObject(user, "type", item);

				item = cJSON_CreateString(t->is_listening ? "LISTENING" : t->is_accepted ? "ACCEPTED "
																						 : "CONNECTED");
				cJSON_AddItemToObject(user, "state", item);
				mg_straddr(&t->loc, loc, sizeof(loc));
				item = cJSON_CreateString(loc);
				cJSON_AddItemToObject(user, "loc", item);
				mg_straddr(&t->rem, rem, sizeof(rem));
				item = cJSON_CreateString(rem);
				cJSON_AddItemToObject(user, "rem", item);
				cJSON_AddItemToArray(points, user);
			}
			char *out = jsonValue("SUCCESS", "", points);
			mg_http_reply(c, 200, "", "%s", out);
			free(out);
			return;
		}

		else if (mg_http_match_uri(hm, "/init"))
		{
			cJSON *arr = cJSON_CreateArray();
			cJSON *point = cJSON_CreateObject();
			cJSON *item = cJSON_CreateString(runcfg->runscript.type);
			cJSON_AddItemToObject(point, "type", item);
			std::string SYSType;
#ifdef _WIN32
			SYSType = "WIN32";
#elif __linux__
			SYSType = "LINUX";
#elif __APPLE__
			SYSType = "APPLE";
#endif
			item = cJSON_CreateString(SYSType.c_str());
			cJSON_AddItemToObject(point, "SDKENV", item);
			cJSON_AddItemToObject(arr, "", point);
			char *out = jsonValue("SUCCESS", "", arr);
			mg_http_reply(c, 200, "", "%s", out);
			free(out);
			return;
		}
		// 雷达的动作控制
		else if (mg_http_match_uri(hm, "/action"))
		{
			cJSON *arr = cJSON_CreateArray();
			char query[256] = {0};
			memcpy(query, hm->query.ptr, hm->query.len);
			char *ret = strstr(query, "cmd=");
			if (!ret || strlen(ret + 4) != 6)
			{
				char *out = jsonValue("ERROR", "url is not current!", NULL);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}

			char cmd[7] = {0};
			sprintf(cmd, "%s", ret + 4);
			runcfg->mode = S_PACK;
			runcfg->send_len = 6;
			strcpy(runcfg->send_cmd, cmd);
			runcfg->action = CONTROL;

			char *out = jsonValue("SUCCESS", "", NULL);
			mg_http_reply(c, 200, "", "%s", out);
			free(out);
			return;
		}
		// 雷达点云数据/报警信息
		else if (mg_http_match_uri(hm, "/data"))
		{
			int pointsNum = runcfg->userdata.data.framedata.N;
			if (pointsNum <= 0 || pointsNum > MAX_FRAMEPOINTS)
			{
				char message[64] = {0};
				sprintf(message, "get point number unusual %d", pointsNum);
				char *out = jsonValue("ERROR", message, NULL);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			cJSON *root = cJSON_CreateObject();
			cJSON *points = cJSON_CreateArray();
			cJSON *point;
			cJSON *item = cJSON_CreateNumber(pointsNum);
			cJSON_AddItemToObject(root, "N", item);
			item = cJSON_CreateIntArray((int *)runcfg->userdata.data.framedata.ts, 2);
			cJSON_AddItemToObject(root, "ts", item);
			FrameData *framedata = &runcfg->userdata.data.framedata;
			for (int i = 0; i < pointsNum; i++)
			{
				point = cJSON_CreateObject();
				item = cJSON_CreateNumber(framedata->data[i].angle * 180 / 3.1415926);
				cJSON_AddItemToObject(point, "angle", item);
				item = cJSON_CreateNumber(framedata->data[i].distance);
				cJSON_AddItemToObject(point, "distance", item);
				item = cJSON_CreateNumber(framedata->data[i].confidence);
				cJSON_AddItemToObject(point, "confidence", item);
				cJSON_AddItemToArray(points, point);
			}

			cJSON_AddItemToObject(root, "data", points);
			item = cJSON_CreateString("SUCCESS");
			cJSON_AddItemToObject(root, "result", item);
			item = cJSON_CreateString("");
			cJSON_AddItemToObject(root, "message", item);

			// 添加防区相关的数据
			LidarMsgHdr zone;
			memcpy(&zone, &runcfg->zonemsg, sizeof(LidarMsgHdr));

			item = cJSON_CreateNumber(zone.flags);
			cJSON_AddItemToObject(root, "zone_flag", item);
			item = cJSON_CreateNumber(zone.events);
			cJSON_AddItemToObject(root, "zone_events", item);
			item = cJSON_CreateNumber(zone.zone_actived);
			cJSON_AddItemToObject(root, "zone_actived", item);

			char *out = cJSON_Print(root);
			mg_http_reply(c, 200, "", "%s", out);
			cJSON_Delete(root);
			free(out);
			msleep(10);
		}
		// 获取雷达设备信息
		else if (mg_http_match_uri(hm, "/getDevinfo"))
		{
			EEpromV101 *eeprom = new  EEpromV101;
			char info[1024]={0};
			BlueSeaLidarSDK::getInstance()->GetDevInfo(id,eeprom);
			EEpromV101ToStr(eeprom,runcfg->hardwareVersion, info); 
			mg_http_reply(c, 200, "", "%s", info);
			return;
		}
		// 设置雷达设备参数
		//else if (mg_http_match_uri(hm, "/setDevinfo"))
		//{
		//	// memset(&g_cfg[*cfg_index]->pointdata, 0, sizeof(PointData));
		//	// memset(&g_cfg[*cfg_index]->zone, 0, sizeof(LidarMsgHdr));

		//	if (strcmp(runcfg->runscript.type, "uart") == 0)
		//	{
		//		char *out = jsonValue("ERROR", "uart is not support this function!", NULL);
		//		mg_http_reply(c, 200, "", "%s", out);
		//		free(out);
		//		return;
		//	}
		//	char query[256] = {0};
		//	memcpy(query, hm->query.ptr, hm->query.len);
		//	std::string str = query;
		//	StringReplace(str, "%20", " ");
		//	int ret1 = str.find("index=");
		//	int ret2 = str.find("&value=");
		//	// 对传入字符串格式是否合法
		//	if (ret1 < 0 || ret2 < 0)
		//	{
		//		char *out = jsonValue("ERROR", "url is not current!", NULL);
		//		mg_http_reply(c, 200, "", "%s", out);
		//		free(out);
		//		return;
		//	}
		//	int index = 0;
		//	index = atoi(str.substr(ret1 + 6, ret2 - ret1 - 6).c_str());
		//	// 传入命令的值超过限制
		//	if (index < 0 || index > 16)
		//	{
		//		char *out = jsonValue("ERROR", "url is not current!", NULL);
		//		mg_http_reply(c, 200, "", "%s", out);
		//		free(out);
		//		return;
		//	}

		//	std::string value = str.substr(ret2 + 7);
		//	DevData tmpData;
		//	memset(&tmpData, 0, sizeof(DevData));
		//	DevDataToStr(&tmpData, index, (char *)value.c_str());
		//	//SetDevInfo_extre(g_cfg[*cfg_index]->thread_ID[1], tmpData);
		//	char buf[3] = {0};
		//	memcpy(buf, tmpData.result + 2 * index, 2);
		//	char *out = jsonValue("SUCCESS", buf, NULL);
		//	mg_http_reply(c, 200, "", "%s", out);
		//	free(out);
		//	return;
		//}
		// 获取当前雷达的列表    0表示子线程运行
		else if (mg_http_match_uri(hm, "/getLidarList"))
		{
			char query[256] = {0};
			memcpy(query, hm->query.ptr, hm->query.len);
			std::string str = query;
			StringReplace(str, "%20", " ");
			int ret1 = str.find("mode=");
			if (ret1 < 0)
			{
				char *out = jsonValue("ERROR", "url is not current!", NULL);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			// 这里0为打开服务,并获取当前的雷达列表  1为关闭服务
			int index = atoi(str.substr(ret1 + 5, 1).c_str());
			if (index == 0)
			{
				std::vector<DevConnInfo> data = BlueSeaLidarSDK::getInstance()->getLidarsList();
				// printf("num:%d\n",data.size());
				cJSON *arr = cJSON_CreateArray();
				cJSON *point;
				cJSON *item;
				char conn_ip[16] = {0};
				short conn_port = 0;
				for (unsigned int i = 0; i < data.size(); i++)
				{
					point = cJSON_CreateObject();
					int type = data.at(i).type;
					item = cJSON_CreateNumber(data.at(i).type);
					cJSON_AddItemToObject(point, "type", item);

					item = cJSON_CreateString(data.at(i).com_port);
					cJSON_AddItemToObject(point, "com_port", item);

					item = cJSON_CreateNumber(data.at(i).com_speed);
					cJSON_AddItemToObject(point, "com_speed", item);

					if (type == 1)
					{
						DevInfo v1;
						memcpy(&v1, &data.at(i).info.v1, sizeof(DevInfo));
						sprintf(conn_ip, "%d.%d.%d.%d", v1.ip[0], v1.ip[1], v1.ip[2], v1.ip[3]);
						conn_port = v1.port;
					}
					else if (type == 2)
					{
						DevInfo2 v1;
						memcpy(&v1, &data.at(i).info.v1, sizeof(DevInfo2));
						sprintf(conn_ip, "%d.%d.%d.%d", v1.ip[0], v1.ip[1], v1.ip[2], v1.ip[3]);
						conn_port = v1.remote_udp;
					}
					else if (type == 3)
					{
						DevInfoV101 v1;
						memcpy(&v1, &data.at(i).info.v1, sizeof(DevInfoV101));
						sprintf(conn_ip, "%d.%d.%d.%d", v1.ip[0], v1.ip[1], v1.ip[2], v1.ip[3]);
						conn_port = v1.remote_udp;
					}
					item = cJSON_CreateString(conn_ip);
					cJSON_AddItemToObject(point, "conn_ip", item);
					item = cJSON_CreateNumber(conn_port);
					cJSON_AddItemToObject(point, "conn_port", item);

					char tmp[16] = {0};
					sprintf(tmp, "%s", data.at(i).timeStr);
					item = cJSON_CreateString(tmp);
					cJSON_AddItemToObject(point, "time", item);

					cJSON_AddItemToObject(arr, "", point);
				}
				char *out = jsonValue("SUCCESS", "", arr);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			else if (index == 1)
			{
				BlueSeaLidarSDK::getInstance()->CloseHeartService();
				cJSON *item = cJSON_CreateArray();
				char *out = jsonValue("SUCCESS", "", item);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			else
			{
				cJSON *item = cJSON_CreateArray();
				char *out = jsonValue("ERROR", "arg is not current!", item);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
		}
		// 获取当前雷达的防区
		//else if (mg_http_match_uri(hm, "/getZone"))
		//{
		//	cJSON *item = cJSON_CreateArray();
		//	char *out = jsonValue("ERROR", "function is not support!", item);
		//	mg_http_reply(c, 200, "", "%s", out);
		//	free(out);
		//}
		//// 设置当前雷达的防区   post
		//else if (mg_http_match_uri(hm, "/setZone"))
		//{
		//	cJSON *item = cJSON_CreateArray();
		//	char *out = jsonValue("ERROR", "function is not support!", item);
		//	mg_http_reply(c, 200, "", "%s", out);
		//	free(out);
		//}
		// 本地资源文件访问
		else
		{
			mg_http_serve_opts opts = {0};
			opts.root_dir = s_root_dir;
			struct mg_http_message tmp = {0};
			mg_http_serve_dir(c, hm, &opts);
		}
	}
	else if (ev == MG_EV_WS_MSG)
	{
		//g_c = c;
		mg_ws_send(c, "OK", 2, WEBSOCKET_OP_TEXT);
	}
	//(void)fn_data;

}

void StringReplace(std::string &strBase, std::string strSrc, std::string strDes)
{
	std::string::size_type pos = 0;
	std::string::size_type srcLen = strSrc.size();
	std::string::size_type desLen = strDes.size();
	pos = strBase.find(strSrc, pos);
	while ((pos != std::string::npos))
	{
		strBase.replace(pos, srcLen, strDes);
		pos = strBase.find(strSrc, (pos + desLen));
	}
}
