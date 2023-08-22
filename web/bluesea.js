/***********全局变量说明 start ***********************/

const pointsData = [[1, 1]];//雷达点云数据
var option;//echart.js设置参数
var dom = document.getElementById('container');//绘图容器
var myChart;//echart组件全局变量
var size = 1;//点位显示的大小
var isPlay = true;//控制雷达是否停止刷新点云数据
var isPlay2 = true;//控制雷达的列表刷新
var currentTime = 1;//测试时间
var currentTime1 = 1;//测试时间
var type = "uart";//雷达的类型
var SDKENV = "other";//SDK运行服务所在的操作系统 分为Windows|Linux
var scale = 0.32; //设置矩形的px和实际坐标系的比例   320/1000
var isDrag = false;//设置的防区拖动状态
var isClick = false;//防区是否被点击
//防区相关全局变量
var echartL = 0;
var echartT = 0;
var echartR = 0;
var echartB = 0;
var zoneSerial = 0;//当前的防区序号
var outputID = 0;//防区种类
var zoneID = 0;//增加的绘制防区的ID,也是总数
//
var zoneColor1 = "#333";//触发观察区颜色设置 #800000
var zoneColor2 = "#333";//触发警戒区颜色设置
var zoneColor3 = "#333";//触发报警区颜色设置

var zone_flag = 0;//防区返回信息的类型
var zone_events = 0;//防区返回的具体信息
var zone_actived = 0;//激活的防区
var zone_text = "提示信息:";

var colorList = ['#E5040F', '0FFA0F', '#F6931C', '#FFD52E']
var offsetX = 0;
var offsetY = 0;
var zoneList = [];

/***********全局变量说明  end ***********************/
//echart.js初始化
myChart = echarts.init(dom, null, {
  renderer: 'canvas',
  useDirtyRect: false
});

//定时获取服务端的点云数据
function getPointsData() {
  isPlay = true;
  $.get("/data", function (data, status) {
    if (isPlay)
      getPointsData();
    if (status != "success")
      return;
    var jsondatas = eval("(" + data + ")");
    $.each(jsondatas.data, function (i, n) {
      pointsData[i] = [n.distance * 1000, n.angle];
      //console.log("dis:" + n.distance + "  angle:" + n.angle);
    })
    option.series[0].symbolSize = size;
    option.series[0].data = pointsData;
    $('#index').attr("value", jsondatas.N);
    if (type != "uart") {
        if(zone_flag!=jsondatas.zone_flag ||zone_events!=jsondatas.zone_events ||zone_actived!=jsondatas.zone_actived)
        {
            zone_flag = jsondatas.zone_flag;
            zone_events = jsondatas.zone_events;
            zone_actived = jsondatas.zone_actived;
            ShowZoneMSG(zone_flag, zone_events, zone_actived);
        }
    }
    $('#timestamp_s').attr("value", jsondatas.ts[0]);
    $('#timestamp_us').attr("value", jsondatas.ts[1]);

    myChart.setOption(option, false);


  });
};
//停止获取雷达数据
function stopGetData() {
  isPlay = false;
}
//控制雷达的运动状态
function action(id) {
  $("#" + id).attr("class", "layui-btn layui-btn-primary");
  var cmd = "/action?cmd=" + id;
  $.get(cmd, function (data, status) {
    if (status = "success") {
      $("#" + id).attr("class", "layui-btn layui-btn-radius");
      console.log("id:" + id + "cmd:" + cmd);
      console.log(data);
    }
  })
}

//获取设备信息
function getDevInfo() {
  //不停止定时器会导致请求堵塞
  isPlay = false;
  $.get("/getDevinfo", function (data, status) {
    var jsondatas = eval("(" + data + ")").data;
    if (type == "udp" || type == "vpc") {
      for (var val in jsondatas) {
        //alert(val + " " + jsondatas[val]);//输出如:name 
        $("#" + val).attr("value", jsondatas[val]);
        if (val == "ERR") {
          var tmp = jsondatas[val];
          $("#set" + val).attr("value", tmp[0]);
        } else if (val == "PNP" || val == "SMT" || val == "DSW" || val == "ATS" || val == "TFX" || val == "PST") {
          var tmp = jsondatas[val];
          $('#set' + val).find("option[value=" + tmp + "]").attr("selected", true);

        }
        else {
          $("#set" + val).attr("value", jsondatas[val]);
        }
      }
      layui.form.render('select') //再次渲染
    }
    //uart版本仅有序列号
    else if (type == "uart") {
      $("#UID").attr("value", jsondatas.UID);
    }
    getPointsData();
  });
}
function ipStr(str) {
  var arr1 = str.split(".");
  let result = arr1[0].toString().padStart(3, '0') + "." + arr1[1].toString().padStart(3, '0') + "." + arr1[2].toString().padStart(3, '0') + "." + arr1[3].toString().padStart(3, '0');
  return result;

}

var g_num = 0;
//设置指定设备信息
function setDevInfo() {
  //需要先获取当前设备信息，然后对比
  var value;
  var value1;
  //轮询所有选项，是否有数据被修改
  for (var index = 0; index <= 16; index++) {
    if (index == 0) {
      value = $("#setRPM").val();
      value1 = $("#RPM").val();
      if (value == value1)
        continue;
    } else if (index == 1) {
      value = $("#setERR").val();
      var tmp = $("#ERR").val();
      var arr1 = tmp.split(",");
      var value1 = arr1[0];
      if (value == value1)
        continue;
    } else if (index == 2) {
      var valuetmp1 = $("#setIPv4").val();
      var valuetmp2 = $("#setmask").val();
      var valuetmp3 = $("#setgateway").val();
      var valuetmp4 = $("#setlocal_port").val();

      var value1tmp1 = $("#IPv4").val();
      var value1tmp2 = $("#mask").val();
      var value1tmp3 = $("#gateway").val();
      var value1tmp4 = $("#local_port").val();
      if (valuetmp1 == value1tmp1 && valuetmp2 == value1tmp2 && valuetmp3 == value1tmp3 && valuetmp4 == value1tmp4)
        continue;
      value = ipStr(valuetmp1) + " " + ipStr(valuetmp2) + " " + ipStr(valuetmp3) + " " + valuetmp4.toString().padStart(5, '0');

    } else if (index == 3) {
      var valuetmp1 = $("#setsrv_ip").val();
      var valuetmp2 = $("#setsrv_port").val();
      var value1tmp1 = $("#srv_ip").val();
      var value1tmp2 = $("#srv_port").val();
      if (valuetmp1 == value1tmp1 && valuetmp2 == value1tmp2)
        continue;
      value = ipStr(valuetmp1) + " " + valuetmp2.toString().padStart(5, '0');
    } else if (index == 4) {
      value = $("#setNSP").val();
      value1 = $("#NSP").val();
      if (value == value1)
        continue;
    } else if (index == 5) {
      value = $("#setUID").val();
      value1 = $("#UID").val();
      if (value == value1)
        continue;
    } else if (index == 6) {
      value = $("#setFIR").val();
      value1 = $("#FIR").val();
      if (value == value1)
        continue;
    } else if (index == 7) {
      continue;
      // value = $("#setPUL").val();
      // value1 = $("#PUL").val();
      // if(value==value1)
      //   continue;
    } else if (index == 8) {
      continue;
      //value = $("#setVER").val();
    } else if (index == 9) {
      value = $("#setIO").val();
      value1 = $("#IO").val();
      if (value == value1)
        continue;
    } else if (index == 10) {
      value = $("#setSMT").val();
      value1 = $("#SMT").val();
      if (value == value1)
        continue;
    } else if (index == 11) {
      value = $("#setDSW").val();
      value1 = $("#DSW").val();
      if (value == value1)
        continue;
    }
    else if (index == 12) {
      value = $("#setDID").val();
      value1 = $("#DID").val();
      if (value == value1)
        continue;
    } else if (index == 13) {
      value = $("#setATS").val();
      value1 = $("#ATS").val();
      if (value == value1)
        continue;
    } else if (index == 14) {
      value = $("#setTFX").val();
      value1 = $("#TFX").val();
      if (value == value1)
        continue;
    } else if (index == 15) {
      value = $("#setPST").val();
      value1 = $("#PST").val();
      if (value == value1)
        continue;
    } else if (index == 16) {
      value = $("#setAF").val();
      value1 = $("#AF").val();
      if (value == value1)
        continue;
    } else {
      return;
    }
    var cmd = "/setDevinfo?index=" + index + "&value=" + value;
    $.get(cmd, function (data, status) {
      alert("cmd:" + cmd + "  " + data);
      getDevInfo();
    })
  }
}
function showClientTable() {
  layui.use('table', function () {
    var table = layui.table;

    $.get("/api/stats", function (data, status) {
      var jsondatas = eval("(" + data + ")");
      if (jsondatas.result != "SUCCESS")
        return;
      layer.open({
        type: 1,
        area: ["700px", '500px'],
        title: "当前连接信息",
        maxmin: false,
        content: '<div><table id="templateTable2" lay-filter="templateTable2"></table></div>', //先定义一个数据表格的div框
        success: function (index, layero) {
          table.render({
            elem: '#templateTable2'
            , height: 420
            , width: '100%'
            , data: jsondatas.data
            , cols: [[ //设置数据表格表头
              { field: 'ID', title: '内部编号', width: 100, sort: true, fixed: 'left' }
              , { field: 'type', title: '类型', fixed: 'left' }
              , { field: 'state', title: '状态', width: 150, sort: true, fixed: 'left' }
              , { field: 'loc', title: '本地地址', width: 200, fixed: 'left' }
              , { field: 'rem', title: '目标地址', width: 200, fixed: 'left' }
            ]]
            , page: true //使用分页
            , limits: [10, 20, 30, 40, 50, 60, 70, 80, 90, 100] //动态设置每页显示数据的条数
            , limit: 10 //默认每页多少条
            , response: { // 响应的数据的格式
              statusName: 'result'
              , statusCode: "SUCCESS"
            }
          });
        }

      });
    });
  });
}

function showLidarTable() {
  layui.use('table', function () {
    var table = layui.table;
    $.get("getLidarList?mode=0", function (data, status) {
      var jsondatas = eval("(" + data + ")");
      if (jsondatas.result != "SUCCESS")
        return;
      layer.open({
        type: 1,
        area: ["700px", '500px'],
        title: "当前可用雷达列表",
        maxmin: false,
        content: '<div><table id="templateTable" lay-filter="templateTable"}></table></div>', //先定义一个数据表格的div框
        success: function (index, layero) {
          table.render({
            elem: '#templateTable'
            , height: 420
            , width: '100%'
            , data: jsondatas.data
            , cols: [[ //设置数据表格表头
              { field: 'type', title: '设备类型', width: 100, sort: true, fixed: 'left' }
              , { field: 'com_port', title: '串口号', width: 100, fixed: 'left' }
              //, { field: 'com_speed', title: '波特率', width: 100, fixed: 'left' }
              , { field: 'conn_ip', title: 'IP地址', width: 150, fixed: 'left' }
              , { field: 'conn_port', title: 'IP端口号', width: 100, fixed: 'left' }
              , { field: 'time', title: '更新时间', width: 100, fixed: 'left' }
            ]]
            , page: true //使用分页
            , limits: [10, 20, 30, 40, 50, 60, 70, 80, 90, 100] //动态设置每页显示数据的条数
            , limit: 10 //默认每页多少条
            , response: { // 响应的数据的格式
              statusName: 'result'
              , statusCode: "SUCCESS"
            }
          });
          var time = setInterval(function () {
            table.reload('templateTable', {
              url: '/getLidarList?mode=0'
              , where: {} //设定异步数据接口的额外参数
              , page: {
                curr: 1 //重新从第 1 页开始
              }
            });
          }, 3000);
        },
        cancel: function () {
          $.get("getLidarList?mode=1", function (data, status) {
            var jsondatas = eval("(" + data + ")");
            if (jsondatas.result != "SUCCESS")
              return;
          })
        }
      });
    });
  });
}
//初始化获取雷达的类型
$(document).ready(function () {
  $.get("/init", function (data, status) {
    var jsondatas = eval("(" + data + ")");
    type = jsondatas.data[0].type;
    SDKENV = jsondatas.data[0].SDKENV;
  });
  getDevInfo();
});

/***********极坐标绘制参数配置 start ********************************/
option = {
  title: {
    text: '雷达图像'
  },
  legend: {
    data: ['雷达点云数据', '防区-观察区', '防区-警戒区', '防区-报警区'],
    left: 'right',
  },
  polar: {
    center: ["60%", "60%"]
  },
  angleAxis: {
    type: 'value',
    boundaryGap: false,
    startAngle: 0,
    min: 0,
    max: 360,
    interval: 30,
    splitLine: {
      show: true
    },
    axisLine: {
      show: false
    }
  },
  radiusAxis: {
    type: 'value',
    min: 0,//'dataMin',   根据获得的数据显示会导致波动过大，不方便观察
    max: 5000,//'dataMax',
    axisLine: {
      show: false
    },
    axisLabel: {
      rotate: 45
    }
  },
  myarea: {
    silent: true
  },
  graphic: [
    {
      id: 'text4',
      type: 'text',
      $action: 'merge',
      //z: 100,
      left: 100,
      top: 100,
      style: {
        fill: '#333',
        text: [
          zone_text
        ].join('\n'),
        font: '20px Microsoft YaHei',
      }
    }
  ],
  series: [
    {
      id: 0,
      name: '雷达点云数据',
      type: 'scatter',
      coordinateSystem: 'polar',
      showAllSymbol: false,
      large: true,
      largeThreshold: 10,
      symbolSize: 2,
      data: pointsData,
      color: 'green'
    }
    ,
    {
      id: 10,
      name: '防区-观察区',
      type: 'scatter',
      coordinateSystem: '',
      symbolSize: 2,
      color: 'blue'
    }
    ,
    {
      id: 20,
      name: '防区-警戒区',
      type: 'scatter',
      coordinateSystem: '',
      symbolSize: 2,
      color: 'yellow'
    },
    {
      id: 30,
      name: '防区-报警区',
      type: 'scatter',
      coordinateSystem: '',
      symbolSize: 2,
      color: 'red'
    }
  ]
};
/***********极坐标绘制参数配置 end ********************************/
if (option && typeof option === 'object') {
  myChart.setOption(option, true);
}
/***********当前绘图容器区域监听鼠标滑轮事件，并且设置坐标的最大值********************************/
if (dom.addEventListener) {
  // IE9, Chrome, Safari, Opera
  dom.addEventListener("mousewheel", MouseWheelHandler, false);
  // Firefox
  dom.addEventListener("DOMMouseScroll", MouseWheelHandler, false);
}

function MouseWheelHandler(e) {
  var e = window.event || e; // old IE support
  var delta = Math.max(-1, Math.min(1, (e.wheelDelta || -e.detail)));
  var tmp = option.radiusAxis.max + delta * 100;
  if (tmp < 500)
    return;
  option.radiusAxis.max = tmp;

  var old_scale = scale;
  scale = 320 / tmp * 1.00;
  //将当前的防区图形，根据鼠标的缩放而缩放(仅限于矩形和扇形)
  var jsonarray = eval(option.graphic);
  for (var i = 0; i < zoneID; i++) {
    if (jsonarray[i + 1].type == 'rect') {
      jsonarray[i + 1].shape.x = 480 + (jsonarray[i + 1].shape.x - 480) / old_scale * scale
      jsonarray[i + 1].shape.y = 480 + (jsonarray[i + 1].shape.y - 480) / old_scale * scale
      jsonarray[i + 1].shape.width = (jsonarray[i + 1].shape.width) / old_scale * scale
      jsonarray[i + 1].shape.height = (jsonarray[i + 1].shape.height) / old_scale * scale
    } else if (jsonarray[i + 1].type == 'sector') {
      jsonarray[i + 1].shape.r = (jsonarray[i + 1].shape.r) / old_scale * scale
      jsonarray[i + 1].shape.r0 = (jsonarray[i + 1].shape.r0) / old_scale * scale
    }
  }
  option.graphic = jsonarray;
  myChart.setOption(option, false);

  console.log(tmp);
  //保证雷达图的滚轮效果和页面整体的滚轮效果独立
  window.event.preventDefault();

}
/***********当前绘图容器区域监听鼠标滑轮事件，并且设置坐标的最大值********************************/

layui.use(['element', 'layer', 'form', 'table'], function () {
  var element = layui.element, //Tab的切换功能，切换事件监听等，需要依赖element模块
    layer = layui.layer,
    form = layui.form;
  //对所有表单的监听
  form.on('select(size)', function (data) {
    size = $("select[id=selectSize").val();
    console.log(size);
    form.render('select');//select是固定写法 不是选择器
  });
  form.on('select(zone)', function (data) {
    var tmp = $("select[id=setZoneGraphics").val();
    if (tmp == 'rect') {
      $("#valueL").html("左");
      $("#valueT").html("上");
      $("#valueR").html("右");
      $("#valueB").html("下");
    } else if (tmp == 'sector') {
      $("#valueL").html("起始角度");
      $("#valueT").html("终止角度");
      $("#valueR").html("远端距离");
      $("#valueB").html("近端距离");
    }

    form.render('select');//select是固定写法 不是选择器
  });
  form.verify({
    require: function (value) {
      value = $.trim(value);
      if (!value) {
        return "必填项不能为空";
      }
    },
    ip: [
      /^(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])\.(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])\.(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])\.(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])$/
      , 'IP地址不符合规则'
    ]
  });
});
/***************************************防区相关函数***********************************/
function readZone() {

    ClearZone();
  $.get("/getZone", function (data, status) {
    var jsondatas = eval("(" + data + ")");
    var N = jsondatas.data.N;
    if (N <= 0) return;

    for (var i = 0; i < N; i++) {
      var type = jsondatas.data.values[i].type;
      if (type != 2 && type != 4) return;
      if (type == 2) { ZoneGraphics = 'rect'; }
      if (type == 4) { ZoneGraphics = 'sector'; }

      zoneSerial = jsondatas.data.values[i].zoneID;
      //如果设置的防区和返回的data数据中的防区信息不一致，则不显示
      if( $("#setZone").val()!=zoneSerial)
          return;
      outputID = jsondatas.data.values[i].outputID;
      echartL = jsondatas.data.values[i].value[0];
      echartT = jsondatas.data.values[i].value[1];
      echartR = jsondatas.data.values[i].value[2];
      echartB = jsondatas.data.values[i].value[3];
      drawZone(ZoneGraphics, zoneSerial, echartL, echartT, echartR, echartB, outputID);
    }
  });
}

function writeZone() {
  debugger
  $.post("/setZone", JSON.stringify(zoneList), function (data, status) {
    debugger
  }, "json");
}
function ShowZone() {
  var ZoneGraphics = $("#setZoneGraphics").val();
  //防区级别不同，颜色不同
  outputID = $("#setZoneAlarm").val();
  zoneSerial = $("#setZone").val();
  echartL = $("#zoneL").val();
  echartT = $("#zoneT").val();
  echartR = $("#zoneR").val();
  echartB = $("#zoneB").val();

  drawZone(ZoneGraphics, zoneSerial, echartL, echartT, echartR, echartB, outputID);
}

function drawZone(ZoneGraphics, zoneSerial, echartL, echartT, echartR, echartB, outputID) {
  $("#setZone").attr("value", zoneSerial);
  if (outputID == '0')
    ZoneAlarm = 'blue';
  else if (outputID == '1')
    ZoneAlarm = 'yellow';
  else if (outputID == '2')
    ZoneAlarm = 'red';

  var type = 0;
  var jsonarray = eval(option.graphic);
  var arr;
  if (ZoneGraphics == 'rect') {
    type = 2;
    arr =
    {
      type: ZoneGraphics,
      id: zoneID,
      $action: 'merge',
      invisible: false,
      draggable: false,
      silent: false,
      shape:
      {
         x: 480 + echartL * 1 * scale,
         y: 480 + echartT * 1 * scale,
         width: (echartR - echartL) * scale,
         height: (echartT - echartB) * -1 * scale
        //这里以90度为0度，根据
        //x: 480 + echartT * 1 * scale,
        //y: 480 - echartL * 1 * scale,
        //width: (echartT - echartB) * -1 * scale,
        //height: -(echartR - echartL) * scale,
      },
      style: {
        fill: '#fff0',
        stroke: ZoneAlarm
      }
    }
  } else if (ZoneGraphics == 'sector') {
    type = 4;
    arr =
    {
      type: ZoneGraphics,
      id: zoneID,
      // top: 480,
      // button:480,
      // right: 480,
      // left:480,
      $action: 'merge',
      invisible: false,
      draggable: false,
      silent: false,
      shape:
      {
        cx: 480,
        cy: 480,
        r: echartR * 1 * scale,
        r0: echartB * 1 * scale,
        //startAngle: -Math.PI / 2 + echartL / 1800 * Math.PI,
        //endAngle: -Math.PI / 2 + echartT / 1800 * Math.PI，
        startAngle:  echartL / 1800 * Math.PI,
        endAngle:  echartT / 1800 * Math.PI,
        clockwise: true
      },
      style: {
        fill: '#fff0',
        stroke: ZoneAlarm
      }
    }
  } else {
    return -1;
  }
  zoneID++;
  jsonarray.push(arr);
  option.graphic = jsonarray;
  myChart.setOption(option, false);

  //整理需要保存的防区范围，并存入数组
  var zone = {};
  zone['type'] = type;
  zone['zoneID'] = $("#setZone").val() * 1;
  zone['outputID'] = outputID * 1;
  zone['arg1'] = echartL * 1;
  zone['arg2'] = echartT * 1;
  zone['arg3'] = echartR * 1;
  zone['arg4'] = echartB * 1;
  zoneList.push(zone);
  return 0;
}


function ClearZone() {
  for (var i = 0; i < zoneID; i++) {
    option.graphic[1 + i].$action = 'remove';
  }
  zoneID = 0;
  zoneList = [];
  myChart.setOption(option, false);
  var jsonarray = eval(option.graphic);
  var tmp = [];
  tmp.push(jsonarray[0]);
  option.graphic = tmp;
}
function getbit(x, y) {
  return ((x) >> (y) & 1);
}
function ShowZoneMSG(flag, event, active) {


  var text = '防区:' + active + '\n';
  if (flag % 2 == 1) {
    //错误信息
    if (getbit(event, 0) == 1) {
      text += '供电不足\n'
    }
    if (getbit(event, 1) == 1) {
      text += '电机堵转足\n'
    }
    if (getbit(event, 2) == 1) {
      text += '测距模块温度过高\n'
    }
    if (getbit(event, 3) == 1) {
      text += '网络错误\n'
    }
    if (getbit(event, 4) == 1) {
      text += '测距模块无输出\n'
    }
  }
  if (flag >= 0x100) {
    //防区信息
    if (getbit(event, 12) == 1) {
      text += '观察！！！\n'
    }
    if (getbit(event, 13) == 1) {
      text += '警戒！！！\n'
    }
    if (getbit(event, 14) == 1) {
      text += '报警！！！\n'
    }
    if (getbit(event, 15) == 1) {
      text += '遮挡！\n'
    }
    if (getbit(event, 16) == 1) {
      text += '无数据\n'
    }
    if (getbit(event, 17) == 1) {
      text += '无防区设置\n'
    }
    if (getbit(event, 18) == 1) {
      text += '系统内部错误\n'
    }
    if (getbit(event, 19) == 1) {
      text += '系统运行异常\n'
    }
    if (getbit(event, 20) == 1) {
      //和上面的第四项重复，这里屏蔽
      //text+='网络错误\n'
    }
    if (getbit(event, 21) == 1) {
      text += '设备更新中\n'
    }
    if (getbit(event, 22) == 1) {
      text += '零位输出\n'
    }
  }
  console.log(text);
  option.graphic[0].style.fill = '#FF3727';
  option.graphic[0].style.text = text;
}
