# 7-Outdoor travel    
一.问题定义
-----------------------------------
户外旅行可以是一种休闲的娱乐方式，也可能是一种探险挑战。无论如何，在户外旅行时，一个独立的，多功能的便携设备能使用户得到良好的体验。对自身所处环境是否适宜的判断；分开行动时交流的工具——手机是一个比较全面，功能强大的检测、通讯工具，但在户外很多情况下，手机信号并不稳定，甚至没有，这时候一个能在通讯工具匮乏情况下能完成基础环境的检测以及简单通讯的工具便有着可观的作用。同伴间相互信息的传递（如体温，心跳，相对距离）可以及时了解相互的状况来确定行动方针，并在有同伴出现危险情况下的时候能即使得知，减少可能发生的不好结果。在遭遇紧急情况时，手机联络需要时间，也不能保证联系成功。我们可以利用手表进行简单操作即可发出求救信号，提醒同伴你遭遇危险，以便即使采取措施展开营救。本项目应用于智能SOS求救手表，基于arduino环境开发，借助Lora模块通信，实现身体基本信息传递和求救信号的发出与接收。

二.国内外产品现状
-----------------------------------
在户外运动领域已经有多功能的手表可以满足大部分需求。颂拓户外表有双星导航，运动模式选择，暴风雨预警等功能，是一款及其专业的户外运动手表；而交流方面，强穿透对讲机可以做到手机弱信号地区的交流。而寻迹等功能小天才智能手表也为父母提供了相当可靠的保证。虽然缺少能将信息传到同伴设备的功能，但社区和设备信息上传功能已是智能手表的标配，当前功能在现在市场已有了相当完善的产品。

# 需求分析访谈提纲设计 
a.户外旅行是喜欢个人旅行还是团队旅行？  
b.户外旅行一般去什么地形的景区？  
c.你是否喜欢登山.潜水.滑翔伞等考验身体素质和心理素质的项目？  
d.户外旅行的过程中是否会担心发生意外（迷路.失联等）？  
e.如果在户外旅行中发生意外事故，自身是否具备自救意识和自救常识?  
f.你认为一款既能检测外界环境，又能监测生命体征，还能进行对话的运动手表对于户外旅行是否有必要？  
g.如果设计一款户外旅行的运动手表，你最看重的功能是什么？  
h.对产品的外观、造型、形式上有什么要求？  
i.市面上的产品你认为有什么缺陷？  
j.你对以下功能有何评价？  

三.项目作者
-----------------------------------
组长：2019112554钟豪（github ID: Hao-Zhong）  
组员：2019112503王俊辉（github ID: Hui-Wang-1）  
组员：2019112565刘镇语（github ID: loudmin）  
组员：2019112570张致平（github ID: gird-z）  
组员：2019112576易祯（github ID: YZ2576）  
组员：2019112580李羽鹏（github ID: zzmatti）  

### 物料清单
| 序号 | 名称 | 规格/型号 | 封装 | 品牌 | 用量 | 商品编号 |
|:---|:---|:---|:---|:---|:---|:---|
|1|保险电阻|51Ω(510) ±5%|Axial|1|台湾双羽|C274979|
|2|2.54杜邦线母对母单P同向|2-1571-28|L=210mm|10|华宇创| C369059|
|3|TMB12A12（蜂鸣器）|TMB12A12|Through Hole 7.62|1|华能|C96079|
|4|SX1278LoRa扩频无线模块|Ra-01|   /|2| Ai-Thinker|C90040|
|5|12x12x7.3 塑料头（按钮）|12*12*7.3塑料头|Through Hole 7.62|1|BOOMELE|C1086|  
|6|ESP32开发板|WT-ESP32_DeKitC_V4|   /|2|Wireless-tag|C719319| 
|7|排针 1*12P 1.27mm 直插|210-1S-1*12P|Through Hole,P=1.27mm|2|Ckmtw|C124353|  
|8|(面包板)DS1136-23-400SNV-Q-Y|DS1136-23-400SNV-Q-Y|82mmx53mm|1|CONNFLY|C93522|   

四.代码
-----------------------------------
### 发射端代码
```
#include <LoRaNow.h>
#include <WiFi.h>
#include <WebServer.h>
#include <StreamString.h>
#include <PubSubClient.h>
#include <WiFiClient.h> 
#include <HTTPClient.h>

//vspi for lora radio module
#define MISO 19
#define MOSI 23
#define SCK 18
#define SS 5
#define DIO0 4

#define buttonPin 2
#define ssid "Redmi"                           //WIFI名称
#define password "xxxxxx"                      //WIFI密码
String uid = "0efd8e4ecc13be0817c81f47eb6f1c30";             // 用户私钥，巴法云控制台获取
String type = "1";                                           // 1表示是预警消息，默认即可
String device = "求助信号发射装置";                             // 设备名称
String msg = "请帮帮我！";                                     //发送的消息
String msg2 = "锤子";                                         //消息备注
int delaytime = 2;                                          
String ApiUrl = "http://ai.bemfa.com/api/wechat/v1/";        //默认 api 网址

static uint32_t lastWiFiCheckTick = 0;

int state_1=1;//变量存储按钮的状态，这里请根据传感器默认返回值输入
int state_2=1;//存储上一个时间状态，初始值和state_1的初始值保持一致

//for TMP Sensor,MCP9701A
#define tmpADPin 35
double tmp;
//-----------------------------------------------------
//WIFI重新连接函数
void startSTA(){
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

//WIFI状态检测函数，如果WIFI断开自动重连
void doWiFiTick(){
    if ( WiFi.status() != WL_CONNECTED ) {
        //未连接1s重连
        if (millis() - lastWiFiCheckTick > 1000) {
          lastWiFiCheckTick = millis();
          startSTA();
        }
      }
 }
 
//微信消息推送函数
void doHttpStick(){  //微信消息推送函数
  HTTPClient http;    //Declare object of class HTTPClient
  String postData;
  //Post Data
  postData = "uid="+uid+"&type=" + type +"&time="+delaytime+"&device="+device+"&msg="+msg+"&msg2="+msg2;
  http.begin(ApiUrl);              //Specify request destination
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(postData);   //Send the request
  String payload = http.getString();    //Get the response payload
  http.end();  //Close connection
  Serial.println("send success");  
  }

void setup() {
  Serial.begin(115200);
  
  pinMode(buttonPin, INPUT);   // declare sensor as input
  WiFi.mode(WIFI_OFF);        //Prevents reconnection issue (taking too long to connect)
  delay(1000);

//第一次尝试连接WIFI
  WiFi.mode(WIFI_STA);        //This line hides the viewing of ESP as wifi hotspot
  
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  Serial.print("Connecting");
  if (WiFi.status() != WL_CONNECTED)        //链接失败
  {
    delay(500);
    Serial.print("连接失败！");
  }
  else if (WiFi.status() == WL_CONNECTED)   //连接成功
  {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());  //IP address assigned to your ESP
  }
  
  Serial.println("LoRaNow Simple Node");
  pinMode(tmpADPin,INPUT);

   LoRaNow.setFrequencyCN(); // Select the frequency 486.5 MHz - Used in China
  // LoRaNow.setFrequencyEU(); // Select the frequency 868.3 MHz - Used in Europe
  // LoRaNow.setFrequencyUS(); // Select the frequency 904.1 MHz - Used in USA, Canada and South America
  // LoRaNow.setFrequencyAU(); // Select the frequency 917.0 MHz - Used in Australia, Brazil and Chile

  // LoRaNow.setFrequency(frequency);
  // LoRaNow.setSpreadingFactor(sf);
  // LoRaNow.setPins(ss, dio0);

   LoRaNow.setPinsSPI(SCK, MISO, MOSI, SS, DIO0); // Only works with ESP32

  if (!LoRaNow.begin()) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  LoRaNow.onMessage(onMessage);
  LoRaNow.onSleep(onSleep);
  LoRaNow.showStatus(Serial);
}

void loop() {
  tmp = readTMP();
  doWiFiTick();
  state_1 = digitalRead(buttonPin);
//
  if((state_1 == HIGH) && (state_2 == LOW)) //按下按钮时
  {
    Serial.println("请帮帮我");
    doHttpStick();              //在想推送消息的地方执行推送函数
  }
  LoRaNow.loop();
  state_2 = state_1;
}

double readTMP()
{
  int RawValue = analogRead(tmpADPin);
  double Vout = (RawValue / 4096.0) * 3270; // 单位mV.
  double tempC = (Vout - 0.4)/19.5;
  //Serial.println(tempC);
  return tempC;
}

void onMessage(uint8_t *buffer, size_t size)
{
  Serial.print("Receive Message: ");
  Serial.write(buffer, size);

  Serial.println();
  Serial.println();
}

void onSleep()
{
  Serial.println(tmp);                                         //打印温度信息
  if((tmp<30) | ((state_1 == HIGH) && (state_2 == LOW)))      //当温度低于一定程度或按下按钮时向手机发送求救信号
  {
    LoRaNow.print("help");
  }
  LoRaNow.send();
}
```
### 接收端代码
```
#include <LoRaNow.h>
#include <pitches.h>
#define MISO 19
#define MOSI 23
#define SCK 18
#define SS 5
#define DIO0 4
#define pinBuzzer 2
const char * a;
const char * c="help";
void setup() {
  Serial.begin(115200);
  pinMode(pinBuzzer, OUTPUT);
  LoRaNow.setFrequencyCN(); // Select the frequency 486.5 MHz - Used in China
  LoRaNow.setPinsSPI(SCK, MISO, MOSI, SS, DIO0); // Only works with ESP32
  if (!LoRaNow.begin()) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  LoRaNow.onMessage(onMessage);
  LoRaNow.showStatus(Serial);
}

void loop() {
  LoRaNow.loop();
}

void onMessage(uint8_t *buffer, size_t size)
{
  a=(const char *)buffer; //将接受到的信号进行强制类型转换，便于判断
  if ( strcmp( a, c) == 0 )//判断接受到的信号是否是求救信号
  {
     //模拟求救信号
     for(int i=200;i<=800;i++)//用循环的方式将频率从200HZ 增加到800HZ
     {
        pinMode(pinBuzzer,OUTPUT);
        tone(pinBuzzer,i);   //在2号端口输出频率
        delay(5);            //该频率维持5毫秒   
     }
     delay(4000);            //最高频率下维持4秒钟
}
```
### 代码如何运行
```
代码是基于arduino esp32开发板进行编写的，
代码运行首先需要进行esp32环境离线搭建：
下载压缩包arduino-esp32-master，在你的Arduino安装路径\hardware下新建一个文件夹，起名为expressif，并将压缩包解压到文件夹里。
然后进入文件夹，点击expressif>arduino-esp-32-master>tools,找到get.exe并右键以管理员身份运行。进入Arduino IDE，这时应该可以看到ESP32
的板了。
代码成功编译需要下载一个“LaRaNow.h”库和一个“pitches.h”库，代码才能正常运行    
```    
五.团建计划
-----------------------------------
1.团队建设的意义  
1)提高团队运行的灵活性。   
2)充分发挥每个人的优点，实现团队人员间的优势互补。   
3)鼓励个人和集体的团队发展和改进，更好地专注于团队目标。   
4)通过团队的分析讨论不断优化改进团队目标。  
5)增强团队意识，加强团队间的相互信任。  

2.团队建设的目标  
1)明确一个共同的团队目标，团队成员凝聚在一起共同为之努力。  
2)实现合理的工作分配，避免某个人任务过于繁重或清闲。  
3)实现团队内的相互信任，团队成员能公开交流想法，尊重差异求同存异。  
4)实现有效的沟通，团队应具备各方面、多方位、线上与线下、私人与整体的交流渠道。  

3.团队建设的方法  
1)通过协商做出决定，共同探讨项目主题的选取，尽量符合所有人的想法，得不到一致结果是投票或由组长决定。  
2)项目进行时做好明确的工作分配，每一个部分或环节都能找到它的负责人，并定期汇报总结任务进程，灵活调整任务分配，有利于调动团队成员的积极性，提高工作效率，保证工作质量。  
3)保持团队内的良好沟通，建立团队群聊，有利于集思广益，作出科学决策。每个人都及时汇报自己的工作进展，也可以根据需要与某部分工作的负责人详细交流项目细节，通过有效的信息沟通，建立良好的团队气氛，有利于减少工作失误，加强工作协作与配合，减少各种浪费，最终提高项目组织效益。  
4)每个项目成员都有个人安全，情感交流，尊重和自我价值实现的需要。加强项目团队管理。创造竞争，公平和发展的气氛与环境，有利于团队成员努力工作，证明自己的价值，增强其成就感，在团队中使成员相互关心，相互帮助，相互尊重。  
  
4.头脑风暴——概念关键词
  a.便于携带（钟豪）--对户外旅行的基本需求  
  b.判断相对距离，显示位置（刘镇语）--安全需求、突发状况需求  
  c.增强续航能力（张致平）--产品优化功能  
  d.保证稳定性（王俊辉）--提高产品可用性  
  e.生命体征检测（李羽鹏）--刘镇语：遇到危险发出SOS信号（补充）  
  f.温湿度显示（易祯）--丰富产品功能    
  ![image](https://github.com/SWJTU-i2e-2020/7-Outdoor-travel/blob/main/%E6%A6%82%E5%BF%B5%E8%8D%89%E5%9B%BE.jpg)  
 
