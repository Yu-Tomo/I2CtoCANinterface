#include <SPI.h>
#include <Wire.h>
#include <CAN.h> 
#include <avr/io.h>
#define SS 9                                               //ROM用のスレーブセレクトピン
#define DEBUG
#define DEBUG_THERE_IS_I2CDATA
#define DEBUG_DELAY
#define DEBUG_DELAY_TIME 100
//EEPROM
#define OPECODE_R 0b00110000
//
//ROMデータ用マクロ定義
#define CANID_TOP_ADD 1
#define CAN_ID_SIZE 2
//
#define I2C_DATA_SIZE 3                                     //ヒープメモリデータサイズ
/*CAN通信速度*/
#define CAN_SPEED 500000     //bps
/*CAN*/
#define MESSAGE_PROTOCOL  1                                 // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
//#define MESSAGE_LENGTH    8                               // Data length: 8 bytes
#define MESSAGE_RTR       0                                 // rtr bit
/**/
//設定ファイルの先頭アドレス
int topAddress = 0;                                      
int read_topAdd=0;
int SENSOR_NUM=0;

void setup() {
  Serial.begin(115200);
  int i;
  uint8_t bookmark;                                        //0xFFのアドレスを探すための変数
  byte setting_file = 0;                                   //設定ファイルの番号
  int move_add=0;                                          //アドレス計算用
  int iWork;
  byte data_num;
  byte bWork;

  while(!Serial);
  if (!CAN.begin(CAN_SPEED)) {
#ifdef DEBUG
    Serial.println("Starting CAN failed!");
#endif
    while (1);
  }
  else
  {
#ifdef DEBUG
    Serial.println("Starting CAN!");
#endif
  }
  Wire.begin();
  Wire.setClock(100000);
  //SPI設定
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  // SPI送受信用のビットオーダー(MSBFIRST)
  SPI.setBitOrder(MSBFIRST);
  // SPIクロック分周器 8MHz = 16MHz/2 
  SPI.setClockDivider(SPI_CLOCK_DIV2);  
  pinMode(SS,OUTPUT);
  //
  //DIPスイッチ分のピンをINPUTにする
  for(int pinCnt=14;pinCnt<18;pinCnt++)
  {
    pinMode(pinCnt,INPUT);
  }
  //
  //設定ファイルを決める
  setting_file = PINC & 0x0F;
 #ifdef DEBUG
  Serial.print("設定ファイル:");
  Serial.println(setting_file,BIN);
#endif
  //選択された設定ファイルの先頭アドレスを探す
  topAddress=get_topadress(setting_file);
  
//デバッグ

#ifdef DEBUG
//設定ファイルサイズ
size_t SETTING_SIZE = 0;
//設定ファイルのサイズを取得
  //SETTING_SIZE=check_dataSize(topAddress);
  ROM_DEBUG();                                         //ROMの中身を確認する
  //Serial.print("設定ファイルサイズ:");                   
  //Serial.println(SETTING_SIZE);                        //データサイズを確認する
#endif
  SENSOR_NUM=iWork = get_ROM_SPI(topAddress);  
  iWork = iWork*2+1;
  
  move_add=topAddress+iWork;                           //初期化したいレジスタの先頭アドレスに有働
  data_num = get_ROM_SPI(move_add);                    //初期化したいレジスタの数を取得
  move_add++;                                          //データの先頭に移動
//レジスタ初期化
#ifdef DEBUG
  Serial.print("センサー数");
  Serial.println(SENSOR_NUM);
  Serial.print("初期化レジスタ数");
  Serial.println(data_num);
#endif
  for(i=0;i<data_num;i++)                               //初期化したいレジスタ数分ループ
  {
    bWork=get_ROM_SPI(move_add);                        //後ろに続くバイト数を取得する
    i2c_device_write(move_add+1,bWork);                 //初期化データを送信する
    move_add+=(bWork+1);                                //次のデータまで移動する
#ifdef DEBUG_DELAY
    delay(DEBUG_DELAY_TIME);
#endif
  }
  read_topAdd = move_add;
}
void loop() {
  byte *heep_ptr;                              //ヒープメモリ用変数
  byte *data_size_ptr;                          //データ操作用ヒープメモリポインタ変数
  byte *i2c_data_heep;
  int i;
  int move_add;
  int can_add;
  int canID;
  bool there_is_i2cData;                      //i2cセンサー値をゲットできた
  while(1)
  {
    move_add = read_topAdd;                    //i2cセンサ値をもらうアドレスに移動
    can_add = topAddress + CANID_TOP_ADD;      //CANIDが入ってるアドレスに移動
    for(i=0;i<SENSOR_NUM;i++)
    {
#ifdef DEBUG
      Serial.print(i);
      Serial.println("回目");
#endif
      heep_ptr = data_size_ptr = (byte *)malloc(I2C_DATA_SIZE);     //i2cセンサーread用データ格納

      //ヒープメモリにデータを書き込む
      input_heep(heep_ptr,I2C_DATA_SIZE,move_add);
      data_size_ptr++;      //センサーから読み出すbyte数まで移動する
      data_size_ptr++; 
      i2c_data_heep=(byte *)malloc(*data_size_ptr);              //センサーデータ格納

#ifdef DEBUG
      Serial.print("get_data_size:");
      Serial.println(*data_size_ptr);

      view_heep_memory(heep_ptr,I2C_DATA_SIZE);                    //ヒープの中身を確認する
#endif
      //i2cセンサーのデータを読み出す
      there_is_i2cData = i2c_device_data_get(heep_ptr,*data_size_ptr,i2c_data_heep);
      //CANIDを取得する
      if(there_is_i2cData == true)
      {
        canID=get_canID(can_add);
                                                       
        //センサーデータをCANに流す
        can_send(i2c_data_heep,*data_size_ptr,canID);
        //
      }
      can_add+=CAN_ID_SIZE;                                        //次のIDまで移動する
      move_add+=I2C_DATA_SIZE;                                      //次のセンサのデータに移動する
      free(i2c_data_heep);
      free(heep_ptr);
    }
  }
}
//ROMからデータを抜き取る
byte get_ROM_SPI(int add)
{
  byte dt1,dt2;
  byte data;
  digitalWrite(SS, HIGH);  
  dt1 = (OPECODE_R | (add & 0b0000011100000000) >>8);
  dt2 = (add & 0x00FF);
  digitalWrite(SS, HIGH);
  // READ命令　+　アドレス
  SPI.transfer(dt1);
  SPI.transfer(dt2);                               //アドレス
        
  data =  SPI.transfer(0x00);
  data = ((SPI.transfer(0x00)>>7)|(data)<<1);       

  digitalWrite(SS, LOW);
  return data;  
}
//設定ファイルの最初のアドレスを求める
byte get_topadress(byte setting_file)
{
  int topadd=0;
  byte bookmark;
  //設定ファイルの位置のアドレスを探す
  for(int i=0;i<setting_file;)
  {
    bookmark=get_ROM_SPI(topadd);
    topadd++;                                                //0xFFの次のアドレスにする
    if(bookmark == 0xFF)                                      //0xFF(終端バイト)の実行
    {
      bookmark=get_ROM_SPI(topadd);
      if(bookmark == 0xFF)
      {
        topadd++;
        i++;
      }
    }
  }
  return topadd;                                             //先頭アドレスを返す
}
//データサイズを確認する関数
int check_dataSize(byte add)
{
  int iCnt;
  byte bookmark =0;
  bool flg = false;
  for(iCnt =0;flg == false;iCnt++)
  {
    bookmark = get_ROM_SPI(add);
    add++;
    if(bookmark == 0xFF)                                      //0xFF(終端バイト)の実行
    {
      bookmark=get_ROM_SPI(add);
      if(bookmark == 0xFF)
      {
        flg = true;
      }
    }
  }
  return iCnt-1;
}
//ROMの中身を確認する
void ROM_DEBUG(void)
{
  byte data;
   for(int i=0;i<200;i++)
  {
    data = get_ROM_SPI(i);
    
    //ビットを修正後、シリアルモニタへ出力
    Serial.print(i);
    Serial.print("番地 ");
    Serial.println(data,HEX);
  }  
}
//ヒープメモリの中身を確認する
void view_heep_memory(byte *setting_heep,int num)
{
  Serial.println("ヒープメモリの中身を表示");
  for(int i=0;i<num;i++)
  {
      Serial.println(*setting_heep,HEX);
      setting_heep++;
  }
}
//ヒープメモリにデータを書き込む
void input_heep(byte *ptr,byte data_size,int add)
{
  int i;
  for(i=0;i<data_size;i++)
  {
    *ptr = get_ROM_SPI(add);
    add++;
    ptr++;
  }
}
/*i2cレジスタに初期値を書き込む*/
void i2c_device_write(int add,byte data_size)
{
  int i=0;
  byte *heep_ptr;                               //ヒープメモリポインタ変数
  byte *heep_ptr_top;                           //ヒープメモリポインタ変数のtopアドレス
  //ヒープメモリを取得する
  heep_ptr = heep_ptr_top = (byte *)malloc(data_size);
  //ヒープに書き込む
  input_heep(heep_ptr,data_size,add);
  //                          
#ifdef DEBUG
    Serial.print("ヒープメモリサイズ:");
    Serial.println(data_size);
    view_heep_memory(heep_ptr,data_size);          //ヒープメモリの中身を確認する
    Serial.print("i2c adder:"); 
    Serial.println(*heep_ptr,HEX);
#endif

    Wire.beginTransmission(*heep_ptr);            //センサーアドレス
    heep_ptr++;
    for(i=0;i<(data_size-1);i++)                  //送るレジスタアドレスと設定データを入れる
    { 
      Wire.write(*heep_ptr);              
#ifdef DEBUG
      Serial.print("i2c send:"); 
      Serial.println(*heep_ptr,HEX);    
#endif
      heep_ptr++;       
    }
     byte error = Wire.endTransmission();
#ifdef DEBUG
    switch(error)
    {
      case 0:
        Serial.println("i2c sendfin"); 
        break;
      case 1:
        Serial.println("send buffer over");
        break;
      case 2:
        Serial.println("send address NACK receved");
        break;
      case 3:
        Serial.println("send data NACK receved");
        break;
      case 4:
        Serial.println("other error");
        Wire.begin();
        break;
      default:
        break;
    }
#endif
//ヒープメモリを開放する
  free(heep_ptr_top);      
}
//canのIDを取得する
int get_canID(byte add)
{
  int local=0;
  int i;
  for(i=0;i<CAN_ID_SIZE;i++)
  {
    local <<= 8;
    
    local |= get_ROM_SPI(add); 
    add++;
  }
  
#ifdef DEBUG
  Serial.print("CAN ID:");
  Serial.println(local,HEX);
#endif
  return local;
}
/*i2c通信でセンサーデータを受け取る*/
bool i2c_device_data_get(byte *i2c_data,byte data_size,byte *data_ptr)
{
  bool there_is_data;
  //i2cセンサからデータを受け取るレジスタを指定
  Wire.beginTransmission(*i2c_data);              //deviceアドレスを指定する
  Wire.write(*(i2c_data+1));                      //データを受けとるレジスタを指定       
  Wire.endTransmission();                         //送信  
  Wire.beginTransmission(*i2c_data);                  
  //センサーアドレスから受け取りたいバイト数分取得
  Wire.requestFrom(*i2c_data,data_size);           //読み取りたいbyte数を指定する
  byte wire_available = Wire.available();
#ifdef DEBUG_THERE_IS_I2CDATA
  wire_available = 1;
#endif
  if(wire_available != 0)
  {
    there_is_data = true;                         //i2cのセンサーからデータをとれた
    for(int i=0;i<data_size;i++)
    {
      *data_ptr=Wire.read(); //データを受け取る
#ifdef DEBUG
      Serial.print("get_data:");
      Serial.println(*data_ptr,HEX);
#endif
      data_ptr++;
    }
  }
  else
  {
    there_is_data = false;                        //i2cのセンサーからデータ取れなかった
  }
  Wire.endTransmission(); 
  return there_is_data;
}
/*取得したセンサーデータをすべて吐き出す*/
void can_send(uint8_t *ptr,uint8_t length,int canID)
{
  CAN.beginPacket(canID);
  for(int i=0;i<length;i++)
  {
    CAN.write(*(ptr+i));
#ifdef DEBUG
    Serial.print("send_can:");
    Serial.println(*(ptr+i),HEX);
#endif
  }
  CAN.endPacket();
}
/***********************/
