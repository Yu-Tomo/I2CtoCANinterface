#include <SPI.h>
#define SS 9
/*EEPROM*/
#define OPECODE_S 0b00101000
#define OPECODE_R 0b00110000
/**/
/*設定ファイルROM先頭アドレス*/
#define ROM_TOP_ADD 0        //ROMに書き込む先頭アドレス
/**/
/*使用するセンサー設定*/
#define SENSOR_ADDR_NUM 5      //接続するセンサーの数(ic2アドレス数)を定義します
#define SIZE 6                 //センサーから受け取るバイト数の最大値を定義しますSIZE2>=(i2cでセンサーから受け取る最大バイト数)
#define RESINIT_ADDR_NUM 21     //初期化するレジスタの数を定義します
//固定
#define RESINIT_SIZE 3         //レジスタ初期化時に送るデータの最大サイズを定義します
//
#define CAN_ID_SIZE 2          //CANIDのサイズを設定します固定
/**/
/*センサーアドレスと受け取るバイト数*/
//format{i2cデバイスアドレス,ほしいデータのレジスタアドレス,ほしいbyte数}    
static const uint8_t SENSOR_ADDR[SENSOR_ADDR_NUM][3]={{0x29,0x14+10,2},{0x1C,0x28,6},{0x39,0x80|0x20|0x14,2},{0x5A,0x02,2},{0x68,0x88,2}};   //通信するセンサーのアドレスと受け取りたいバイト数を定義します。データは二次元配列です
static const uint8_t RESISTER_INIT_NUM[RESINIT_ADDR_NUM]={3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,3,2};           //初期化したいレジスタのごとに、i2cで送信するバイト数を設定します。
//format{i2cデバイスアドレス,初期化したいレジスタアドレス,初期値}
static uint8_t sensor_init_data[RESINIT_ADDR_NUM][RESINIT_SIZE] = {{0x29,0x88,0x00},{0x29,0x80,0x01},{0x29,0xFF,0x01},{0x29,0x00, 0x00},{0x29,0x91,60},{0x29,0x00,0x01},{0x29,0xFF,0x00},{0x29,0x80,0x00},{0x29,0x00,0x02}//距離センサ
                                                                  ,{0x1C,0x22,0x00}                                                                                                                      //磁気センサ
                                                                  ,{0x39,0x80|0x20|0x01,0xC0},{0x39,0x80|0x20|0x0F,0x00},{0x39,0x80|0x20|0x0D,0x00},{0x39,0x80|0x20|0x00,0x03}                           //照度センサ
                                                                  ,{0x5A,0xFF,0x11},{0x5A,0xFF,0xE5},{0x5A,0xFF,0x72},{0x5A,0xFF,0x8A},{0x5A,0xF4,0xFF},{0x5A,0x01,0x40}                                 //CO2センサ
                                                                  ,{0x68,0x98,0xFF}};                                                                                                                    //AD変換機 
                                                                  //各センサーのレジスタに初期値を設定します。  

/*CANのデバイスID*/    
static int CAN_DEVICE_ID[SENSOR_ADDR_NUM] = {0x0700,0x0701,0x0702,0x0703,0x0704};             //接続するセンサーごとにIDを設定します。 


void setup() {
  Serial.begin(115200);
  pinMode(SS,OUTPUT);
  digitalWrite(SS,LOW);
  SPI.begin();

  SPI.setDataMode(SPI_MODE0);
  // SPI送受信用のビットオーダー(MSBFIRST)
  SPI.setBitOrder(MSBFIRST);
  // SPIクロック分周器 2MHz = 16MHz/8 
  SPI.setClockDivider(SPI_CLOCK_DIV8);  

  /*設定ファイル書き込み*/
  
  digitalWrite(SS,HIGH);
  // EWEN命令(書き込みの有効) 
  SPI.transfer(0b10011000);
  SPI.transfer(0b00000000);
  digitalWrite(SS,LOW);
  delay(10);

  int adderCont =ROM_TOP_ADD;
  //0番地に取得したいレジスタ数を格納する
  int can_id =0;
  writeROM(adderCont,SENSOR_ADDR_NUM);
  adderCont++;
  //
  //CAN IDを入力する
  for(int i=0;i<SENSOR_ADDR_NUM;i++)
  {
    can_id = CAN_DEVICE_ID[i];
    for(int j=CAN_ID_SIZE;j>0;j--)
    {
      writeROM(adderCont,(byte)((can_id >> (8*(j-1)))& (0xFF)));  
      adderCont++;
    }
  }
  //
  //初期化したいレジスタの数
  writeROM(adderCont,RESINIT_ADDR_NUM);
  adderCont++;
  //初期化したいレジスタ数*RESISTER_INIT_NUM[]バイト
  for(int i=0;i<RESINIT_ADDR_NUM;i++)
  {
    writeROM(adderCont,RESISTER_INIT_NUM[i]);
    adderCont++;
    for(int j=0;j<RESISTER_INIT_NUM[i];j++)
    //for(int j=0;j<3;j++)
    {
      writeROM(adderCont,sensor_init_data[i][j]);
      adderCont++;
    }
  }
  //センサーから受け取る最大バイト数
  //writeROM(adderCont,SIZE);
 
  /*センサーから受け取るデータのレジスタとバイト数*/
  for(int i=0;i<SENSOR_ADDR_NUM;i++)
  {
    for(int j=0;j<3;j++)
    {
      writeROM(adderCont,SENSOR_ADDR[i][j]);
      adderCont++;
    }
  }
  /*終端バイト*/
  writeROM(adderCont,0xFF);
  adderCont++;
  writeROM(adderCont,0xFF);
  readROM(200);

   // EWDS命令(書き込みの無効→読み込み専用)    
    //SPI.transfer(0b00000001);  
    //SPI.transfer(0b00000000);    
}
bool writeROM(int adder,byte data)
{
  byte dt1,dt2;
  dt1 = (OPECODE_S | (adder & 0b0000011100000000) >> 8);      //オペコード101とアドレス11bit
  dt2 = (adder & 0x00FF);
  digitalWrite(SS, HIGH);
  // Write命令
  SPI.transfer(dt1);
  SPI.transfer(dt2);
  //データ書き込み
  SPI.transfer(data);
  digitalWrite(SS,LOW);
  delay(10);
  
  return true;
}
void readROM(int num)
{
  byte dt1;
  byte dt2;
  for (int i=0; i< num; i++)
  {
    dt1 = (OPECODE_R | (i & 0b0000011100000000) >>8);
    dt2 = (i & 0x00FF);
    digitalWrite(SS, HIGH);
    // READ命令　+　アドレス
    SPI.transfer(dt1);
    SPI.transfer(dt2);
    delay(5);
 
    // ビットがずれているので2byteを取得する
    uint8_t b1 =  SPI.transfer(0x00);
    uint8_t b2 =  SPI.transfer(0x00);
 
    // ビットを修正後、シリアルモニタへ出力
    Serial.print(i);
    Serial.print("番地 = ");
    Serial.println(((b1 << 1) | (b2 >> 7)),HEX);

    digitalWrite(SS, LOW);
  } 
} 
void loop() {
  // put your main code here, to run repeatedly:

}
