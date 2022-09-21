#必要なものをインポート
import os
import select
import termios
import tty
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import sys #キーボード入力
import serial #シリアル通信


class Serial(Node):

    
    #コンストラクタ
    def __init__(self):
        super().__init__('YAMAHA')

        qos = QoSProfile(depth=1)
        self.subscription = self.create_subscription(
            Twist,
            'yamaha/cmd_vel',
            self.WriteRead,
            qos)

        self.subscription
        self.linear_x = 0
        self.angular_z = 0

        self.request = True
        self.callBack = True
        
        #シリアル通信設定
        self.serial = serial.Serial(
        port="/dev/ttyUSB0",
        baudrate=38400,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=2)

        self.RequestSend()


    def RequestSend(self):

        #リクエストコマンド配列
        rCommand = ['header', 'SA', 'DA', 'FNO', 'CMD', 'BC', 'MODE','DATA1', 'DATA2', 'DATA3', 'timeOut', 'DATA5', 'DATA6', 'DATA7', 'checkSum']
        rCommand[0] = 0x01; #ヘッダ
        rCommand[1] = 0x08; #ソースアドレス
        rCommand[2] = 0x03; #ディスティネーションアドレス 
        rCommand[3] = 0x01; #通信番号 
        rCommand[4] = 0x68; #コマンド
        rCommand[5] = 0x08; #データ長
        rCommand[6] = 0x01; #モード（アカデミックモード＝01h）
        rCommand[7] = 0x00;
        rCommand[8] = 0x00;
        rCommand[9] = 0x00;
        rCommand[10] = 0x00; #タイムアウト（無効＝00h）
        rCommand[11] = 0x00;
        rCommand[12] = 0x00; 
        rCommand[13] = 0x00;

        #チェックサム
        sum =0
        for i in range(1, 13, 1):
            sum += rCommand[i]
        rCommand[14] = sum

        self.serial.write(rCommand)
        self.RequestReceive()

    def RequestReceive(self):
        line = self.serial.readline()
        #self.line = self.serial.read(14)
        print('[RESULT]')
        print(line)
        #self.WriteRead()

    def WriteRead(self,twist):
        self.linear_x = twist.linear.x
        self.angular_z = twist.angular.z 
        self.get_logger().info('I heard: "%s"' % twist.linear.x)
        self.get_logger().info('I heard: "%s"' % twist.angular.z)
        linear = self.linear_x
        angular = self.angular_z
        gear = 12.64
        perimeter = 1.22
        distance = 0.24 #中心から車輪までの距離
        Angular = angular/3.6

        if angular == 0:
          turning_radius = 0
          rpm_right = (1000*linear*gear)/(60*perimeter)
          rpm_left = (1000*linear*gear)/(60*perimeter)
        else :
          #旋回半径
          turning_radius = (linear)/(angular)
          #rightのタイヤ
          velocity_right =(turning_radius + distance)*(Angular)
          Velocity_right =velocity_right*3.6
          rpm_right = (1000*Velocity_right*gear)/(60*perimeter)
          #leftのタイヤ
          velocity_left =(turning_radius - distance)*(Angular)
          Velocity_left =velocity_left*3.6
          rpm_left = (1000*Velocity_left*gear)/(60*perimeter)


        wrCommand = ['header', 'SA', 'DA', 'FNO', 'CMD', 'BC', 'UNIT','NONE', 
                     'JS1', 'JS2', 'JS3', 'JS4', 'RSPEED1', 'RSPEED2', 'LSPEED1', 'LSPEED',
                     'DATA1', 'DATA2','DATA3', 'DATA4','DATA5', 'DATA6', 'checkSum']
        wrCommand[0] = 0x01; #ヘッダ
        wrCommand[1] = 0x08; #ソースアドレス
        wrCommand[2] = 0x03; #ディスティネーションアドレス 
        wrCommand[3] = 0x01; #通信番号 
        wrCommand[4] = 0x69; #コマンド
        wrCommand[5] = 0x10; #データ長
        wrCommand[6] = 0x02; #ユニット有効指示（モータ回転指示値＝02h）
        wrCommand[7] = 0x00; 
        wrCommand[8] = 0x00; #JoyStick
        wrCommand[9] = 0x00; #JoyStick
        wrCommand[10] = 0x00; #JoyStick
        wrCommand[11] = 0x00; #JoyStick
        wrCommand[12] = (int(rpm_right) & 0x00FF); #右モーター回転指示（下位バイト） (-1000rpm~1000rpm)
        wrCommand[13] = ((int(rpm_right) & 0xFF00) >> 8); #右モーター回転指示（上位バイト） (-1000rpm~1000rpm)
        wrCommand[14] = (int(rpm_left) & 0x00FF); #左モーター回転指示（下位バイト） (-1000rpm~1000rpm)
        wrCommand[15] = ((int(rpm_left) & 0xFF00) >> 8); #左モーター回転指示（上位バイト） (-1000rpm~1000rpm)
        wrCommand[16] = 0x00; 
        wrCommand[17] = 0x00;
        wrCommand[18] = 0x00; 
        wrCommand[19] = 0x00;
        wrCommand[20] = 0x00; 
        wrCommand[21] = 0x00;

        #チェックサム
        sum2 =0
        for i in range(1, 22, 1):
            sum2 += wrCommand[i]
        wrCommand[22] = ((sum2 & 0x00FF));

        #送信
        self.serial.write(wrCommand)

        #受信
            
        line2 = self.serial.read(23)
        print(line2)

           
        
def main(args=None):
    rclpy.init(args=args)

    s = Serial()

    rclpy.spin(s)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
