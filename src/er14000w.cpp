#include "ros/ros.h"
#include "std_msgs/Float64.h"

const int NUMPARAM = 10; // param[] の配列の要素数、必要なら増やせる

// 以下は、個々の関節、モータを区別するための ID 番号
// timerCallback() の各インスタンスが持つこの値を調べることで
// 関節毎に異なった異なった動きを実現できる
// 関節の個数は必要なだけ増やすことが可能
// 関節を増やす場合、様々なファイルに設定が必要なのでマニュアルをよく読む

const int J1 = 1; // id number of joint 1
const int J2 = 2;
const int J3 = 3;
const int J4 = 4;
const int J5 = 5; // id number of joint 5
const int J6 = 6; // id number of joint 5
const int J7 = 7; // id number of joint 5
const int J8 = 8; // id number of joint 5
const int J9 = 9; // id number of joint 5
const int J10 = 10; // id number of joint 5

class Control
{
public:
  Control(char *name, int id, float param[NUMPARAM]); // コンストラクタ
  ~Control(); // デストラクタ
  void timerCallback(const ros::TimerEvent&);
  void timerCallback2(const ros::TimerEvent&); // 別の実装例、書き方と関節の動き方が異なる
private:
  int id; // 関節を区別するための ID
  int timing; // timerCallback() が呼び出された回数を記憶する、時間の管理用
  float angle1; // 回転角度の限界値１
  float angle2; // 回転角度の限界値２
  float targetangle; // その時々の目標角度を記憶
  int cycle; // 動きを切り替える周期を関節毎に個別に指定できるようにした
  ros::Publisher c_pub;
  ros::Subscriber p_sub;
  ros::Timer timer;
  ros::NodeHandle nh;
};

Control::Control(char *name, int idnum, float param[NUMPARAM])
{
  c_pub = nh.advertise<std_msgs::Float64>(name, 1000);

// 0.1sec 毎にtimerCallback() が呼び出されるように登録する  
// timer = nh.createTimer(ros::Duration(0.1), &Control::timerCallback, this);

// timerCallback() の別の実装 timerCallback2() を作ってみた 
  timer = nh.createTimer(ros::Duration(0.1), &Control::timerCallback2, this);

  id = idnum; // 渡された値を変数 id にセットする
  timing = 0; // この変数は timerCallback() が呼び出された回数を数えて記憶している
  angle1 = param[0]; // 渡された param[] を各変数にセットする
  angle2 = param[1]; // 最大10個まで指定できるが、現状は3個だけ使用
  cycle  = (int)param[2];
  targetangle = angle1; // 最初の目標角度は angle1 にしている
}

Control::~Control()
{
// Nothing to do now.
}

// モータの目標角度はこの関数で決められている
// ros_Duration が 0.1 なので、(0.1 * cycle) 秒ごとに目標角度を変更できる
void Control::timerCallback(const ros::TimerEvent&)
{
    std_msgs::Float64 pos;
    
    if (cycle == timing) {
        timing = 0;
	if (angle1 == targetangle) targetangle = angle2;
	else                       targetangle = angle1;
//	printf("%d:%f ", id, targetangle);
//	if (J5 == id) {printf("\n"); fflush(stdout);}
    }
    timing++;
    pos.data = targetangle;

// 以下は、新たに加えた部分
// 上記の目標角度の入れ替え以外に各関節に固有の特別な処理をしたい
// 場合はここに書ける。
  
    switch (id) {
    case J1:
//    nothing;
      break;
    case J2:
//    nothing;
      break;
    case J3:
//    if (10 < cycle) cycle--;
      break;
    case J4:
//    nothing;
      break;
    case J5:
//    nothing;
      break;
    default:
//    nothing;
      break;
    }

    c_pub.publish(pos);
}

// timerCallback() の別の書き方の例、こちらの方がわかりやすいかも
// モータの目標角度はこの関数で決められている
// ros_Duration が 0.1 なので、(0.1 * cycle) 秒ごとに目標角度を変更できる
void Control::timerCallback2(const ros::TimerEvent&)
{
    std_msgs::Float64 pos; // 目標角度を publish する時に使うデータの型
    
    switch (id) { // id によって処理を変えることでモータ毎に異なった動きを実現できる
    case J1: // 上部のリンクの肩部分
	if (cycle == timing) { // cycle=30 なので 3 秒毎に目標角度が変わる
	    timing = 0; // この関数が呼び出された回数をリセットする
	    if (angle1 == targetangle) targetangle = angle2; // この2行で2つの角度を
	    else                       targetangle = angle1; // 3秒毎に切り替える
	}
	break;
    case J2: // 上部のリンクの肘
	if (20 == timing) { // 2 秒毎に目標角度が変わる
	    timing = 0;
	    if (angle1 == targetangle) targetangle = angle2;
	    else                       targetangle = angle1;
	}
	break;
    case J3: // 上部のリンクの手首
// 何も指定しなければ最初の目標角度に達したらその後動かない
	break;
    case J4: // 赤色X軸方向のL字形のリンクの根本
	if (cycle == timing) {
	    timing = 0;
	    static int sign = 1; // 動く方向を決めるフラグ
	    const float a90 = (M_PI*3/6); // 90 度 
	    const float a30 = (M_PI*1/6); // 30 度 
	    const float ma30 = (-M_PI*1/6); // -30 度 
	    const float ma90 = (-M_PI*3/6); // -90 度 
// 複数の角度を往復する動き
	    if (angle1 == targetangle) {targetangle = a90; sign = -1;}
	    else if (a90 == targetangle) {
		if (1 == sign) targetangle = angle1; // 角度を増す場合
		else           targetangle = a30;    // 角度を減らす場合
	    }
	    else if (a30 == targetangle) {
		if (1 == sign) targetangle = a90;
		else           targetangle = ma30;
	    }
	    else if (ma30 == targetangle) {
		if (1 == sign) targetangle = a30;
		else           targetangle = ma90;
	    }
	    else if (ma90 == targetangle) {
		if (1 == sign) targetangle = ma30;
		else           targetangle = angle2;
	    }
	    else if (angle2 == targetangle) {targetangle = ma90; sign = 1;}
	}
	break;
    case J5: // 緑色Y軸方向のリンクの根本
	if (cycle == timing) {
	    timing = 0;
	    if (angle1 == targetangle) targetangle = angle2;
	    else                       targetangle = angle1;
	}
	break;
    default:
//    nothing;
      break;
    }

    timing++; // 関数が呼び出された回数を１増やす
    pos.data = targetangle;
    c_pub.publish(pos); // 目標角度を Gazebo に送信する
}

int main(int argc, char **argv)
{
// 以前は目標角度の値を１個しか指定できなかったが、他の値も渡せるように
// 配列にした。最大10個まで。
  float param[NUMPARAM];
  
  ros::init(argc, argv, "joint_controller");

// ジョイントのID番号を引数で渡せるようにした。
  
// joint 毎に専用のコントローラのクラスのインスタンスを生成する。
// 各コントローラに固有のパラメータを渡したいときは、param[] 経由で渡す。
// 例として、param[0]は角度１、param[1]は角度２、param[2]は周期としてみた。
// これらの値を書き換えてみると動きが変化するのが見られるはず。
  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30; // 3秒
  Control Control1((char*)"/ri4/joint1_position_controller/command", J1, param);
  
  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30;
  Control Control2((char*)"/ri4/joint2_position_controller/command", J2, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30;
  Control Control3((char*)"/ri4/joint3_position_controller/command", J3, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30;
  Control Control4((char*)"/ri4/joint4_position_controller/command", J4, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30;
  Control Control5((char*)"/ri4/joint5_position_controller/command", J5, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30; // 3秒
  Control Control6((char*)"/ri4/joint6_position_controller/command", J6, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30; // 3秒
  Control Control7((char*)"/ri4/joint7_position_controller/command", J7, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30; // 3秒
  Control Control8((char*)"/ri4/joint8_position_controller/command", J8, param);

  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30; // 3秒
  Control Control9((char*)"/ri4/joint9_position_controller/command", J9, param);
  
  param[0] = M_PI/6.0; // 30 度
  param[1] = -M_PI/6.0; // -30 度
  param[2] = 30; // 3秒
  Control Control10((char*)"/ri4/joint10_position_controller/command", J10, param);

  ros::spin();

  return 0;
}
