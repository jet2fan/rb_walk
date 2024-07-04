//20240217先ずは股y軸の計算結果をグラフに出力する。
#include <stdio.h>
#include <math.h>
#include <assert.h>		//erorr検出装置
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

//sheet "1" 軌道算出
#define ID_MAX 40		//配列変数個数
#define INITIAL_LEG_BENDING_LENGTH 10.0	//初期足曲げ長さ
#define THIGH_LENGTH 40.0	//太股長さ
#define SHIN_LENGTH 40.0	//すね長さ
#define STRIDE_LENGTH 40.0	//歩幅
#define LEG_UP 30.0		//足上げ
#define REFERENCE_ANGLE_HIP_X 0.0	// 股関節基準角度
#define REFERENCE_ANGLE_HIP_Z -1.0	// 股関節基準角度
// 膝の基準角度は無い。すなわち股と膝の間の角度だから
#define REFERENCE_ANGLE_ANKLE_X 0.0	// 踝関節基準角度 -1.0
#define REFERENCE_ANGLE_ANKLE_Z -1.0	// 踝関節基準角度    0.0
//sheet"2"前足 sheet"3"後ろ足 水平移動算出
#define PALLALEL_VERTICAL_MOTION 0.0	// 平行移動上下運動
//sheet "4" 左右振れ幅
#define LEFT_RIGHT_AMPLITUDE 50.0	//左右振れ幅
#define ANKLE_HIP_HIGHT 10.0	//踝xから踝y + 股xから股yまでの距離
#define REFERENCE_ANGLE_HIP_Y_SHEET4 -1.0	// 股関節基準角度
#define REFERENCE_ANGLE_HIP_Z_SHEET4 0.0	// 股関節基準角度
//sheet "5" 歩行全体統括
#define WALKING_PROCESS 4	//歩行プロセス
double count[15];

//classの作成
class LegMotion_xz
{
private:
  //変数定義
  //sheet "1" trajectory 軌道 一歩前
  double array_rad_pi_divide2[ID_MAX];	//rad pi/2
  double array_rad_pi_multiply2[ID_MAX];	//rad 2pi
  double array_ankle_trajectory_x[ID_MAX];	//踝軌道 x
  double array_ankle_trajectory_z[ID_MAX];	//踝軌道 z
  double array_hip_trajectory_x[ID_MAX];	//股関節位置 x
  double array_hip_trajectory_z[ID_MAX];	//股関節位置 z
  double array_hip_trajectory_x1[ID_MAX];	//股関節位置 x1 原点移動
  double array_hip_trajectory_z1[ID_MAX];	//股関節位置 z1 原点移動
  double array_knee_trajectory_x[ID_MAX];	//膝の座標
  double array_knee_trajectory_z[ID_MAX];	//膝の座標
  double array_knee_trajectory_x1[ID_MAX];	//膝の座標 x1 原点移動
  double array_knee_trajectory_z1[ID_MAX];	//膝の座標 z1 原点移動
  double array_ankle_trajectory_x1[ID_MAX];	//踝軌道 x1 原点移動
  double array_ankle_trajectory_z1[ID_MAX];	//踝軌道 z1 原点移動
  double array_hip_angle_trajectory[ID_MAX];	//股角度(rad)
  double array_knee_angle_trajectory[ID_MAX];	//膝角度(rad)
  double array_ankle_angle_trajectory[ID_MAX];	//踝角度(rad)
  //sheet "2" horizontal 水平:前足の動き 重心移動前足
  double array_hip_horizontal_x[ID_MAX];	//股水平移動
  double array_hip_horizontal_z[ID_MAX];	//股水平移動
  double array_hip_horizontal_x1[ID_MAX];	//股水平移動 原点移動
  double array_hip_horizontal_z1[ID_MAX];	//股水平移動 原点移動
  double array_knee_horizontal_x[ID_MAX];	//膝の座標
  double array_knee_horizontal_z[ID_MAX];	//膝の座標
  double array_knee_horizontal_x1[ID_MAX];	//膝の座標
  double array_knee_horizontal_z1[ID_MAX];	//膝の座標
  double array_ankle_horizontal_x[ID_MAX];	//踝軌道 x 原点移動
  double array_ankle_horizontal_z[ID_MAX];	//踝軌道 z 原点移動
  double array_hip_angle_horizontal[ID_MAX];	//股角度_sheet2(rad)
  double array_knee_angle_horizontal[ID_MAX];	//膝角度_sheet2(rad)
  double array_ankle_angle_horizontal[ID_MAX];	//踝角度_sheet2(rad)
  //sheet"3" 平行移動 :後足(hind legs)の動き x軸の動き、y軸はsheet"2"と同じ  重心移動後足
  double array_hip_horizontal_x1_sheet3[ID_MAX];	//股水平移動 原点移動
  double array_hip_horizontal_z1_sheet3[ID_MAX];	//股水平移動 原点移動 sheet"2"と同じ値
  double array_knee_horizontal_x_sheet3[ID_MAX];	//膝の座標
  double array_knee_horizontal_z_sheet3[ID_MAX];	//膝の座標
  double array_knee_horizontal_x1_sheet3[ID_MAX];	//膝の座標
  double array_knee_horizontal_z1_sheet3[ID_MAX];	//膝の座標
  double array_ankle_horizontal_x_sheet3[ID_MAX];	//踝軌道 x 原点移動
  double array_ankle_horizontal_z_sheet3[ID_MAX];	//踝軌道 z 原点移動
  double array_hip_angle_horizontal_sheet3[ID_MAX];	//股角度_sheet2(rad)
  double array_knee_angle_horizontal_sheet3[ID_MAX];	//膝角度_sheet2(rad)
  double array_ankle_angle_horizontal_sheet3[ID_MAX];	//踝角度_sheet2(rad)
  //sheet "4" 左右振れ幅 重心移動左右
  double array_hip_horizontal_y[ID_MAX];	//股水平移動
  double array_hip_angle_sheet4[ID_MAX];	//股角度sheet4 Y軸
  double array_ankle_angle_sheet4[ID_MAX];	//踝角度sheet4 Y軸

public:
  //sheet "5" 一連の計算結果をもとに、全身サーボの運行を記述する。
  double array_reg_hip[ID_MAX * WALKING_PROCESS];	//右足の運動記述
  double array_reg_knee[ID_MAX * WALKING_PROCESS];	//右足の運動記述
  double array_reg_ankle[ID_MAX * WALKING_PROCESS];	//右足の運動記述
  //sheet"4" 左右重心移動20240611
  double array_hip_up[ID_MAX* WALKING_PROCESS];	//股角度sheet4 Y軸
  double array_ankle_down[ID_MAX* WALKING_PROCESS];	//踝角度sheet4 Y軸

private:
//sheet "1"関数
  int rad_pi_divide2 (size_t ID, double *rad)	// 0からπ/2までのradを算出
  {
    double delta_rad = (M_PI / 2.0) / (ID - 1);
    for (int i = 0; i < ID; i++)
      {
	*rad = delta_rad * i;
	printf ("i=%d, *rad=%1.3f\n", i, *rad);
	rad++;
      }
    return 0;
  }

  int				//0から2πまでのradを算出
    rad_pi_multiply2 (size_t ID, double *rad)
  {
    double delta_rad = (M_PI * 2) / (ID - 1);
    for (int i = 0; i < ID; i++)
      {
	*rad = delta_rad * i;
	printf ("i=%d, *rad=%1.3f\n", i, *rad);
	rad++;
      }
    return 0;
  }

  int ankle_trajectory_x (size_t ID, double *rad, double *ankle_x,	//踵のX座標を算出 
			  double stride_length)
  {
    for (int i = 0; i < ID; i++)
      {
	*ankle_x = sin (*rad) * stride_length;
	printf ("rad=%1.4f, *ankle_x=%2.4f\n", *rad, sin (*rad));
	ankle_x++;
	rad++;
      }
    return 0;
  }

  int				//踵のz座標を算出
   
    ankle_trajectory_z (size_t ID, double *rad, double *ankle_z,
			double leg_up)
  {
    for (int i = 0; i < ID; i++)
      {
	*ankle_z = (sin (*rad - M_PI / 2) / 2 + 0.5) * leg_up;
	printf ("rad=%1.4f, *ankle_z=%2.4f\n", *rad, *ankle_z);
	ankle_z++;
	rad++;
      }
    return 0;
  }

  int				//股座標を算出
   
    hip_trajectory (size_t ID,
		    double *array_hip_trajectory_x,
		    double *array_hip_trajectory_z,
		    double initial_leg_bending_length,
		    double thigh_length, double shin_length,
		    double stride_length)
  {
    double x = stride_length / 2;
    double z = thigh_length + shin_length - initial_leg_bending_length;
    for (int i = 0; i < ID; i++)
      {
	*array_hip_trajectory_x = x;
	*array_hip_trajectory_z = z;
	printf ("i=%d,x=%2.4f, z=%2.4f\n", i,
		*array_hip_trajectory_x, *array_hip_trajectory_z);
	array_hip_trajectory_x++;
	array_hip_trajectory_z++;
      }
    return 0;
  }

  int				//股座標の原点移動
   
    origin_moving_hip_joint (size_t ID,
			     double *array_hip_trajectory_x,
			     double *array_hip_trajectory_z,
			     double *array_ankle_trajectory_x,
			     double *array_ankle_trajectory_z,
			     double *array_hip_trajectory_x1,
			     double *array_hip_trajectory_z1)
  {
    for (int i = 0; i < ID; i++)
      {
	*array_hip_trajectory_x1 =
	  *array_hip_trajectory_x - *array_ankle_trajectory_x;
	*array_hip_trajectory_z1 =
	  *array_hip_trajectory_z - *array_ankle_trajectory_z;
	//
	array_hip_trajectory_x++;
	array_hip_trajectory_z++;
	array_ankle_trajectory_x++;
	array_ankle_trajectory_z++;
	array_hip_trajectory_x1++;
	array_hip_trajectory_z1++;
      }
    return 0;
  }

  int				//膝座標の算出
   
    knee_trajectory (size_t ID, double *array_hip_trajectory_x1,
		     double *array_hip_trajectory_z1, double shin_length,
		     double thigh_length, double *array_knee_trajectory_x,
		     double *array_knee_trajectory_z)
  {
    double s1[ID];
    double s2[ID];
    for (int i = 0; i < ID; i++)
      {
	s1[i] =
	  pow (*array_hip_trajectory_x1, 2) + pow (*array_hip_trajectory_z1,
						   2);
	//
	s2[i] = s1[i] + pow (shin_length, 2) - pow (thigh_length, 2);
	//
	*array_knee_trajectory_x =
	  (*array_hip_trajectory_x1 * s2[i]) / (2 * s1[i]) +
	  (*array_hip_trajectory_z1 *
	   sqrt (4 * s1[i] * pow (shin_length, 2) -
		 pow (s2[i], 2))) / (2 * s1[i]);
	//
	*array_knee_trajectory_z =
	  sqrt (pow (shin_length, 2) - pow (*array_knee_trajectory_x, 2));
	// 
	printf ("i=%d,s1=%4.0f, s2=%4.0f\n", i, s1[i], s2[i]);
	array_hip_trajectory_x1++;
	array_hip_trajectory_z1++;
	array_knee_trajectory_x++;
	array_knee_trajectory_z++;
      }
    return 0;
  }

  int				//膝座標の原点移動
   
    origin_moving_knee_ankle_joint (size_t ID,
				    double *array_hip_trajectory_x1,
				    double *array_hip_trajectory_z1,
				    double *array_knee_trajectory_x,
				    double *array_knee_trajectory_z,
				    double *array_knee_trajectory_x1,
				    double *array_knee_trajectory_z1,
				    double *array_ankle_trajectory_x1,
				    double *array_ankle_trajectory_z1)
  {
    for (int i = 0; i < ID; i++)
      {
	*array_knee_trajectory_x1 =
	  *array_knee_trajectory_x - *array_hip_trajectory_x1;
	*array_knee_trajectory_z1 =
	  *array_knee_trajectory_z - *array_hip_trajectory_z1;
	*array_ankle_trajectory_x1 = *array_hip_trajectory_x1 * (-1);
	*array_ankle_trajectory_z1 = *array_hip_trajectory_z1 * (-1);
	//
	array_hip_trajectory_x1++;
	array_hip_trajectory_z1++;
	array_knee_trajectory_x++;
	array_knee_trajectory_z++;
	array_knee_trajectory_x1++;
	array_knee_trajectory_z1++;
	array_ankle_trajectory_x1++;
	array_ankle_trajectory_z1++;
      }
    return 0;
  }

  int				//股関節角度算出
   
    hip_angle (size_t ID,
	       double reference_angle_hip_x,
	       double reference_angle_hip_z,
	       double thigh_length,
	       double *array_knee_trajectory_x1,
	       double *array_knee_trajectory_z1, double *array_hip_angle)
  {
    double inner_product;
    double ab;
    for (int i = 0; i < ID; i++)
      {
	inner_product =
	  reference_angle_hip_x * (*array_knee_trajectory_x1) +
	  reference_angle_hip_z * (*array_knee_trajectory_z1);
	ab = inner_product / thigh_length;
	*array_hip_angle = acos (ab);
	//
	array_knee_trajectory_x1++;
	array_knee_trajectory_z1++;
	array_hip_angle++;
      }
    return 0;
  }

  int				//踝関節角度算出
   
    knee_angle (size_t ID,
		double *array_knee_trajectory_x1,
		double *array_knee_trajectory_z1,
		double thigh_length,
		double shin_length,
		double *array_ankle_trajectory_x1,
		double *array_ankle_trajectory_z1, double *array_knee_angle)
  {
    double inner_product;
    double ab;
    double knee_ankle_x;
    double knee_ankle_z;
    for (int i = 0; i < ID; i++)
      {
	knee_ankle_x = *array_ankle_trajectory_x1 - *array_knee_trajectory_x1;
	knee_ankle_z = *array_ankle_trajectory_z1 - *array_knee_trajectory_z1;
	inner_product =
	  *array_knee_trajectory_x1 * knee_ankle_x +
	  *array_knee_trajectory_z1 * knee_ankle_z;
	ab = inner_product / thigh_length / shin_length;
	*array_knee_angle = acos (ab);
	//
	array_knee_trajectory_x1++;
	array_knee_trajectory_z1++;
	array_ankle_trajectory_x1++;
	array_ankle_trajectory_z1++;
	array_knee_angle++;
      }
    return 0;
  }

  int				//踝関節角度
   
    ankle_angle (size_t ID,
		 double reference_angle_ankle_x,
		 double reference_angle_ankle_z,
		 double *array_knee_trajectory_x1,
		 double *array_knee_trajectory_z1,
		 double shin_length,
		 double *array_ankle_trajectory_x1,
		 double *array_ankle_trajectory_z1, double *array_ankle_angle)
  {
    double inner_product;
    double ab;
    double knee_ankle_x;
    double knee_ankle_z;
    for (int i = 0; i < ID; i++)
      {
	knee_ankle_x = *array_ankle_trajectory_x1 - *array_knee_trajectory_x1;
	knee_ankle_z = *array_ankle_trajectory_z1 - *array_knee_trajectory_z1;
	inner_product =
	  reference_angle_ankle_x * knee_ankle_x +
	  reference_angle_ankle_z * knee_ankle_z;
	ab = inner_product / shin_length;
	*array_ankle_angle = acos (ab);
	//
	array_knee_trajectory_x1++;
	array_knee_trajectory_z1++;
	array_ankle_trajectory_x1++;
	array_ankle_trajectory_z1++;
	array_ankle_angle++;
      }
    return 0;
  }

//sheet "2"
  int				//股x軸水平軌道
   
    hip_x_horizontal_trajectory (size_t ID, double *rad,
				 double *hip_x_horizontal,
				 double stride_length)
  {
    for (int i = 0; i < ID; i++)
      {
	*hip_x_horizontal =
	  (sin (*rad / 2 - M_PI / 2) + 1) / 2 * stride_length / 2;
	//printf ("rad=%1.4f, *hip_x_horizontal=%2.4f\n", *rad, *hip_x_horizontal);
	*hip_x_horizontal++;
	rad++;
      }
    return 0;
  }

  int				//股Z軸水平軌道
   
    hip_z_horizontal_trajectory (size_t ID, double *rad,
				 double *hip_z_horizontal,
				 double pallalel_vertical_motion)
  {
    for (int i = 0; i < ID; i++)
      {
	*hip_z_horizontal =
	  (sin (*rad - M_PI / 2) + 1) / 2 * pallalel_vertical_motion;
	printf ("rad=%1.4f, *hip_z_horizontal=%2.4f\n", *rad,
		*hip_z_horizontal);
	*hip_z_horizontal++;
	rad++;
      }
    return 0;
  }

  int				//股座標を算出
   
    hip_horizontal (size_t ID,
		    double *array_hip_horizontal_x,
		    double *array_hip_horizontal_z,
		    double initial_leg_bending_length,
		    double thigh_length, double shin_length,
		    double stride_length,
		    double *array_hip_horizontal_x1,
		    double *array_hip_horizontal_z1)
  {
    double z = thigh_length + shin_length - initial_leg_bending_length;
    for (int i = 0; i < ID; i++)
      {
	*array_hip_horizontal_x1 =
	  *array_hip_horizontal_x - stride_length / 2;
	*array_hip_horizontal_z1 = z + (*array_hip_horizontal_z);
	printf ("i=%d,x=%2.4f, z=%2.4f\n", i,
		*array_hip_horizontal_x1, *array_hip_horizontal_z1);
	array_hip_horizontal_x++;
	array_hip_horizontal_z++;
	array_hip_horizontal_x1++;
	array_hip_horizontal_z1++;
      }
    return 0;
  }

//sheet"3" 平行移動２ :後足の動き x軸の動きのみ計算、y軸はsheet"2"と同じ
  int				//股座標を算出
   
    hip_horizontal_sheet3 (size_t ID,
			   double *array_hip_horizontal_x,
			   double *array_hip_horizontal_z,
			   double initial_leg_bending_length,
			   double thigh_length, double shin_length,
			   double stride_length,
			   double *array_hip_horizontal_x1_sheet3,
			   double *array_hip_horizontal_z1_sheet3)
  {
    double z = thigh_length + shin_length - initial_leg_bending_length;
    for (int i = 0; i < ID; i++)
      {
	*array_hip_horizontal_x1_sheet3 = *array_hip_horizontal_x;
	*array_hip_horizontal_z1_sheet3 = z + (*array_hip_horizontal_z);
	printf ("i=%d,x=%2.4f, z=%2.4f\n", i,
		*array_hip_horizontal_x1_sheet3,
		*array_hip_horizontal_z1_sheet3);
	array_hip_horizontal_x++;
	array_hip_horizontal_z++;
	array_hip_horizontal_x1_sheet3++;
	array_hip_horizontal_z1_sheet3++;
      }
    return 0;
  }

//sheet "4" 左右振れ幅
  int hip_y_horizontal_trajectory (size_t ID,	//股y軸水平軌道
				   double *rad,
				   double *hip_y_horizontal,
				   double left_right_amplitude)
  {
    for (int i = 0; i < ID; i++)
      {
	*hip_y_horizontal =
	  (sin (*rad / 2 - M_PI / 2) + 1) / 2 * left_right_amplitude -
	  left_right_amplitude / 2;
	//printf ("rad=%1.4f, *hip_y_horizontal=%2.4f\n", *rad, *hip_y_horizontal);
	*hip_y_horizontal++;
	rad++;
      }
    return 0;
  }

  //股関節角度算出 y軸
  int hip_ankle_angle_sheet4 (size_t ID, double reference_angle_hip_y_sheet4, double reference_angle_hip_z_sheet4,	// 股関節基準角度
			      double thigh_length,	//太股長さ
			      double shin_length,	//すね長さ
			      double initial_leg_bending_length,	//初期足曲げ長さ
			      double ankle_hip_hight,	//踝xから踝y + 股xから股yまでの距離
            //計算結果
			      double *array_hip_horizontal_y,	//股水平移動
			      double *array_hip_angle_sheet4,
			      double *array_ankle_angle_sheet4)
  {
    double inner_product;
    double ab;
    double r =
      thigh_length + shin_length - initial_leg_bending_length +
      ankle_hip_hight;
    double hip_z = r;
    for (int i = 0; i < ID; i++)
      {
	inner_product =
	  reference_angle_hip_y_sheet4 * (*array_hip_horizontal_y) +
	  reference_angle_hip_z_sheet4 * hip_z;
	ab = inner_product / r;
	*array_hip_angle_sheet4 = acos (ab) - M_PI / 2;
	*array_ankle_angle_sheet4 = -1.0 * *array_hip_angle_sheet4;
	//
	array_hip_horizontal_y++;
	array_hip_angle_sheet4++;
	array_ankle_angle_sheet4++;
      }
    return 0;
  }

//sheet "5" 一連の計算結果をもとに、右足のサーボを記述する。
  int				//右足の動き
    right_reg_motion (size_t ID, int walking_process,
		      //sheet "1" trajectory 軌道 一歩前
		      double *array_hip_angle_trajectory,	//股角度(rad)
		      double *array_knee_angle_trajectory,	//膝角度(rad)
		      double *array_ankle_angle_trajectory,	//踝角度(rad)
		      //sheet "2" horizontal 水平:前足の動き 重心移動前足
		      double *array_hip_angle_horizontal,	//股角度_sheet2(rad)
		      double *array_knee_angle_horizontal,	//膝角度_sheet2(rad)
		      double *array_ankle_angle_horizontal,	//踝角度_sheet2(rad)
		      //sheet"3" 平行移動 :後足(hind legs)の動き x軸の動き、y軸はsheet"2"と同じ  重心移動後足
		      double *array_hip_angle_horizontal_sheet3,	//股角度_sheet2(rad)
		      double *array_knee_angle_horizontal_sheet3,	//膝角度_sheet2(rad)
		      double *array_ankle_angle_horizontal_sheet3,	//踝角度_sheet2(rad)
          //sheet"4" 左右の体重移動 20240611
          double *array_hip_angle_sheet4,	//股角度sheet4 Y軸
          double *array_ankle_angle_sheet4,	//踝角度sheet4 Y軸
          //計算結果を格納する変数
		      double *array_reg_hip,
		      double *array_reg_knee, 
          double *array_reg_ankle,
          double *array_hip_up,	//股角度sheet4 Y軸 20240611
          double *array_ankle_down//,	//踝角度sheet4 Y軸
          )
  {
    for (int ii = 0; ii < walking_process; ii++)
      {
	for (int i = 0; i < ID; i++)
	  {
	    switch (ii)
	      {
	      case 0:
		*array_reg_hip = *array_hip_angle_trajectory;
		*array_reg_knee = *array_knee_angle_trajectory;
		*array_reg_ankle = *array_ankle_angle_trajectory;
    *array_hip_up = array_hip_angle_sheet4[0];
    *array_ankle_down = *array_hip_up * -1;
		array_hip_angle_trajectory++;
		array_knee_angle_trajectory++;
		array_ankle_angle_trajectory++;
		break;
	      case 1:
		*array_reg_hip = *array_hip_angle_horizontal;
		*array_reg_knee = *array_knee_angle_horizontal;
		*array_reg_ankle = *array_ankle_angle_horizontal;
    *array_hip_up = array_hip_angle_sheet4[i];
    *array_ankle_down = *array_hip_up * -1;
		array_hip_angle_horizontal++;
		array_knee_angle_horizontal++;
		array_ankle_angle_horizontal++;
		break;
	      case 2:
		*array_reg_hip = *(array_reg_hip - 1);
		*array_reg_knee = *(array_reg_knee - 1);
		*array_reg_ankle = *(array_reg_ankle - 1);
    *array_hip_up = array_hip_angle_sheet4[ID - 1];
    *array_ankle_down = *array_hip_up * -1;
		break;
	      case 3:
		*array_reg_hip = *array_hip_angle_horizontal_sheet3;
		*array_reg_knee = *array_knee_angle_horizontal_sheet3;
		*array_reg_ankle = *array_ankle_angle_horizontal_sheet3;
    *array_hip_up = array_ankle_angle_sheet4[i];
    *array_ankle_down = *array_hip_up * -1;
		array_hip_angle_horizontal_sheet3++;
		array_knee_angle_horizontal_sheet3++;
		array_ankle_angle_horizontal_sheet3++;
		break;
	      default:
		printf ("right_reg_motionでErrorですな。\n");
		break;
	      }
	    array_reg_hip++;
	    array_reg_knee++;
	    array_reg_ankle++;
      array_hip_up++ ;
      array_ankle_down++ ; 
	  }
      }
    return 0;
  }

  //左足のために位相をずらす。
  int PhaseShift (size_t ID, int walking_process,
		  double *array_reg_hip,
		  double *array_reg_knee, double *array_reg_ankle)
  {
    int i = 0;
    double array_phase_reg_hip[ID * walking_process];
    double array_phase_reg_knee[ID * walking_process];
    double array_phase_reg_ankle[ID * walking_process];
    for (i = 0; i < ID * walking_process / 2; i++)
      {
	array_phase_reg_hip[i] =
	  *(array_reg_hip + i + ID * walking_process / 2);
	array_phase_reg_knee[i] =
	  *(array_reg_knee + i + ID * walking_process / 2);
	array_phase_reg_ankle[i] =
	  *(array_reg_ankle + i + ID * walking_process / 2);
      }
    for (i = ID * walking_process / 2; i < ID * walking_process; i++)
      {
	array_phase_reg_hip[i] =
	  *(array_reg_hip + i - ID * walking_process / 2);
	array_phase_reg_knee[i] =
	  *(array_reg_knee + i - ID * walking_process / 2);
	array_phase_reg_ankle[i] =
	  *(array_reg_ankle + i - ID * walking_process / 2);
      }
    for (i = 0; i < ID * walking_process; i++)
      {
	*(array_reg_hip + i) = array_phase_reg_hip[i];
	*(array_reg_knee + i) = array_phase_reg_knee[i];
	*(array_reg_ankle + i) = array_phase_reg_ankle[i];
      }
    return 0;
  }

public:
  //位相の移動公開 
  int PhaseShift_pub ()
  {
    PhaseShift (ID_MAX, WALKING_PROCESS,
		array_reg_hip, array_reg_knee, array_reg_ankle);
    return 0;
  }

  //コンストラクタ
  LegMotion_xz ()
  {
    //関数実行
    //sheet "1" 軌道計算
    rad_pi_divide2 (ID_MAX, array_rad_pi_divide2);
    ankle_trajectory_x (ID_MAX, array_rad_pi_divide2,
			array_ankle_trajectory_x, STRIDE_LENGTH);
    rad_pi_multiply2 (ID_MAX, array_rad_pi_multiply2);
    ankle_trajectory_z (ID_MAX, array_rad_pi_multiply2,
			array_ankle_trajectory_z, LEG_UP);
    hip_trajectory (ID_MAX, array_hip_trajectory_x,
		    array_hip_trajectory_z, INITIAL_LEG_BENDING_LENGTH,
		    THIGH_LENGTH, SHIN_LENGTH, STRIDE_LENGTH);
    origin_moving_hip_joint (ID_MAX, array_hip_trajectory_x,
			     array_hip_trajectory_z,
			     array_ankle_trajectory_x,
			     array_ankle_trajectory_z,
			     array_hip_trajectory_x1,
			     array_hip_trajectory_z1);
    knee_trajectory (ID_MAX, array_hip_trajectory_x1, array_hip_trajectory_z1,
		     SHIN_LENGTH, THIGH_LENGTH, array_knee_trajectory_x,
		     array_knee_trajectory_z);
    origin_moving_knee_ankle_joint (ID_MAX, array_hip_trajectory_x1,
				    array_hip_trajectory_z1,
				    array_knee_trajectory_x,
				    array_knee_trajectory_z,
				    array_knee_trajectory_x1,
				    array_knee_trajectory_z1,
				    array_ankle_trajectory_x1,
				    array_ankle_trajectory_z1);
    hip_angle (ID_MAX, REFERENCE_ANGLE_HIP_X, REFERENCE_ANGLE_HIP_Z,
	       THIGH_LENGTH, array_knee_trajectory_x1,
	       array_knee_trajectory_z1, array_hip_angle_trajectory);
    knee_angle (ID_MAX, array_knee_trajectory_x1, array_knee_trajectory_z1,
		THIGH_LENGTH, SHIN_LENGTH, array_ankle_trajectory_x1,
		array_ankle_trajectory_z1, array_knee_angle_trajectory);
    ankle_angle (ID_MAX, REFERENCE_ANGLE_ANKLE_X, REFERENCE_ANGLE_ANKLE_Z,
		 array_knee_trajectory_x1, array_knee_trajectory_z1,
		 SHIN_LENGTH, array_ankle_trajectory_x1,
		 array_ankle_trajectory_z1, array_ankle_angle_trajectory);
//sheet "2" 水平移動計算 ：前足の動き
    hip_x_horizontal_trajectory (ID_MAX, array_rad_pi_multiply2,
				 array_hip_horizontal_x, STRIDE_LENGTH);
    hip_z_horizontal_trajectory (ID_MAX, array_rad_pi_multiply2,
				 array_hip_horizontal_z,
				 PALLALEL_VERTICAL_MOTION);
    hip_horizontal (ID_MAX,
		    array_hip_horizontal_x,
		    array_hip_horizontal_z,
		    INITIAL_LEG_BENDING_LENGTH,
		    THIGH_LENGTH, SHIN_LENGTH,
		    STRIDE_LENGTH,
		    array_hip_horizontal_x1, array_hip_horizontal_z1);
    knee_trajectory (ID_MAX,
		     array_hip_horizontal_x1,
		     array_hip_horizontal_z1,
		     SHIN_LENGTH,
		     THIGH_LENGTH,
		     array_knee_horizontal_x, array_knee_horizontal_z);
    origin_moving_knee_ankle_joint (ID_MAX,
				    array_hip_horizontal_x1,
				    array_hip_horizontal_z1,
				    array_knee_horizontal_x,
				    array_knee_horizontal_z,
				    array_knee_horizontal_x1,
				    array_knee_horizontal_z1,
				    array_ankle_horizontal_x,
				    array_ankle_horizontal_z);
    hip_angle (ID_MAX,
	       REFERENCE_ANGLE_HIP_X,
	       REFERENCE_ANGLE_HIP_Z,
	       THIGH_LENGTH,
	       array_knee_horizontal_x1,
	       array_knee_horizontal_z1, array_hip_angle_horizontal);
    knee_angle (ID_MAX,
		array_knee_horizontal_x1,
		array_knee_horizontal_z1,
		THIGH_LENGTH,
		SHIN_LENGTH,
		array_ankle_horizontal_x,
		array_ankle_horizontal_z, array_knee_angle_horizontal);
    ankle_angle (ID_MAX,
		 REFERENCE_ANGLE_ANKLE_X,
		 REFERENCE_ANGLE_ANKLE_Z,
		 array_knee_horizontal_x1,
		 array_knee_horizontal_z1,
		 SHIN_LENGTH,
		 array_ankle_horizontal_x,
		 array_ankle_horizontal_z, array_ankle_angle_horizontal);
//sheet"3" 平行移動２ :後足の動き x軸の動きのみ計算、y軸はsheet"2"と同じ
    hip_horizontal_sheet3 (ID_MAX,
			   array_hip_horizontal_x,
			   array_hip_horizontal_z,
			   INITIAL_LEG_BENDING_LENGTH,
			   THIGH_LENGTH, SHIN_LENGTH,
			   STRIDE_LENGTH,
			   array_hip_horizontal_x1_sheet3,
			   array_hip_horizontal_z1_sheet3);
    knee_trajectory (ID_MAX, array_hip_horizontal_x1_sheet3,
		     array_hip_horizontal_z1_sheet3, SHIN_LENGTH,
		     THIGH_LENGTH, array_knee_horizontal_x_sheet3,
		     array_knee_horizontal_z_sheet3);
    origin_moving_knee_ankle_joint (ID_MAX, array_hip_horizontal_x1_sheet3,
				    array_hip_horizontal_z1_sheet3,
				    array_knee_horizontal_x_sheet3,
				    array_knee_horizontal_z_sheet3,
				    array_knee_horizontal_x1_sheet3,
				    array_knee_horizontal_z1_sheet3,
				    array_ankle_horizontal_x_sheet3,
				    array_ankle_horizontal_z_sheet3);
    hip_angle (ID_MAX, REFERENCE_ANGLE_HIP_X, REFERENCE_ANGLE_HIP_Z,
	       THIGH_LENGTH, array_knee_horizontal_x1_sheet3,
	       array_knee_horizontal_z1_sheet3,
	       array_hip_angle_horizontal_sheet3);
    knee_angle (ID_MAX, array_knee_horizontal_x1_sheet3,
		array_knee_horizontal_z1_sheet3, THIGH_LENGTH, SHIN_LENGTH,
		array_ankle_horizontal_x_sheet3,
		array_ankle_horizontal_z_sheet3,
		array_knee_angle_horizontal_sheet3);
    ankle_angle (ID_MAX, REFERENCE_ANGLE_ANKLE_X, REFERENCE_ANGLE_ANKLE_Z,
		 array_knee_horizontal_x1_sheet3,
		 array_knee_horizontal_z1_sheet3, SHIN_LENGTH,
		 array_ankle_horizontal_x_sheet3,
		 array_ankle_horizontal_z_sheet3,
		 array_ankle_angle_horizontal_sheet3);
//sheet "4" 左右振れ幅
    hip_y_horizontal_trajectory (ID_MAX, array_rad_pi_multiply2,
				 array_hip_horizontal_y,
				 LEFT_RIGHT_AMPLITUDE);
    hip_ankle_angle_sheet4 (ID_MAX, REFERENCE_ANGLE_HIP_Y_SHEET4, REFERENCE_ANGLE_HIP_Z_SHEET4,	// 股関節基準角度
			    THIGH_LENGTH,	//太股長さ
			    SHIN_LENGTH,	//すね長さ
			    INITIAL_LEG_BENDING_LENGTH,	//初期足曲げ長さ
			    ANKLE_HIP_HIGHT,	//踝xから踝y + 股xから股yまでの距離
			    array_hip_horizontal_y,	//股水平移動
			    array_hip_angle_sheet4, 
          array_ankle_angle_sheet4);	//踝角度sheet4
//sheet "5" 変数統合 
    right_reg_motion (ID_MAX, WALKING_PROCESS,
		      //sheet "1" trajectory 軌道 一歩前
		      array_hip_angle_trajectory,	//股角度(rad)
		      array_knee_angle_trajectory,	//膝角度(rad)
		      array_ankle_angle_trajectory,	//踝角度(rad)
		      //sheet "2" horizontal 水平:前足の動き 重心移動前足
		      array_hip_angle_horizontal,	//股角度_sheet2(rad)
		      array_knee_angle_horizontal,	//膝角度_sheet2(rad)
		      array_ankle_angle_horizontal,	//踝角度_sheet2(rad)
		      //sheet"3" 平行移動 :後足(hind legs)の動き x軸の動き、y軸はsheet"2"と同じ  重心移動後足
		      array_hip_angle_horizontal_sheet3,	//股角度_sheet2(rad)
		      array_knee_angle_horizontal_sheet3,	//膝角度_sheet2(rad)
		      array_ankle_angle_horizontal_sheet3,	//踝角度_sheet2(rad)
          //sheet"4" 左右の体重移動 20240611
          array_hip_angle_sheet4,	//股角度sheet4 Y軸
          array_ankle_angle_sheet4,	//踝角度sheet4 Y軸
          //計算結果
		      array_reg_hip,
          array_reg_knee,
          array_reg_ankle,
          array_hip_up,	//股角度sheet4 Y軸 20240611
          array_ankle_down//,	//踝角度sheet4 Y軸
          );
  }
};

//共通関数 グラフ作成
double				//配列変数内の最大値を選択
max_element (const double *array, size_t size)
{
  assert (array != NULL);
  assert (size >= 1);
  double max = array[0];
  for (size_t i = 1; i < size; ++i)
    {
      if (max < array[i])
	{
	  max = array[i];
	}
    }
  return max;
}

double				//配列変数内の最小値を選択
min_element (const double *array, size_t size)
{
  assert (array != NULL);
  assert (size >= 1);
  double min = array[0];
  for (size_t i = 1; i < size; ++i)
    {
      if (min > array[i])
	{
	  min = array[i];
	}
    }
  return min;
}


int				//配列変数の値をx,yでグラフ化
gnuplot (size_t ID, double *x, double *y)
{
  FILE *gp;
  //---- グラフ作成 ---- 
  gp = popen ("gnuplot -persist", "w");
  fprintf (gp, "plot '-' with points pt 7 title \"line\"\n");
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%f\t%f\n", *x, *y);	// データの書き込み
      x++;
      y++;
    }
  fprintf (gp, "e\n");
  pclose (gp);
  return 0;
}

int				//配列変数の値をIDの順番にプロット
gnuplot1 (size_t ID, double *x1)
{
  FILE *gp;
  //---- グラフ作成 ---- 
  gp = popen ("gnuplot -persist", "w");
  fprintf (gp, "plot '-' with points pt 7 title \"line\"\n");
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%d\t%f\n", i, *x1);	// gnumeric 補正係数： * 180 / M_PI -90
      x1++;
    }
  fprintf (gp, "e\n");
  pclose (gp);
  return 0;
}

int				//配列変数の値をIDの順番にプロット
gnuplot1_1 (size_t ID, double *x1, double *x2)
{
  FILE *gp;
  //---- グラフ作成 ---- 
  gp = popen ("gnuplot -persist", "w");
  fprintf (gp, "plot '-' with points pt 7 title \"line\"\n");
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%d\t%f\n", i, *x1);	// gnumeric 補正係数： * 180 / M_PI -90
      x1++;
    }
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%d\t%f\n", i, *x2);	// gnumeric 補正係数： * 180 / M_PI -90
      x2++;
    }
  fprintf (gp, "e\n");
  pclose (gp);
  return 0;
}

int				//配列変数の値をIDの順番にプロット
gnuplot1_2 (size_t ID, double *x1, double *x2, double *x3)
{
  FILE *gp;
  //---- グラフ作成 ---- 
  gp = popen ("gnuplot -persist", "w");
  fprintf (gp, "plot '-' with points pt 7 title \"line\"\n");
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%d\t%f\n", i, *x1);	// gnumeric 補正係数： * 180 / M_PI -90
      x1++;
    }
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%d\t%f\n", i, *x2);	// gnumeric 補正係数： * 180 / M_PI -90
      x2++;
    }
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%d\t%f\n", i, *x3);	// gnumeric 補正係数： * 180 / M_PI -90
      x3++;
    }
  fprintf (gp, "e\n");
  pclose (gp);
  return 0;
}

int				//配列変数x1,y1,x2,y2をプロット
gnuplot2 (size_t ID, double *x1, double *y1, double *x2, double *y2)
{
  FILE *gp;
  //---- グラフ作成 ---- 
  gp = popen ("gnuplot -persist", "w");
  fprintf (gp, "plot '-' with points pt 7 title \"line\"\n");
  fprintf (gp, "%f\t%f\n", 0.0, 0.0);	// データの書き込み
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%f\t%f\n", *x1, *y1);	// データの書き込み
      x1++;
      y1++;
    }
  for (int i = 0; i < ID; i++)
    {
      fprintf (gp, "%f\t%f\n", *x2, *y2);	// データの書き込み
      x2++;
      y2++;
    }
  fprintf (gp, "e\n");
  pclose (gp);
  return 0;
}

int
main (int argc, char **argv)
{
  //double array_reg_up[ID_MAX * WALKING_PROCESS];      //右足の運動記述
  //double array_reg_down[ID_MAX * WALKING_PROCESS];    //右足の運動記述
  //クラスの定義
  LegMotion_xz right_leg;
  LegMotion_xz left_leg;
  left_leg.PhaseShift_pub ();
  //グラフ化
  //gnuplot1_2 (ID_MAX * WALKING_PROCESS,	//右足
	//      right_leg.array_reg_hip,
	//      right_leg.array_reg_knee, 
  //      right_leg.array_reg_ankle);
  //gnuplot1_2 (ID_MAX * WALKING_PROCESS,	//左足
	//      left_leg.array_reg_hip,
	//      left_leg.array_reg_knee, 
  //      left_leg.array_reg_ankle);
  //gnuplot1_1 (ID_MAX* WALKING_PROCESS,	//左右の移動 20240611
	//      right_leg.array_hip_up,
  //      right_leg.array_ankle_down);
  ros::init (argc, argv, "rvis_joint_publisher");
  ros::NodeHandle nh;
  ros::Publisher joint_pub =
    nh.advertise < sensor_msgs::JointState > ("joint_states", 10);
  ros::Rate loop_rate (10);

  sensor_msgs::JointState js0;
  js0.name.resize (16);
  //右足
  js0.name[3] = "body2_joint";
  js0.name[4] = "body3_joint";
  js0.name[5] = "body4_joint";
  js0.name[6] = "body5_joint";
  js0.name[7] = "body6_joint";
  //左足
  js0.name[12] = "body7_joint";
  js0.name[11] = "body8_joint";
  js0.name[10] = "body9_joint";
  js0.name[9] = "body10_joint";
  js0.name[8] = "body11_joint";
  //右腕
  js0.name[2] = "body12_joint";
  js0.name[1] = "body13_joint";
  js0.name[0] = "body14_joint";
  //左腕
  js0.name[13] = "body15_joint";
  js0.name[14] = "body16_joint";
  js0.name[15] = "body17_joint";
  js0.position.resize (16);

  int i;
  for (i = 0; i < 16; i++)
    {
      count[i] = 0;
    }
  i=0;
  while (ros::ok ())
    {
      js0.header.stamp = ros::Time::now ();
      //変数ループ
      if ( i >= ID_MAX * WALKING_PROCESS ) i=0 ;
      //変数設定
      count[4]  = right_leg.array_reg_hip[i];
      count[5]  = right_leg.array_reg_knee[i];
      count[6]  = right_leg.array_reg_ankle[i];
      count[11] = left_leg.array_reg_hip[i];
      count[10] = left_leg.array_reg_knee[i];
      count[9]  = left_leg.array_reg_ankle[i];
      //左右体重移動20240613
      count[3] = right_leg.array_hip_up[i]; 
      count[7] = right_leg.array_ankle_down[i]; 
      count[8] = left_leg.array_hip_up[i]; 
      count[12] = left_leg.array_ankle_down[i]; 
      //右腕
      js0.position[0] = (float) count[0]   ;	//肩
      js0.position[1] = (float) count[1]   ;	//肩
      js0.position[2] = (float) count[2]   ;	//肘
      //右足の上から
      js0.position[3] = (float) count[3]   ;	//上
      js0.position[4] = (float) count[4]   ;	//股
      js0.position[5] = (float) count[5]   ;	//膝
      js0.position[6] = (float) count[6]   ;	//踝
      js0.position[7] = (float) count[7]   ;	//下
      //左足の下から
      js0.position[8] = (float) count[8]   ;	//下
      js0.position[9] = (float) count[9]   ;	//踝
      js0.position[10] = (float) count[10] ;	//膝
      js0.position[11] = (float) count[11] ;	//股
      js0.position[12] = (float) count[12] ;	//上
      //左腕
      js0.position[13] = (float) count[13] ;	//肩
      js0.position[14] = (float) count[14] ;	//肩
      js0.position[15] = (float) count[15] ;	//肘
      joint_pub.publish (js0);
      //count++;
      //ros::spinOnce ();
      loop_rate.sleep ();
      i++;
    }
  //  return 0;
}
