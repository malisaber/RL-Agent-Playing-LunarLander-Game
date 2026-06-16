#pragma once


//////////////////////////////////////////////////////////////////////////
//						The Neural Network								//
//////////////////////////////////////////////////////////////////////////
#define STATE_PRECISION_POSITION						0.05f
#define STATE_PRECISION_SPEED							0.1f
#define STATE_PRECISION_ANGLE							0.01f








//////////////////////////////////////////////////////////////////////////
//							Physics										//
//////////////////////////////////////////////////////////////////////////
//	Reward Function
#define Margin											0.1f
#define Max_Distance_Allowed							Window_Width / SCALE / 2.0f * 1.4f
#define Minimum_Respawn_Attitude						0.0f
#define Check_Point_Distances							0.125f
#define Reward_Val_4_Change_Distance					0.78125f
#define Reward_Val_4_Make_Successful_Contact			250.0f
#define Reward_Pen_4_Make_UnSuccessful_Contact			250.0f
#define Reward_Pen_4_Out_of_Scope						250.0f
//#define Reward_Pen_4_Below_Mountain_Slope				5.0f
#define Reward_Val_4_Fule_Left							25.0f
#define Reward_Pen_4_Off_center							50.0f
#define Reward_Pen_4_Off_Balance						50.0f
#define Reward_Pen_4_Lin_Velocity						50.0f
#define Reward_Pen_4_Ang_Velocity						50.0f
#define Reward_Pen_4_Base_Actions						0.078125f
//	Window 
static constexpr float									PIXELS_PER_METER = 50.0f;
#define Window_Width									1200
#define Window_Height									900
#define Window_Max_FPS									120
#define Simulation_TimeStep								1.0f/Window_Max_FPS
#define SCALE											PIXELS_PER_METER
#define Window_Midle_X									Window_Width / SCALE / 2
//	Colors
#define Color_Ground									sf::Color::Green
#define Color_Box1										sf::Color::Red
#define Color_Box2										sf::Color::Blue
#define Color_Mountain									sf::Color(139, 69,  19)
#define Color_LunarLander								sf::Color(100, 100, 100)
#define Color_Flag_Pole									sf::Color::Cyan
#define Color_Flag_Cloth								sf::Color::Green
#define Color_Window_Background							sf::Color(20, 20, 30)
//	World
#define Physic_solver_iteration							20
#define Physic_max_allowed_penetration					0.0001f
#define Physic_Ground_Restitution						1.0f
#define Physic_Boxes_Restitution						0.3f
#define Physic_Circles_Restitution						0.3f
#define Physic_Mountain_Restitution						0.3f
#define Physic_LunarLander_Restitution					0.35f
//	Gravity
#define Gravity_X										0.0f
#define Gravity_Y										-2.0f
#define Gravity_Z										0.0f
//	Ground
#define Ground_Pos_X									Window_Width/SCALE
#define Ground_Pos_Y									1.0f
#define Ground_Pos_Z									0.0f
#define Ground_Half_Width								Window_Width/SCALE/2
#define Ground_Half_Height								0.5f
#define Ground_Half_Depth								0
//	a Box
#define Body_Half_Width									0.5f
#define Body_Half_Height								0.5f
#define Body_Half_Depth									0.5f
#define Body_Pos_X										13.0f
#define Body_Pos_Y										10.0f
#define Body_Pos_Z										0.0f
#define Body_mass										2.0f
//	an Arm
#define Arm_Half_Width									1.5f
#define Arm_Half_Height									0.1f
#define Arm_Half_Depth									0.5f
#define Arm_Pos_X										13.0f
#define Arm_Pos_Y										15.0f
#define Arm_Pos_Z										0.0f
#define Arm_mass										2.0f
//	a Circle
#define Circle_Radius									0.5f
#define Circle_Pos_X									15.0f
#define Circle_Pos_Y									10.0f
#define Circle_Pos_Z									0.0f
#define Circle_mass										2.0f
//	The Platform
#define Platform_Length									4.0f
#define Platform_X										Window_Width/SCALE/2-Platform_Length/2
#define Platform_Y										4.0f
#define Platform_center_X								Window_Width/SCALE/2
#define Platform_flag_offcenter							Platform_Length*3/8
//	The Platform's Flag
#define Flag_Pole_Half_Width							0.01f
#define Flag_Pole_Half_Height							0.25f
#define Flag_Cloth_Width								0.5f
#define Flag_Cloth_Half_Height							0.1f
//	The Mountain
#define Mount_Max_DeltaX								1.2f
#define Mount_Max_DeltaY								0.5f
#define Mount_Ave_Height								Platform_Y
#define Mount_length									Window_Width/SCALE
#define Mount_Restitution								0.1f
#define Mount_max_slope									0.4f
//	The Lunar Lander
#define LL_Unit											1.0f
#define LL_Depth										0.5f
#define LL_Body_Half_Size								0.5f
#define LL_Arm_Half_Width								0.4f
#define LL_Arm_Half_Height								0.1f
#define LL_Leg_Half_Width								0.1f
#define LL_Leg_half_Height								0.2f
#define LL_Eng_Half_Length								0.2f
#define LL_mass											1.0f
#define LL_Pos_X										Window_Width/SCALE/2-Platform_Length/2+1
#define LL_Pos_Y										9.0f
#define LL_Pos_Z										0.0f
#define LL_Engine_thrust								1.8f
//	Vapor 
#define Vapor_radius									0.05f
#define Vapor_radius_expantion_rate						0.01f
#define Vapor_mass										0.01f
#define Vapor_life_time_base							((1.0f + (LL_Unit-1.0f)/4.0f) * 0.125f  )
#define Vapor_life_time_vary							((1.0f + (LL_Unit-1.0f)/4.0f) * 0.06125f)
#define Vapor_life_time_speed_coef						0.2f
#define Vapor_thrust_base								((1.0f + (LL_Unit-1.0f)/4.0f) * 5.0f)
#define Vapor_thrust_vary								((1.0f + (LL_Unit-1.0f)/4.0f) * 2.5f)
#define Vapor_thrust_speed_coef							0.5f
#define Vapor_ejection_speed_base						0.125f
#define Vapor_angular_spread_range						0.1f
#define Vapor_cnt										2
#define Vapor_Decay										10
#define Vapor_cnt_speed_coef							1
#define Vapor_step_len									0.3f
#define Vapor_color										sf::Color(255, 255, 255, 180)
//	The LunarBody
/////	the body 
#define LL_Blt_Origin_Right_Arm_X						-LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Arm_Half_Width
#define LL_Blt_Origin_Right_Arm_Y						+LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Arm_Half_Height
#define LL_Blt_Origin_Left_Arm_X						+LL_Unit	*	LL_Body_Half_Size	+	LL_Unit	*	LL_Arm_Half_Width
#define LL_Blt_Origin_Left_Arm_Y						+LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Arm_Half_Height
#define LL_Blt_Origin_Right_Leg_X						-LL_Unit	*	LL_Body_Half_Size	+	LL_Unit	*	LL_Leg_Half_Width
#define LL_Blt_Origin_Right_Leg_Y						-LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Leg_half_Height
#define LL_Blt_Origin_Left_Leg_X						+LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Leg_Half_Width
#define LL_Blt_Origin_Left_Leg_Y						-LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Leg_half_Height
#define LL_Blt_Origin_Right_Eng_X						-LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Arm_Half_Width	*	2	 +LL_Unit	*	LL_Eng_Half_Length
#define LL_Blt_Origin_Right_Eng_Y						+LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Arm_Half_Height	*	2	 -LL_Unit	*	LL_Eng_Half_Length - LL_Unit * LL_Eng_Half_Length / 3
#define LL_Blt_Origin_Left_Eng_X						+LL_Unit	*	LL_Body_Half_Size	+	LL_Unit	*	LL_Arm_Half_Width	*	2	 -LL_Unit	*	LL_Eng_Half_Length
#define LL_Blt_Origin_Left_Eng_Y						+LL_Unit	*	LL_Body_Half_Size	-	LL_Unit	*	LL_Arm_Half_Height	*	2	 -LL_Unit	*	LL_Eng_Half_Length - LL_Unit * LL_Eng_Half_Length / 3
/////	the body Origin in Bullet
#define LL_Blt_Origin_body								{   0,											0,										0}
#define LL_Blt_Origin_Right_Arm							{   LL_Blt_Origin_Right_Arm_X,					LL_Blt_Origin_Right_Arm_Y,				0}
#define LL_Blt_Origin_Left_Arm							{   LL_Blt_Origin_Left_Arm_X,					LL_Blt_Origin_Left_Arm_Y,				0}
#define LL_Blt_Origin_Right_Leg							{   LL_Blt_Origin_Right_Leg_X,					LL_Blt_Origin_Right_Leg_Y,				0}
#define LL_Blt_Origin_Left_Leg							{   LL_Blt_Origin_Left_Leg_X,					LL_Blt_Origin_Left_Leg_Y,				0}
#define LL_Blt_Origin_Right_Eng							{   LL_Blt_Origin_Right_Eng_X,					LL_Blt_Origin_Right_Eng_Y,				0}
#define LL_Blt_Origin_Left_Eng							{   LL_Blt_Origin_Left_Eng_X,					LL_Blt_Origin_Left_Eng_Y,				0}
/////	the body Origin in SFML
#define LL_SF_Origin_body								{	LL_Unit * LL_Body_Half_Size  * SCALE,		LL_Unit * LL_Body_Half_Size  * SCALE }
#define LL_SF_Origin_Right_Arm							{	LL_Unit * LL_Arm_Half_Width  * SCALE,		LL_Unit * LL_Arm_Half_Height * SCALE }
#define LL_SF_Origin_Left_Arm							{	LL_Unit * LL_Arm_Half_Width  * SCALE,		LL_Unit * LL_Arm_Half_Height * SCALE }
#define LL_SF_Origin_Right_Leg							{	LL_Unit * LL_Leg_Half_Width  * SCALE,		LL_Unit * LL_Leg_half_Height * SCALE }
#define LL_SF_Origin_Left_Leg							{	LL_Unit * LL_Leg_Half_Width  * SCALE,		LL_Unit * LL_Leg_half_Height * SCALE }
#define LL_SF_Origin_Right_Eng							{	0,											0 }
#define LL_SF_Origin_Left_Eng							{	0,											0 }
/////	the body Offset in SFML
#define LL_SF_offset_body								{   0,											0}
#define LL_SF_offset_Right_Arm							{   (LL_Blt_Origin_Right_Arm_X)	*	SCALE,		(LL_Blt_Origin_Right_Arm_Y)	*	SCALE	}
#define LL_SF_offset_Left_Arm							{   (LL_Blt_Origin_Left_Arm_X)	*	SCALE,		(LL_Blt_Origin_Left_Arm_Y)	*	SCALE	}
#define LL_SF_offset_Right_Leg							{   (LL_Blt_Origin_Right_Leg_X)	*	SCALE,		(LL_Blt_Origin_Right_Leg_Y)	*	SCALE	}
#define LL_SF_offset_Left_Leg							{   (LL_Blt_Origin_Left_Leg_X)	*	SCALE,		(LL_Blt_Origin_Left_Leg_Y)	*	SCALE	}
#define LL_SF_offset_Right_Eng							{   (LL_Blt_Origin_Right_Eng_X)	*	SCALE,		(LL_Blt_Origin_Right_Eng_Y)	*	SCALE	}
#define LL_SF_offset_Left_Eng							{   (LL_Blt_Origin_Left_Eng_X)	*	SCALE,		(LL_Blt_Origin_Left_Eng_Y)	*	SCALE	}





