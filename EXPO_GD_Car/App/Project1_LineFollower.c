// https://blog.csdn.net/m0_53966219/article/details/126711218
// https://blog.csdn.net/zhuoqingjoking97298/article/details/120093315
// PID: https://zhuanlan.zhihu.com/p/586532545?utm_id=0
// https://blog.csdn.net/weixin_42208428/article/details/122173575
// https://blog.csdn.net/weixin_43964993/article/details/112383192

#include "Project1.h"

extern image2_t image2_use;			// use 1/3 of the original image (40 continuous lines in the middle)
extern image2_t image2_show;		// show 1/3 of the full size screen
extern image2_t image2_temp;		// show 1/3 of the full size screen
extern uint8_t image_ready;			// MT9V034 new data frame ready in buffer
extern uint8_t image_size;				// 0: full size; 1: half size; 2: 1/3 sized
extern int16_t encoder_speed;
extern uint16_t round_speed; // 车轮转速，单位round per second
image2_t image2_binary;
image2_t bin;
uint8_t sw_state = 0; // 调整模式
uint8_t last_s2_state = 1;
uint8_t select_pid = 0; // 选择PID的p或i或d
char temp_str[9];

float kp_pw = 6, ki_pw = 0, kd_pw = 14;
volatile int cross_detected = 0;		// cross detection flag

// PID controller struct
typedef struct {
    float kp, ki, kd;
    float integral;
    int16_t last_error;
    uint16_t out_min, out_max;
} pid_ctrl_t;

// PID init
static void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, uint16_t out_min, uint16_t out_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = 0; pid->last_error = 0;
    pid->out_min = out_min; pid->out_max = out_max;
}

char* float_to_str_2x3(float value, char *buf) {
    // ���Ŵ���
    int neg = (value < 0.0f);
    float absval = neg ? -value : value;

    // ����������С�����ַ���
    int int_part = (int)absval;
    float frac_part = absval - (float)int_part;

    // �������뵽��λС��
    int frac_3 = (int)(frac_part * 1000.0f + 0.5f);

    // ����С����λ���� 1.9996 -> 2.000��
    if (frac_3 >= 1000) {
        frac_3 -= 1000;
        int_part += 1;
    }

    // ֻ�����������ֵ������λ
    int_part = int_part % 100;

    // ���ո�ʽ�����
    if (neg) {
        // �����ţ�"-DD.DDD" �� 8 �ַ� + '\0'
        snprintf(buf, 9, "-%02d.%03d", int_part, frac_3);
    } else {
        // �������ţ�"DD.DDD" �� 7 �ַ� + '\0'
        snprintf(buf, 9,  "%02d.%03d", int_part, frac_3);
    }

    return buf;
}


// PID compute
static uint16_t pid_compute(pid_ctrl_t *pid, int16_t target, int16_t current, float dt) {
    int16_t err = target - current;
    pid->integral += err * dt;
    float d = (err - pid->last_error) / dt;
    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * d;
    pid->last_error = err;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return (uint16_t)out;
}

// LFR main
void Project_LFR(void) {
    // control variables
    int16_t target_speed = 200;
    int16_t current_speed;
    const float ctrl_period = 0.02f;
    pid_ctrl_t speed_pid;
    pid_init(&speed_pid, 0.05f, 0.001f, 0.001f, 0, 60);

    // encoder for knob
    uint8_t pha = PHA2(), phb = PHB2();
    uint8_t prev_pha = pha, prev_phb = phb;

    // init
    image_size = 2;
    MEN_HIGH();
    hsp_tft18_clear(BLACK);

    // labels
    hsp_tft18_show_str(0, 0, "Target:");
    hsp_tft18_show_str(0, 1, "Speed :");
    hsp_tft18_show_str(0, 2, "Duty  :");



    while (1) {
		int res = usart_data_receive(USART5);
		while(res == 101){
			hsp_motor_voltage(MOTORF, 0); // 停车
			res = usart_data_receive(USART5);
			if(res == 102){
				break;
			}
		}

		if(!S2() && sw_state==0){ // 按下S2切换到调整模式
			sw_state = 1; // 调整模式	
			hsp_tft18_clear(BLACK);
		}
		while(sw_state==1){ // 调整模式
			hsp_tft18_show_str(0, 0, "kp_pw: ");
			hsp_tft18_show_str(0, 1, "ki_pw: ");
			hsp_tft18_show_str(0, 2, "kd_pw: ");
			float_to_str_2x3(kp_pw, temp_str);
			hsp_tft18_show_str(60, 0, temp_str);
			float_to_str_2x3(ki_pw, temp_str);
			hsp_tft18_show_str(60, 1, temp_str);
			float_to_str_2x3(kd_pw, temp_str);
			hsp_tft18_show_str(60, 2, temp_str);

			if(!S2() && last_s2_state == 1) {
				select_pid++;
				if (select_pid > 2) select_pid = 0; // wrap around
			} 

			pha = PHA2(); phb = PHB2();
			// �����涨��
			const float STEP = 0.01f;

			// ÿ�μ�⵽��ť�仯ʱ����
			if (pha != prev_pha || phb != prev_phb) {
				// �ж����򣺳���������ֻ�� A ������ʱ B ���״̬
				int8_t dir;
				if (pha != prev_pha) {
					// A �������������� B ���жϷ���
					dir = (phb == pha) ? +1 : -1;
				} else {
					// B �����䣬�� A ���жϷ���
					dir = (pha != phb) ? +1 : -1;
				}

				// ���� select_pid ѡ��������ӻ����ͬ�� STEP
				switch (select_pid) {
				case 0: kp_pw += dir * STEP; break;
				case 1: ki_pw += dir * STEP; break;
				case 2: kd_pw += dir * STEP; break;
				}

				// ������ʷ״̬
				prev_pha = pha;
				prev_phb = phb;

				// ���ޱ���
				if (kp_pw < 0) kp_pw = 0;
				if (ki_pw < 0) ki_pw = 0;
				if (kd_pw < 0) kd_pw = 0;
			}

			

			if(!S1() && sw_state==1){ // ���ڵ������沢�����ϲ�����ť
				sw_state = 0; // �˻�������
				hsp_tft18_clear(BLACK);
			}
		}
		
		hsp_tft18_show_str(0, 0, "Target:");
		hsp_tft18_show_str(0, 1, "Speed :");
		hsp_tft18_show_str(0, 2, "Duty  :");
        // knob adjust
        pha = PHA2(); phb = PHB2();
        if (pha != prev_pha || phb != prev_phb) {
            if (phb == prev_phb) {
                target_speed += (phb == SET ? (pha == RESET ? 10 : -10) : (pha == SET ? 10 : -10));
            } else {
                target_speed += (pha == SET ? (phb == SET ? 10 : -10) : (phb == RESET ? 10 : -10));
            }
            prev_pha = pha; prev_phb = phb;
            if (target_speed < 0) target_speed = 0;
            if (target_speed > 200) target_speed = 200;
        }

        // camera processing
        if (image_ready == SET) {
            image_ready = RESET;
            hsp_image2_binary_sobel(image2_use, image2_temp);
			
            hsp_image2_binary_minmax(image2_use, bin);
            cross_detected = hsp_cross_detect(bin);
            if (cross_detected) BUZZ_ON(); else BUZZ_OFF();
            // steering (existing)
            uint16_t pw = hsp_image_judge2(image2_temp);
            static uint16_t pwt = 1500;
            static uint16_t tloss = 0;
            if (pw == 0) {
                tloss++;
                pw = pwt;
                if (tloss > 10) { pwt = 1500; tloss = 0; target_speed = 0; }
            } else tloss = 0;
            if (pw < 1300) pw = 1300; if (pw > 1700) pw = 1700;
            if (pw != pwt) {
                for (int i = 1; i <=4; i++) hsp_servo_angle(i, pw);
                pwt = pw;
            }
            hsp_tft18_show_int16(0, 3, pw);
        }

        // read speed
        current_speed = motor_get_speed(); if (current_speed<0) current_speed=-current_speed;
        // PID
        uint16_t duty = 3*pid_compute(&speed_pid, target_speed, current_speed, ctrl_period);
        if (cross_detected) { duty=0; speed_pid.integral=0; }
        hsp_motor_voltage(MOTORF, duty);

        // display
        hsp_tft18_show_int16(60,0,target_speed);
        hsp_tft18_show_int16(60,1,current_speed);
        hsp_tft18_show_int16(60,2,duty);
		if(!SW1())
			{
				hsp_image2_show_dma(image2_temp);
			}
			else
			{
				hsp_image2_show_dma(bin);
			}

        if (!S3()) break;
    }

    // exit
    hsp_motor_voltage(MOTORF,0);
    MEN_LOW(); while(!S3());
}


// Project#1: Line Following Robot (LFR)
/* void Project_LFR(void)
{
	uint16_t pw = 1500, pwt = 0;
	uint16_t dc = 0;
	uint16_t tloss = 0;				// target lost loop counter
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	
	image_size = 2;         // use 1/3 of the full size
	MEN_HIGH();					// enable H-bridge

	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;
	image_ready = RESET;
	
	hsp_tft18_clear(BLACK);
	
	while(1)
	{
//		if (!PUSH())			// push button pressed        
//		{
//			//delay_1ms(50);		// de-jitter
//			if (!PUSH())
//			{
//				while(!PUSH());
//				dc = 0;
//			}
//		}
//
//		state_pha = PHA2();			state_phb = PHB2();
//		if((state_pha_t != state_pha) || (state_phb_t != state_phb))
//		{
//			if(state_phb_t == state_phb)
//			{
//				if(SET == state_phb)
//				{
//					if(RESET == state_pha) dc++;
//					else if(0 < dc) dc--;
//				}
//				else
//				{
//					if(SET == state_pha) dc++;
//					else if(0 < dc) dc--;
//				}
//			}
//			else
//			{
//				if(SET == state_pha)
//				{
//					if(SET == state_phb) dc++;
//					else if(0 < dc) dc--;
//				}
//				else
//				{
//					if(RESET == state_pha) dc++;
//					else if(0 < dc) dc--;
//				}
//			}
//			state_pha_t = state_pha;
//			state_phb_t = state_phb;
//            //delay_1ms(10);		// de-jitter
//		}
//		// PWM output stage, subjected to duty cycle limits
//		if(35 < dc)
//			dc = 35;
//		if(SW2())
//		{
//			hsp_motor_voltage(MOTORF, dc);		// run forward
//		}
//		else
//		{
//			hsp_motor_voltage(MOTORB, dc);		// run backward
//		}
		
		// camera image processing
		if(image_ready == SET)
		{
			//threshold = hsp_image2_threshold_otsu(image2_use);
			//threshold = hsp_image2_threshold_mean(image2_use);
			//threshold = hsp_image2_threshold_minmax(image2_use);
			//hsp_image2_show_dma(image2_use);
			//hsp_image2_show_dma(image2_show);
			//hsp_image2_binary_minmax(image2_use, image2_temp);
			hsp_image2_binary_sobel(image2_use, image2_temp);

			image2_t image2_binary;

			hsp_image2_binary_minmax(image2_use, image2_binary);
			cross_detected = hsp_cross_detect(image2_binary);	// check for cross detection
			if(cross_detected)	// check for cross detection
			{
				BUZZ_ON();
			}
			else
			{
				BUZZ_OFF();
			}

			pw = hsp_image_judge2(image2_temp);
			if(pw == 0)
			{
				tloss++;
				pw = pwt;		// use previous result
				if(tloss > 10)	// off-road protection
				{
					dc = 0;
					pw = 1500;
					tloss = 0;
				}
			}
			else
			{
				tloss = 0;
			}
			
			// apply steering angle limits
			if(1700 < pw)
				pw = 1700;
			if(1300 > pw)
				pw = 1300;
			if(pwt != pw)
			{
				hsp_servo_angle(SERVO1, pw);
				hsp_servo_angle(SERVO2, pw);
				hsp_servo_angle(SERVO3, pw);
				hsp_servo_angle(SERVO4, pw);
				pwt = pw;
			}
			hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
			hsp_tft18_show_int16(0, 0, pw);
			hsp_tft18_show_int16_color(56, 0, encoder_speed, WHITE, BLACK);
			
			if(!SW1())
			{
				hsp_image2_show_dma(image2_use);
			}
			else
			{
				hsp_image2_show_dma(image2_binary);
			}
			image_ready = RESET;
		}
		
		if(!S3()) break;
	}
	
	hsp_servo_angle(SERVO1, 1500);
	hsp_servo_angle(SERVO2, 1500);
	hsp_servo_angle(SERVO3, 1500);
	hsp_servo_angle(SERVO4, 1500);
}*/

uint16_t hsp_image_judge2(image2_t image)
{
	uint16_t pw = 1500;
	uint8_t up_black_num = 0;
	uint8_t mid_black_num = 0;
	uint8_t low_black_num = 0;
	uint8_t up_left = 255, up_right = 255, mid_left = 255, mid_right = 255, low_left = 255, low_right = 255;	// 255 is an invalid value
	uint8_t up_mid_index = 255, mid_mid_index = 255, low_mid_index = 255;	// 255 is an invalid value
	uint8_t k;
	static uint8_t mid_index = 0, last_mid_index = 0;


	// ?????????????????????????????????????????????????��???????????
	for (uint8_t i = 1; i < IMAGEW2-1; i++)
	{
		if(image[5][i] == 0) 
		{
			up_black_num++;
			if(image[5][i-1] != 0)	// ?????????????????
			{
				if(up_left == 255) up_left = i;
				else up_right = i;
			}
		}
		if(image[20][i] == 0) 
		{
			mid_black_num++;
			if(image[20][i-1] != 0)	// ?????????????????
			{
				if(mid_left == 255) mid_left = i;
				else mid_right = i;
			}
		}
		if(image[35][i] == 0) 
		{
			low_black_num++;
			if(image[35][i-1] != 0)	// ?????????????????
			{
				if(low_left == 255) low_left = i;
				else low_right = i;
			}
		}
	}
	// ????????????????????????????????????????????????????????????
	hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
	hsp_tft18_show_uint8(60, 0, up_black_num);
	hsp_tft18_show_uint8(60, 1, mid_black_num);
	hsp_tft18_show_uint8(60, 2, low_black_num);

	// ???????????????????????????????????????????????��??????
	if(up_black_num <= 10 && up_black_num >2)
	{
		if(up_right == 255)
		{
			if(up_left < 94) 
			{
				if(up_left < 10) up_mid_index = 0;
				else up_mid_index = up_left / 2;
			}
			else
			{
				if(up_left > 177) up_mid_index = 187;
				else up_mid_index = (up_left+188) / 2;
			}
		}
		else up_mid_index = (up_left + up_right) / 2;
	}
	if(mid_black_num <= 10 && mid_black_num >2)
	{
		if(mid_right == 255)
		{
			if(mid_left < 94) 
			{
				if(mid_left < 10) mid_mid_index = 0;
				else mid_mid_index = mid_left / 2;
			}
			else
			{
				if(mid_left > 177) mid_mid_index = 187;
				else mid_mid_index = (mid_left+188) / 2;
			}
		}
		else mid_mid_index = (mid_left + mid_right) / 2;
	}
	if(low_black_num <= 10 && low_black_num >2)
	{
		if(low_right == 255)
		{
			if(low_left < 94) 
			{
				if(low_left < 10) low_mid_index = 0;
				else low_mid_index = low_left / 2;
			}
			else
			{
				if(low_left > 177) low_mid_index = 187;
				else low_mid_index = (low_left+188) / 2;
			}
		}
		else low_mid_index = (low_left + low_right) / 2;
	}
	
	if(up_mid_index != 255) mid_index = up_mid_index;
	else if(mid_mid_index != 255) mid_index = mid_mid_index;
	else if(low_mid_index != 255) mid_index = low_mid_index;
	else mid_index = last_mid_index;

	int8_t cur_pw_error = 94 - mid_index;						//
	int8_t last_pw_error = 94 - last_mid_index;					//
	int8_t diff_pw_error = cur_pw_error - last_pw_error;		//
	pw = 1500 + kp_pw * cur_pw_error + kd_pw * diff_pw_error;
	last_mid_index = mid_index;
	
	return pw;
}

uint16_t hsp_image_judge(image2_t image)
{
	uint16_t pw;			// pulse-width control steering angle
	uint8_t i, j;
	uint8_t gte_l, gte_r, gte_ok;				// guide tape edge flag
	uint8_t gte_l_idx, gte_r_idx, gte_c_idx;		// guide tape index
	
	gte_l = RESET;
	gte_r = RESET;
	gte_ok = RESET;
	for(i=2; i<(IMAGEW2-2); i++)
	{
		if(RESET == gte_l)
		{
			if((255 == image[20][i]) && (0 == image[20][i+1]))	// left edge found
			{
				gte_l = SET;
				gte_l_idx = i;									// left edge index
			}
		}
		if((SET == gte_l) && (RESET == gte_r))
		{
			if((0 == image[20][i]) && (255 == image[20][i+1]))	// right edge found
			{
				gte_r = SET;
				gte_r_idx = i;									// right edge index
			}
		}
		if((SET == gte_l) && (SET == gte_r) && (RESET == gte_ok))		// both edges found
		{
			if(((gte_r_idx - gte_l_idx) > 6) && ((gte_r_idx - gte_l_idx) < 30))		// proper tape width
			{
				gte_ok = SET;
				gte_c_idx = (gte_r_idx + gte_l_idx) >> 1;	// tape center index
			}
			else
			{
				gte_l = RESET;
				gte_r = RESET;
				gte_ok = RESET;
			}
		}
	}
	
	if(SET == gte_ok)
		pw = 1500 + 10 * (94 - gte_c_idx);
	else
		pw = 0;
	
	return pw;
}

// ??????????????????????????????????????????????1?????????????????????????????????0???????????????????
uint8_t hsp_cross_detect(image2_t image)
{
	// ???��?��???????????????????????????????????????????????????
	uint8_t rows[3] = {19, 20, 21}; // ????????????????????
	uint8_t cols[3] = {93, 94, 95}; // ????????????????????
	uint8_t row_black[3] = {0}, col_black[3] = {0};
	uint8_t i, j;

	// ????????????????????????????????
	for(j = 0; j < 3; j++) {
		for(i = 0; i < IMAGEW2; i++) {
			if(image[rows[j]][i] == 0) row_black[j]++;
		}
	}
	// ????????????????????????????????
	for(j = 0; j < 3; j++) {
		for(i = 0; i < IMAGEH2; i++) {
			if(image[i][cols[j]] == 0) col_black[j]++;
		}
	}

	// ????????????��?????????????????????????????????????????????????????????????????????????????????????????????
	if(row_black[0] > 30 && row_black[1] > 30 && row_black[2] > 30 &&
	   col_black[0] > 30 && col_black[1] > 30 && col_black[2] > 30) {
		return 1;
	} else {
		return 0;
	}
}

// ???��???????????????????????????????????????????????????????????
void hsp_image2_binary_simple(image2_t input, image2_t output)
{
	int i, j;
	uint8_t threshold = 128; // ???????????????????????????????????????????????????

	for(i = 0; i < IMAGEH2; i++)
	{
		for(j = 0; j < IMAGEW2; j++)
		{
			if(input[i][j] < threshold)
				output[i][j] = 0U;
			else
				output[i][j] = 255U;
		}
	}
}