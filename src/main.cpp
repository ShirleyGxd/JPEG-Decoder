#include<iostream>
#include<stdlib.h>
#include<math.h>
#include<Windows.h>

#define MAXLENGTH 200000
#define Byte unsigned char//1字节
#define SByte signed char
#define TByte unsigned short int//2字节
#define STByte signed short int
#define uint unsigned int
//常用标记
#define SOI 0xD8
#define EOI 0xD9
#define SOS 0xDA
#define DQT 0xDB
#define DNL 0xDC
#define DRI 0xDD
#define DHT 0xC4


//帧开始标志
//huffman编码
#define SOF0 0xC0//基本DCT
#define SOF1 0xC1//扩展顺序DCT
#define SOF2 0xC2//累进DCT
#define SOF3 0xC3//无失真过程

#define SOF5 0xC5//差分顺序DCT
#define SOF6 0xC6//差分累进DCT
#define SOF7 0xC7//差分无失真过程

//算数编码
#define SOF9 0xC9
#define SOF10 0xCA
#define SOF11 0xCB

#define SOF13 0xCD
#define SOF14 0xCE
#define SOF15 0xCF


// IDCT parameters
#define  C1 0.9808
#define  C2 0.9239
#define  C3 0.8315
#define  C4 0.7071
#define  C5 0.5556
#define  C6 0.3827
#define  C7 0.1951

static Byte zigzag[64] = { 0, 1, 5, 6, 14, 15, 27, 28,
2, 4, 7, 13, 16, 26, 29, 42,
3, 8, 12, 17, 25, 30, 41, 43,
9, 11, 18, 24, 31, 40, 44, 53,
10, 19, 23, 32, 39, 45, 52, 54,
20, 22, 33, 38, 46, 51, 55, 60,
21, 34, 37, 47, 50, 56, 59, 61,
35, 36, 48, 49, 57, 58, 62, 63 };


uint X_image_bytes;	//图像一行的byte数，因为包含三个通道，所以等于H_round*3



Byte *in_buffer,*out_buffer;


struct pixel_RGB{
	Byte B;
	Byte G;
	Byte R;
};
//---------------------------待定---------------------------------
struct BMP_header{
	TByte Type=0x4D42;//BM
	uint Size;
	uint zero_Reserved=0;
	uint Offbits=54;

	uint biSize=40;
	uint Width;
	uint Height;
	TByte Planes=1;
	TByte BitCount=24;
	uint Compression=0;
	uint biImageSize=0;//因为采用不压缩模式，所以可设置为0
	uint biXPelsPerMeter = 0xB40;
	uint biYPelsPerMeter = 0xB40;
	uint biClrUsed=0;
	uint biClrImportant=0;
};

BMP_header bmp_header_buffer;
//---------------------------待定---------------------------------
struct BitPlace{
	uint Addr;
	Byte BitNumber;
};
BitPlace Current_Bit_Place;//待读的bit的位置

TByte QT_8bits[4][64];


TByte QT_16bits[4][64];


struct Huffman_Table{
	Byte Number[16];
	TByte TotalNumber;
	Byte *Weight;
	TByte Addr[16];
	TByte min_code[16];
	TByte max_code[16];
};

Huffman_Table HT_DC[2],HT_AC[2];
void init_HT(Huffman_Table *HT)
{
	HT->Weight = (Byte *)malloc(HT->TotalNumber*sizeof(Byte));
}

void build_HT(Huffman_Table *HT)
{
	int i = 0;
	while (i < 16 && 0 == HT->Number[i])
	{
		HT->Addr[i] = HT->TotalNumber + 1;
		HT->min_code[i] = 0xFFFF;
		HT->max_code[i] = 0x0000;
		i++;
	}
	if (i < 16)
	{
		HT->Addr[i] = 0;
		HT->min_code[i] = 0x0000;
		HT->max_code[i] = HT->min_code[i] + HT->Number[i] - 1;
		int last = i;
		i++;
		for (; i < 16; i++)
		{
			if (0 == HT->Number[i])
			{
				HT->Addr[i] = HT->TotalNumber + 1;
				HT->min_code[i] = 0xFFFF;
				HT->max_code[i] = 0x0000;
			}
			else
			{
				HT->Addr[i] = HT->Addr[last]+HT->Number[last];
				HT->min_code[i] = (HT->max_code[last]+1)<<(i-last);
				HT->max_code[i] = HT->min_code[i] + HT->Number[i] - 1;
				last = i;
			}
		}
	}

}


struct image{
	uint rows;
	uint cols;
};
image Y_image, Cr_image, Cb_image;

uint V_round, H_round;
uint V_image, H_image;

struct Color{
	Byte H_sample;
	Byte V_sample;
	Byte QT_number;
	Byte HT_DC_number;
	STByte pre_DC;
	Byte HT_AC_number;
};

Color ColorsInfo[3],Y_info,Cb_info,Cr_info;

int Restart_maker;
uint MCU_resrtart_th;

long data_place;
Byte get_byte(long *place)
{
	Byte Byte_value = in_buffer[*place];
	*place++;
	return Byte_value;
}



void zigzag_QT()
{
	TByte tmp_QT[64];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 64; j++)
		{
			tmp_QT[j] = QT_8bits[i][j];
		}
		for (int t = 0; t < 64; t++)
		{
			QT_8bits[i][t] = tmp_QT[zigzag[t]];
		}
	}
}

int read_JPG_Header(Byte *buffer){
	
	int Lq, Pq, Tq, DQT_end;//DQT
	int Lh, end_place;//DHT
	int Lr;//DRI
	int Lf, P, Nf;//SOF0;
	int Ls, Ns, tmp_place;//SOS
	TByte tmp_sum;

	long place = 0;
	data_place = 0;
	//通过SOI标记，确定是否是JPEG文件
	while (buffer[place] == 0xFF && buffer[place + 1] == 0xFF)
	{
		place++;
	}
	if (buffer[place] != 0xFF || buffer[place+1]!=SOI)
	{
		printf("这不是JPG文件！\n");
		system("pause");
		return 0;
	}
	place += 2;
	
	int SOS_found = 0, SOF_found = 0;
	Restart_maker = 0;
	while (!(buffer[place] == 0xFF && buffer[place + 1] == EOI) && SOS_found != 1)
	{
		if (buffer[place] != 0xFF)
		{
			place++;
			continue;
		} 
		place++;
		switch (buffer[place])
		{
		case DQT://读取量化表
			Lq = (buffer[place + 1] << 8) + buffer[place + 2];
			DQT_end = place + Lq;
			place += 3;
			while (place <= DQT_end)
			{
				Pq = buffer[place] >> 4;
				Tq = buffer[place] & 0x0F;
				place++;
				if (!Pq)//量化精度为8位
				{
					for (int j = 0; j < 64; j++)
					{
						QT_8bits[Tq][j] = buffer[place];
						place++;
					}
				}
				else//量化精度为16位
				{
					for (int j = 0; j < 64; j++)
					{
						QT_16bits[Tq][j] = (buffer[place] << 8) + buffer[place + 1];
						place += 2;
					}
					printf("暂不支持量化精度为16位模式！\n");
					system("pause");
					return 0;
				}
			}
			break;
		case DHT://读取huffman表
			Lh = (buffer[place + 1] << 8) + buffer[place + 2];
			end_place = place + Lh;
			place += 3;
			while (place<=end_place)
			{
				int DHT_Info = buffer[place];
				if (DHT_Info == 0x00)
				{
					place++;
					HT_DC[0].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_DC[0].Number[i] = buffer[place];
						HT_DC[0].TotalNumber += HT_DC[0].Number[i];
						place++;
					}
					init_HT(&HT_DC[0]);
					for (int i = 0; i < HT_DC[0].TotalNumber; i++)
					{
						HT_DC[0].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_DC[0]);
				}
				else if (DHT_Info == 0x01)
				{
					place++;
					HT_DC[1].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_DC[1].Number[i] = buffer[place];
						HT_DC[1].TotalNumber += HT_DC[1].Number[i];
						place++;
					}
					init_HT(&HT_DC[1]);
					for (int i = 0; i < HT_DC[1].TotalNumber; i++)
					{
						HT_DC[1].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_DC[1]);
				}
				else if (DHT_Info == 0x10)
				{
					place++;
					HT_AC[0].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_AC[0].Number[i] = buffer[place];
						HT_AC[0].TotalNumber += HT_AC[0].Number[i];
						place++;
					}
					init_HT(&HT_AC[0]);
					for (int i = 0; i < HT_AC[0].TotalNumber; i++)
					{
						HT_AC[0].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_AC[0]);
				}
				else if (DHT_Info == 0x11)
				{
					place++;
					HT_AC[1].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_AC[1].Number[i] = buffer[place];
						HT_AC[1].TotalNumber += HT_AC[1].Number[i];
						place++;
					}
					init_HT(&HT_AC[1]);
					for (int i = 0; i < HT_AC[1].TotalNumber; i++)
					{
						HT_AC[1].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_AC[1]);
				}
				else 
					continue;
			}
			break;
		case DRI:
			Lr = (buffer[place + 1] << 8) + buffer[place + 2];
			MCU_resrtart_th = (buffer[place + 3] << 8) + buffer[place + 4];
			if (0 == MCU_resrtart_th)
			{
				Restart_maker = 0;
			}
			else
			{
				Restart_maker = 1;
				printf("暂不支持重启动模式！\n");
				system("pause");
				return 0;
			}
			place = place + Lr + 1;
			break;
		case SOF0:
			Lf = (buffer[place + 1] << 8) + buffer[place + 2];
			P = buffer[place + 3];
			if (P != 8)
			{
				printf("只支持8位精度！\n");
				system("pause");
				return 0;
			}
			place += 4;
			Y_image.rows = (buffer[place] << 8) + buffer[place + 1];
			Y_image.cols = (buffer[place + 2] << 8) + buffer[place + 3];
			Nf = buffer[place+4];
			if (Nf != 3)
			{
				printf("只支持3个颜色分量的彩色图片！\n");
				system("pause");
				return 0;
			}
			place +=5;
			for (int i = 0; i < 3; i++)
			{
				int Ci = buffer[place];
				if (Ci < 1 || Ci > 3)
				{
					printf("只支持颜色分量编号1-3的彩色图片！\n");
					system("pause");
					return 0;
				}
				
				ColorsInfo[Ci - 1].H_sample = buffer[place + 1] >> 4;
				ColorsInfo[Ci - 1].V_sample = buffer[place + 1] & 0x0F;
				ColorsInfo[Ci - 1].QT_number = buffer[place + 2];
				place += 3;
			}
			Y_info = ColorsInfo[0];
			Cb_info = ColorsInfo[1];
			Cr_info = ColorsInfo[2];
			SOF_found = 1;
			break;
		case SOF1:
		case SOF2:
		case SOF3: 
		case SOF5:
		case SOF6:
		case SOF7:
		case SOF9:
		case SOF10:
		case SOF11:
		case SOF13:
		case SOF14:
		case SOF15: 
			printf("暂不支持除基本DCT（SOF0）外的模式\n"); 
			system("pause");
			return 0;
		
		case SOS:
			Ls = (buffer[place + 1] << 8) + buffer[place + 2];
			Ns = buffer[place + 3];
			if (Ns != 3)
			{
				printf("不合理的SOS标记！\n");
				system("pause");
				return 0;
			}
			tmp_place = place + 4;
			for (int i = 0; i < 3; i++)
			{
				int Cs = buffer[tmp_place];
				if (Cs < 1 || Cs>3)
				{
					printf("SOS头标中的分量设置不合理！\n");
					system("pause");
					return 0;
				}
				ColorsInfo[Cs - 1].HT_DC_number = buffer[tmp_place + 1] >> 4;
				ColorsInfo[Cs - 1].HT_AC_number = buffer[tmp_place + 1] & 0x0F;
				tmp_place += 2;
			}
			Y_info = ColorsInfo[0];
			Cr_info = ColorsInfo[1];
			Cb_info = ColorsInfo[2];
			SOS_found = 1;
			place = place + Ls + 1;
			data_place = place;
			Current_Bit_Place.Addr = data_place;
			Current_Bit_Place.BitNumber = 7;//该字节的最高位
			break;
		default: place++; break;
		}
	}
	if (!SOS_found)
	{
		printf("没有发现SOS标记！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}
	if (!SOF_found)
	{
		printf("没有发现SOF标记！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}
	if (Cb_info.H_sample>Y_info.H_sample || Cr_info.H_sample>Y_info.H_sample)
	{
		printf("Y在垂直方向的采样因子小于了Cr或Cb！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}
	if (Cb_info.V_sample>Y_info.V_sample || Cr_info.V_sample>Y_info.V_sample)
	{
		printf("Y在水平方向的采样因子小于了Cr或Cb！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}
	if (Cb_info.H_sample>=2|| Cb_info.V_sample>=2)
	{
		printf("Cb的采样因子只能为1！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}
	if (Cr_info.H_sample >= 2 || Cr_info.V_sample >= 2)
	{
		printf("Cr的采样因子只能为1！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}

	H_image = Y_image.cols;
	V_image = Y_image.rows;

	if (Y_image.rows % (Y_info.V_sample * 8) == 0)
		V_round = Y_image.rows;
	else
		V_round = (Y_image.rows / (Y_info.V_sample * 8) + 1)*(Y_info.V_sample * 8);
	if (Y_image.cols % (Y_info.H_sample * 8) == 0)
		H_round = Y_image.cols;
	else
		H_round = (Y_image.cols / (Y_info.H_sample * 8) + 1)*(Y_info.H_sample * 8);
	
	out_buffer = (Byte *)malloc(H_round*V_round * 3);
	
	zigzag_QT();

	printf("头文件解码结束后的位置为：%x\n", place);
	return 1;
}




TByte get_1_bit()
{
	TByte BitValue;
	BitValue = (in_buffer[Current_Bit_Place.Addr] >> Current_Bit_Place.BitNumber) & 0x01;
	if (0 == Current_Bit_Place.BitNumber)
	{
		if (0xFF == in_buffer[Current_Bit_Place.Addr] && 0x00 == in_buffer[Current_Bit_Place.Addr+1])
		{
			Current_Bit_Place.Addr+=2;
			Current_Bit_Place.BitNumber = 7;
		}
		else
		{
			Current_Bit_Place.Addr++;
			Current_Bit_Place.BitNumber = 7;
		}	
	}
	else
	{
		Current_Bit_Place.BitNumber--;
	}
	return BitValue;
}



//-------------------------------------------
unsigned int huffman_decode_times = 1;
//--------------------------------------------

STByte tmp_8x8_unit[64]; //huffman解码得到的8x8的块
STByte DCT_coeff[64];
int huffman_decode_8x8_unit(Byte HT_DC_num,Byte HT_AC_num,STByte *pre_DC)
{
	TByte DC_code=0, AC_code=0;
	TByte DC_value, AC_value;
	Byte size;
	
	for (int i = 0; i < 64; i++)
	{
		tmp_8x8_unit[i] = 0;
	}

	int Weight_Place;

	//解码DC系数
	int DC_code_found = 0;
	DC_value = 0;
	for (int k = 0; k < 16; k++)
	{
		TByte tmp_Bit;
		tmp_Bit = get_1_bit();
		DC_code = (DC_code << 1) + tmp_Bit;
		if (DC_code >= HT_DC[HT_DC_num].min_code[k]&&DC_code <= HT_DC[HT_DC_num].max_code[k])
		{
			DC_code_found = 1;
			Weight_Place = HT_DC[HT_DC_num].Addr[k] + (DC_code - HT_DC[HT_DC_num].min_code[k]);
			size = HT_DC[HT_DC_num].Weight[Weight_Place];
			break;
		}
	}
	if (DC_code_found)
	{
		if (size > 15)
		{
			printf("DC系数太大，超过15位！\n");
			system("pause");
			return 0;
		}
		else if (0 == size)
		{
			tmp_8x8_unit[0] = 0;
		}
		else
		{
			for (int i = 0; i < size; i++)
			{
				TByte tmp_Bit;
				tmp_Bit = get_1_bit();
				DC_value = (DC_value << 1) + tmp_Bit;
			}
			int sign;
			sign = (DC_value >> (size - 1));
			if (1 == sign)
			{
				tmp_8x8_unit[0] = DC_value;
			}
			else
			{
				int tmp = 0x00;
				for (int i = 0; i < size; i++)
				{
					tmp = (tmp << 1) + 1;
				}
				tmp_8x8_unit[0] = 0-((~DC_value) & tmp);

			}
		}

		tmp_8x8_unit[0] += *pre_DC;
		*pre_DC = tmp_8x8_unit[0];
	}
	else
	{
		printf("没有找到DC_code!\n");
		system("pause");
		return 0;
	}

	//解码AC系数
	int j = 1;
	int EOB_found = 0;
	int RunLegnth;

	while ((j < 64) && (!EOB_found))
	{
		
		int AC_code_found=0;
		AC_code = 0;
		AC_value = 0;
		for (int k = 0; k < 16; k++)
		{
			TByte tmp_Bit;
			tmp_Bit = get_1_bit();
			AC_code = (AC_code << 1) + tmp_Bit;
			if (AC_code >= HT_AC[HT_AC_num].min_code[k] && AC_code <= HT_AC[HT_AC_num].max_code[k])
			{
				AC_code_found = 1;
				Weight_Place = HT_AC[HT_AC_num].Addr[k] + (AC_code - HT_AC[HT_AC_num].min_code[k]);
				RunLegnth = HT_AC[HT_AC_num].Weight[Weight_Place] >> 4;
				j += RunLegnth;
				size = HT_AC[HT_AC_num].Weight[Weight_Place] & 0x0f;
				break;
			}
		}
		if (AC_code_found)
		{
			if (0 == size)
			{
				if (0 == RunLegnth)
					EOB_found = 1;
				else
				{
					tmp_8x8_unit[j] = 0;
					j++;
				}

			}
			else
			{
				for (int i = 0; i < size; i++)
				{
					TByte tmp_Bit;
					tmp_Bit = get_1_bit();
					AC_value = (AC_value << 1) + tmp_Bit;
				}
				int sign;
				sign = (AC_value >> (size - 1));
				if (1 == sign)
				{
					tmp_8x8_unit[j] = AC_value;
				}
				else
				{
					int tmp = 0x00;
					for (int i = 0; i < size; i++)
					{
						tmp = (tmp << 1) + 1;
					}
					tmp_8x8_unit[j] = 0 - ((~AC_value) & tmp);
				}
				j++;
			}
		}
		else
		{
			printf("没有找到AC_code!\n");
			system("pause");
			return 0;
		}
		
	}
	
	//zigzag//
	for (int t = 0; t < 64; t++) 
		DCT_coeff[t] = tmp_8x8_unit[zigzag[t]];
}



void restart()
{
	Y_info.pre_DC = 0; 
	Cb_info.pre_DC = 0;
	Cr_info.pre_DC = 0;
}

//---------------------------------------------------------------------------------

STByte DCT_coeff_IQ[64];//IQ后输出的buffer

void IQuntization(STByte *input, STByte *output, Byte QT_num) 
{
	TByte *Qtable;
	Qtable = QT_8bits[QT_num];
	for (int i = 0; i < 64; i++) {
		output[i] = input[i] * Qtable[i];
	}
}

SByte Y_DCT_coeff[64], Cr_DCT_coeff[64], Cb_DCT_coeff[64];//IDCT后输出的buffer
//IDCT in rows
void IDCT_row(STByte *row_start, double *row_idct) {
	double tmp[16];
	//first step
	tmp[0] = row_start[0] * C4 + row_start[2] * C2;
	tmp[1] = row_start[4] * C4 + row_start[6] * C6;
	tmp[2] = row_start[0] * C4 + row_start[2] * C6;
	tmp[3] = -row_start[4] * C4 - row_start[6] * C2;
	tmp[4] = row_start[0] * C4 - row_start[2] * C6;
	tmp[5] = -row_start[4] * C4 + row_start[6] * C2;
	tmp[6] = row_start[0] * C4 - row_start[2] * C2;
	tmp[7] = row_start[4] * C4 - row_start[6] * C6;

	tmp[8] = row_start[1] * C7 - row_start[3] * C5;
	tmp[9] = row_start[5] * C3 - row_start[7] * C1;
	tmp[10] = row_start[1] * C5 - row_start[3] * C1;
	tmp[11] = row_start[5] * C7 + row_start[7] * C3;
	tmp[12] = row_start[1] * C3 - row_start[3] * C7;
	tmp[13] = -row_start[5] * C1 - row_start[7] * C5;
	tmp[14] = row_start[1] * C1 + row_start[3] * C3;
	tmp[15] = row_start[5] * C5 + row_start[7] * C7;
	//second step
	tmp[0] = 0.5*(tmp[0] + tmp[1]);
	tmp[1] = 0.5*(tmp[2] + tmp[3]);
	tmp[2] = 0.5*(tmp[4] + tmp[5]);
	tmp[3] = 0.5*(tmp[6] + tmp[7]);
	tmp[4] = 0.5*(tmp[8] + tmp[9]);
	tmp[5] = 0.5*(tmp[10] + tmp[11]);
	tmp[6] = 0.5*(tmp[12] + tmp[13]);
	tmp[7] = 0.5*(tmp[14] + tmp[15]);
	//third step
	row_idct[0] = tmp[0] + tmp[7];
	row_idct[1] = tmp[1] + tmp[6];
	row_idct[2] = tmp[2] + tmp[5];
	row_idct[3] = tmp[3] + tmp[4];
	row_idct[4] = tmp[3] - tmp[4];
	row_idct[5] = tmp[2] - tmp[5];
	row_idct[6] = tmp[1] - tmp[6];
	row_idct[7] = tmp[0] - tmp[7];
}


// IDCT in cols
void IDCT_col(double *col_start, SByte *idct_output)
{
	double tmp[16];
	//idct_output = (STByte*)malloc(sizeof(STByte) * 64);
	//first step
	tmp[0] = col_start[0 * 8] * C4 + col_start[2 * 8] * C2;
	tmp[1] = col_start[4 * 8] * C4 + col_start[6 * 8] * C6;
	tmp[2] = col_start[0 * 8] * C4 + col_start[2 * 8] * C6;
	tmp[3] = -col_start[4 * 8] * C4 - col_start[6 * 8] * C2;
	tmp[4] = col_start[0 * 8] * C4 - col_start[2 * 8] * C6;
	tmp[5] = -col_start[4 * 8] * C4 + col_start[6 * 8] * C2;
	tmp[6] = col_start[0 * 8] * C4 - col_start[2 * 8] * C2;
	tmp[7] = col_start[4 * 8] * C4 - col_start[6 * 8] * C6;

	tmp[8] = col_start[1 * 8] * C7 - col_start[3 * 8] * C5;
	tmp[9] = col_start[5 * 8] * C3 - col_start[7 * 8] * C1;
	tmp[10] = col_start[1 * 8] * C5 - col_start[3 * 8] * C1;
	tmp[11] = col_start[5 * 8] * C7 + col_start[7 * 8] * C3;
	tmp[12] = col_start[1 * 8] * C3 - col_start[3 * 8] * C7;
	tmp[13] = -col_start[5 * 8] * C1 - col_start[7 * 8] * C5;
	tmp[14] = col_start[1 * 8] * C1 + col_start[3 * 8] * C3;
	tmp[15] = col_start[5 * 8] * C5 + col_start[7 * 8] * C7;


	//second step
	tmp[0] = 0.5*(tmp[0] + tmp[1]);
	tmp[1] = 0.5*(tmp[2] + tmp[3]);
	tmp[2] = 0.5*(tmp[4] + tmp[5]);
	tmp[3] = 0.5*(tmp[6] + tmp[7]);
	tmp[4] = 0.5*(tmp[8] + tmp[9]);
	tmp[5] = 0.5*(tmp[10] + tmp[11]);
	tmp[6] = 0.5*(tmp[12] + tmp[13]);
	tmp[7] = 0.5*(tmp[14] + tmp[15]);

	//third step
	idct_output[0 * 8] = round(tmp[0] + tmp[7]);
	idct_output[1 * 8] = round(tmp[1] + tmp[6]);
	idct_output[2 * 8] = round(tmp[2] + tmp[5]);
	idct_output[3 * 8] = round(tmp[3] + tmp[4]);
	idct_output[4 * 8] = round(tmp[3] - tmp[4]);
	idct_output[5 * 8] = round(tmp[2] - tmp[5]);
	idct_output[6 * 8] = round(tmp[1] - tmp[6]);
	idct_output[7 * 8] = round(tmp[0] - tmp[7]);
}


void IDCT(STByte *input, SByte *output) 
{
	double middle_matrix[64];
	for (int i = 0; i < 8; i++) 
		IDCT_row(&input[8 * i], &middle_matrix[8 * i]);

	

	for (int j = 0; j < 8; j++) {
		IDCT_col(&middle_matrix[j], &output[j]);
		//printf("%d\n", &output[i]);
	}
}


void convert_YCbCr_to_RGB(SByte *Y, SByte *Cb, SByte *Cr, uint location, uint image_bytes, Byte *im_buffer) 
{
	Byte *ibuffer = im_buffer + location;
	SByte Y_temp, Cb_temp, Cr_temp;
	STByte B_temp, G_temp, R_temp;
	//TByte Cr_Cb;
	Byte nr, im_nr;// nr is used for each element of YCrCb, im_nr is sequence of im_buffer
	//ibuffer = im_buffer + location;// adress of image buffer
	nr = 0;
	for (int i = 0; i < 8; i++) {
		im_nr = 0;// for every row begin with the first element
		for (int j = 0; j < 8; j++) {
			Y_temp = Y[nr]; Cb_temp = Cb[nr]; Cr_temp = Cr[nr];
			// the sequnce of saving image is B-G-R
			
			B_temp = Y_temp + 1.77200*Cb_temp + 128;  //B
			if (B_temp > 255)  ibuffer[im_nr++] = (Byte)255;
			else if (B_temp < 0) ibuffer[im_nr++] = (Byte)0;
			else ibuffer[im_nr++] = (Byte)B_temp;
			
			G_temp = Y_temp - 0.34414*Cb_temp - 0.71414*Cr_temp + 128;  //G
			if (G_temp > 255)  ibuffer[im_nr++] = (Byte)255;
			else if (G_temp < 0) ibuffer[im_nr++] = (Byte)0;
			else ibuffer[im_nr++] = (Byte)G_temp;

			R_temp = Y_temp + 1.40200*Cr_temp + 128;  //R
			if (R_temp > 255)  ibuffer[im_nr++] = (Byte)255;
			else if (R_temp < 0) ibuffer[im_nr++] = (Byte)0;
			else ibuffer[im_nr++] = (Byte)R_temp;

			nr++;
		}
		ibuffer += image_bytes;// to the next row
	}

}



//////////////////////////////////////////////////////////////////////////////////////////
////待定函数start
//////////////////////////////////////////////////////////////////////////////////////////
void decode_MCU_1x1(uint im_loc)
{
	//Y
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y_DCT_coeff);
	
	//Cr
	huffman_decode_8x8_unit(Cr_info.HT_DC_number, Cr_info.HT_AC_number, &Cr_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cr_info.QT_number);
	IDCT(DCT_coeff_IQ, Cr_DCT_coeff);
	
	//Cb
	huffman_decode_8x8_unit(Cb_info.HT_DC_number, Cb_info.HT_AC_number, &Cb_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cb_info.QT_number);
	IDCT(DCT_coeff_IQ, Cb_DCT_coeff);
	
	convert_YCbCr_to_RGB(Y_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
}

SByte Y1_DCT_coeff[64], Y2_DCT_coeff[64], Y3_DCT_coeff[64], Y4_DCT_coeff[64];
void decode_MCU_2x1(uint im_loc)
{
	uint Hstep;
	Hstep = 8 * 3;
	//Y
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y1_DCT_coeff);
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y2_DCT_coeff);


	//Cb
	huffman_decode_8x8_unit(Cb_info.HT_DC_number, Cb_info.HT_AC_number, &Cb_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cb_info.QT_number);
	IDCT(DCT_coeff_IQ, Cb_DCT_coeff);

	//Cr
	huffman_decode_8x8_unit(Cr_info.HT_DC_number, Cr_info.HT_AC_number, &Cr_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cr_info.QT_number);
	IDCT(DCT_coeff_IQ, Cr_DCT_coeff);

	

	convert_YCbCr_to_RGB(Y1_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
	im_loc += Hstep;
	convert_YCbCr_to_RGB(Y2_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
}

void decode_MCU_1x2(uint im_loc)
{
	uint Vstep;
	Vstep = X_image_bytes * 8;
	//Y
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y1_DCT_coeff);
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y2_DCT_coeff);
	
	//Cb
	huffman_decode_8x8_unit(Cb_info.HT_DC_number, Cb_info.HT_AC_number, &Cb_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cb_info.QT_number);
	IDCT(DCT_coeff_IQ, Cb_DCT_coeff);

	//Cr
	huffman_decode_8x8_unit(Cr_info.HT_DC_number, Cr_info.HT_AC_number, &Cr_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cr_info.QT_number);
	IDCT(DCT_coeff_IQ, Cr_DCT_coeff);

	
	

	convert_YCbCr_to_RGB(Y1_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
	im_loc += Vstep;
	convert_YCbCr_to_RGB(Y2_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
}

void decode_MCU_2x2(uint im_loc)
{
	uint Hstep, Vstep;
	Hstep = 8 * 3;
	Vstep = -24 + X_image_bytes * 8;
	//Y
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y1_DCT_coeff);
	/*//-------------------------------------------------------
	printf("\nhuffman result:%d\n", huffman_decode_times);
	for (int i = 0; i < 64; i++)
	{
		printf("%d ", DCT_coeff[i]);
	}
	printf("\nIDCT result:%d\n", huffman_decode_times);
	for (int i = 0; i < 64; i++)
	{
		printf("%d ", Y1_DCT_coeff[i]);
	}
	huffman_decode_times++;
	//---------------------------------------------------------*/


	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y2_DCT_coeff);
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y3_DCT_coeff);
	huffman_decode_8x8_unit(Y_info.HT_DC_number, Y_info.HT_AC_number, &Y_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Y_info.QT_number);
	IDCT(DCT_coeff_IQ, Y4_DCT_coeff);

	//Cb
	huffman_decode_8x8_unit(Cb_info.HT_DC_number, Cb_info.HT_AC_number, &Cb_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cb_info.QT_number);
	IDCT(DCT_coeff_IQ, Cb_DCT_coeff);

	//Cr
	huffman_decode_8x8_unit(Cr_info.HT_DC_number, Cr_info.HT_AC_number, &Cr_info.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, Cr_info.QT_number);
	IDCT(DCT_coeff_IQ, Cr_DCT_coeff);




	convert_YCbCr_to_RGB(Y1_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
	im_loc += Hstep;
	convert_YCbCr_to_RGB(Y2_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
	im_loc += Vstep;
	convert_YCbCr_to_RGB(Y3_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
	im_loc += Hstep;
	convert_YCbCr_to_RGB(Y4_DCT_coeff, Cb_DCT_coeff, Cr_DCT_coeff, im_loc, X_image_bytes, out_buffer);
}


void decode_JPEG_image()
{
	uint im_loc;//当前MCU在图像中的位置
	TByte processed_MCU_num;//已经处理过的MCU的数目
	TByte X_MCU_bytes;//一个MCU一行的byte数
	TByte change_line_distance;
	TByte X_MCU_num, Y_MCU_num;
	Byte Hmax, Vmax;
	Hmax = Y_info.H_sample;
	Vmax = Y_info.V_sample;

	void(*decode_MCU)(uint);
	if (1 == Y_info.H_sample && 1 == Y_info.V_sample)
		decode_MCU = decode_MCU_1x1;
	else if (1 == Y_info.H_sample && 2 == Y_info.V_sample)
		decode_MCU = decode_MCU_1x2;
	else if (2 == Y_info.H_sample && 1 == Y_info.V_sample)
		decode_MCU = decode_MCU_2x1;
	else
		decode_MCU = decode_MCU_2x2;

	X_MCU_num = H_round / (Hmax * 8);	//水平方向的MCU数目
	Y_MCU_num = V_round / (Vmax * 8);	//竖直方向的MCU数目

	X_image_bytes = H_round * 3;
	X_MCU_bytes = Hmax * 8 * 3;
	change_line_distance = X_image_bytes * 8 * Vmax - X_image_bytes;

	im_loc = 0;
	processed_MCU_num = 0;

	for (int y = 0; y < Y_MCU_num; y++)
	{
		for (int x = 0; x < X_MCU_num; x++)
		{
			decode_MCU(im_loc);////待定
			im_loc += X_MCU_bytes;
			processed_MCU_num++;
		}
		im_loc += change_line_distance;
	}

}



int write_to_BMP(char *BMP_path)
{
	
	//恢复图片实际大小的buffer,存为bmp_data_buffer;
	uint bmp_data_size = H_image*V_image * 3;
	Byte *bmp_data_buffer,*tmp_dst,*tmp_src;
	uint dst_bytes_per_line = H_image * 3;
	uint src_bytes_per_line = H_round * 3;
	
	bmp_data_buffer = (Byte *)malloc(bmp_data_size*sizeof(Byte));
	tmp_dst = bmp_data_buffer;
	tmp_src = out_buffer;
	for (int v = 0; v < V_image; v++)
	{
		memcpy(tmp_dst, tmp_src, dst_bytes_per_line);
		tmp_dst += dst_bytes_per_line;
		tmp_src += src_bytes_per_line;
	}
	//uint length = sizeof(bmp_data_buffer);

	FILE *bmp_fp;
	pixel_RGB pixel;
	Byte zero_filling_bytes;
	
	//写成bmp文件
	errno_t err;
	if ((err = fopen_s(&bmp_fp, BMP_path, "wb")) != 0)
	{
		printf("不能正常打开用于输出的bmp文件！");
		fclose(bmp_fp);
		system("pause");
		return err;
	}
	
	if (H_image % 4 != 0)
		zero_filling_bytes = 4 - ((H_image * 3) % 4);
	else
		zero_filling_bytes = 0;
	
	//bmp头文件信息设置
	bmp_header_buffer.Size = 54 + V_image*(H_image * 3 + zero_filling_bytes);
	bmp_header_buffer.Width = H_image;
	bmp_header_buffer.Height = V_image;

	//先输出头文件信息
	fwrite(&bmp_header_buffer.Type, 2, 1, bmp_fp);
	fwrite(&bmp_header_buffer.Size, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.zero_Reserved, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.Offbits, 4, 1, bmp_fp);

	fwrite(&bmp_header_buffer.biSize, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.Width, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.Height, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.Planes, 2, 1, bmp_fp);
	fwrite(&bmp_header_buffer.BitCount, 2, 1, bmp_fp);
	fwrite(&bmp_header_buffer.Compression, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.biImageSize, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.biXPelsPerMeter, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.biYPelsPerMeter, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.biClrUsed, 4, 1, bmp_fp);
	fwrite(&bmp_header_buffer.biClrImportant, 4, 1, bmp_fp);

	//输出图像数据

	uint pixel_loc = dst_bytes_per_line*(V_image - 1);
	Byte zero_byte = 0;
	for (int v = 0; v < V_image; v++)
	{
		for (int h = 0; h < H_image; h++)
		{
			
			pixel.B = bmp_data_buffer[pixel_loc++];
			pixel.G = bmp_data_buffer[pixel_loc++];
			pixel.R = bmp_data_buffer[pixel_loc++];
			fwrite(&pixel.B, 1, 1, bmp_fp);
			fwrite(&pixel.G, 1, 1, bmp_fp);
			fwrite(&pixel.R, 1, 1, bmp_fp);
		}
		for (int i = 0; i < zero_filling_bytes; i++)
			fwrite(&zero_byte, 1, 1, bmp_fp);
		pixel_loc -= X_image_bytes * 2;
	}


	fclose(bmp_fp);
	free(bmp_data_buffer);
}

//////////////////////////////////////////////////////////////////////////////////////////
////待定函数end
//////////////////////////////////////////////////////////////////////////////////////////

long getfilesize(FILE *fp)
{
	long filesize;
	fseek(fp, 0, SEEK_END);
	filesize = ftell(fp);
	return filesize;
}
/*//测试IDCT
int main()
{
	STByte input[64] = { 1125, -32, -185, -7, 2, -1, -2, 2, -22, -16, 45, -3, -2, 0, -2, -2, -165, 32, 17, 2, 1, -1, -3, 0, -7, -4, 0, 2, 2, -1, -1, 2, -2, 0, 0, 3, 0, 0, 2, 1, 3, 1, 1, -1, -2, 0, 2, 0, 0, 0, 2, -1, -1, 2, 1, -1, 0, 3, 1, -1, 2, 1, -2, 0 };
	Byte output[64];
	IDCT(input, output);
	printf("\n输入数组：\n");
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			printf("%d ", input[i * 8 + j]);
		}
		printf("\n");
	}
	printf("\n输出数组：\n");
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			printf("%d ", output[i * 8 + j]);
		}
		printf("\n");
	}
	system("pause");
	return 0;
}
*/


int main()
{
	char filename[] = "../data/input.jpg";
	char BMP_path[] = "../data/output.bmp";
	FILE *fp;
	errno_t err;


	//读入文件大小
	if ((err = fopen_s(&fp, filename, "rb")) != 0)
	{
		printf("输入的jpg文件不能正常打开！");
		fclose(fp);
		system("pause");
		return err;
	}
	long JPG_length = getfilesize(fp);
	fclose(fp);
	
	////读入jpg文件，存入缓存中
	if ((err = fopen_s(&fp, filename, "rb")) != 0)
	{
		printf("输入的jpg文件不能正常打开！");
		fclose(fp);
		system("pause");
		return err;
	}
	in_buffer = (Byte *)malloc(JPG_length);
	long read_result = fread_s(in_buffer, JPG_length, 1, JPG_length, fp);
	fclose(fp);
	if (read_result< JPG_length)
	{
		printf("读取文件的时候出现了问题！\n");
		system("pause");
		return 0;
	}
	
	read_JPG_Header(in_buffer);
	restart();
	decode_JPEG_image();
	free(HT_DC[0].Weight);
	free(HT_DC[1].Weight);
	free(HT_AC[0].Weight);
	free(HT_AC[1].Weight);
	free(in_buffer);

	write_to_BMP(BMP_path);
	
	fclose(fp);
	
	free(out_buffer);
	return 0;
}