// --------------------------------------------------------------------
// Copyright (c) 2005 by Terasic Technologies Inc. 
// --------------------------------------------------------------------
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// --------------------------------------------------------------------
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
//
// Major Functions:	VGA experments
// modifed by BRL4 for SRAM; Displays a color grid 
// VGA controller, PLL, and reset are from DE2 distribution CD
// --------------------------------------------------------------------

module DE2_Default
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	    TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_27;				//	27 MHz
input			CLOCK_50;				//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output	[17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output	[11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output	[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output	[17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			OTG_INT0;				//	ISP1362 Interrupt 0
input			OTG_INT1;				//	ISP1362 Interrupt 1
input			OTG_DREQ0;				//	ISP1362 DMA Request 0
input			OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
inout		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
output/*inout*/	AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1

//	LCD ON
assign	LCD_ON		=	1'b0;
assign	LCD_BLON	=	1'b0;

//	All inout port turn to tri-state
assign	DRAM_DQ		=	16'hzzzz;
assign	FL_DQ		=	8'hzz;
assign	SRAM_DQ		=	16'hzzzz;
assign	OTG_DATA	=	16'hzzzz;
assign	SD_DAT		=	1'bz;
assign	ENET_DATA	=	16'hzzzz;
assign	GPIO_0		=	36'hzzzzzzzzz;
assign	GPIO_1		=	36'hzzzzzzzzz;

wire [31:0]	mSEG7_DIG;
reg	 [31:0]	Cont;
wire		VGA_CTRL_CLK;
wire		AUD_CTRL_CLK;
wire [9:0]	mVGA_R;
wire [9:0]	mVGA_G;
wire [9:0]	mVGA_B;
wire [19:0]	mVGA_ADDR;			//video memory address
wire [9:0]  Coord_X, Coord_Y;	//display coods
wire		DLY_RST;


//always@(posedge CLOCK_50 or negedge KEY[0])
//begin
//	if(!KEY[0])
//	Cont	<=	0;
//	else
//	Cont	<=	Cont+1;
//end

assign	TD_RESET	=	1'b1;	//	Allow 27 MHz input
assign	AUD_ADCLRCK	=	AUD_DACLRCK;
assign	AUD_XCK		=	AUD_CTRL_CLK;

//assign	LEDR[17:1] =	17'h0;
//assign	LEDG =	8'h0;

//assign mVGA_G = (mVGA_ADDR<20'h0ffff)? 10'd1000:0;
//assign mVGA_R = 10'd0000;
//assign mVGA_B = 10'd0000;	

Reset_Delay			r0	(	.iCLK(CLOCK_50),.oRESET(DLY_RST)	);

VGA_Audio_PLL 		p1	(	.areset(~DLY_RST),.inclk0(CLOCK_27),.c0(VGA_CTRL_CLK),.c1(AUD_CTRL_CLK),.c2(VGA_CLK)	);

PS2_KEYBOARD keyboard(.ps2_dat(PS2_DAT),
	.ps2_clk(PS2_CLK),
	.sys_clk(MYCLOCK[23]),
	.reset(1),
	.reset1(1),
	.key1_code(keyOut));
	
wire [7:0] keyOut;
assign LEDG= keyOut;

VGA_Controller		u1	(	//	Host Side
							.iCursor_RGB_EN(4'b0111),
							.oAddress(mVGA_ADDR),
							.oCoord_X(Coord_X),
							.oCoord_Y(Coord_Y),
							.iRed(mVGA_R),
							.iGreen(mVGA_G),
							.iBlue(mVGA_B),
							//	VGA Side
							.oVGA_R(VGA_R),
							.oVGA_G(VGA_G),
							.oVGA_B(VGA_B),
							.oVGA_H_SYNC(VGA_HS),
							.oVGA_V_SYNC(VGA_VS),
							.oVGA_SYNC(VGA_SYNC),
							.oVGA_BLANK(VGA_BLANK),
							//	Control Signal
							.iCLK(VGA_CTRL_CLK),
							.iRST_N(DLY_RST)	);


//Down sample to 512x512 and use coordinates to get memory address
assign SRAM_ADDR = {Coord_X[9:1],Coord_Y[9:1]} ;		// [17:0]
//assign SRAM_ADDR = mVGA_ADDR[19:2];
assign SRAM_UB_N = 0;					// hi byte select enabled
assign SRAM_LB_N = 0;					// lo byte select enabled
assign SRAM_CE_N = 0;					// chip is enabled
//assign SRAM_WE_N = 1;					// write when ZERO
assign SRAM_OE_N = 0;					//output enable is overidden by WE
// If KEY1 is not pressed, then float bus, so that SRAM can drive it (READ)
assign SRAM_WE_N = (KEY[1]? 1'b1 : 1'b0);
// If KEY1 is pressed, drive it with data from SWITCHES[15:0] to be stored in SRAM (WRITE)
//assign SRAM_DQ = (KEY[1]? 16'hzzzz : SW[15:0]);
/*assign SRAM_DQ = (KEY[1]? 16'hzzzz : 
			(KEY[2]?	//if key2 is up
			{Coord_X[8:5], 						//red
			Coord_Y[8:5] & ~{4{Coord_X[9]}},  	//green
			Coord_Y[8:5] & {4{Coord_X[9]}}, 	//blue
			 4'b0}								//not used 
			: SW[15:0])); //else write the switches
*/
reg [15:0] data;
assign SRAM_DQ = data;
integer coord;
integer Type, myrand, preType;
reg [1:0]Flip;
integer MYCLOCK;
integer X,Y, lowY, rightX;
reg comedown;
//wire inshape;
reg inshape;
reg pauseF;
//*****color
reg [15:0] color[6:0];
initial
begin
	color[0]='hFC00; // Yellow
	color[1]='h07E0; // Blue
	color[2]='hFBE0; // Pink 
	color[3]='hF0F0; // Magenta
	color[4]='d4080; // light blue
	color[5]='hB0F0; // Purple
	color[6]='hF800; // Orange
	
	pauseF<=0;
	coord = 200;
	X = 500;
	Y=20;
	Type=0;
	myrand = 132;
	preType=myrand%7;
	comedown =0;
end
wire Yt,Xt;
assign Yt=Y;
assign Xt=X;
wire [3:0]shape [6:0];
wire inNextShape[6:0];


//shapes
assign shape[0][0]=(Coord_X<X+40 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+40);
assign shape[0][1]=(Coord_X<X+40 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+40);
assign shape[0][2]=(Coord_X<X+40 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+40);
assign shape[0][3]=(Coord_X<X+40 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+40);
//****1
assign shape[1][0]=(Coord_X<X+80 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20 );
assign shape[1][1]=(Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+80 );
assign shape[1][2]=(Coord_X<X+80 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20 );
assign shape[1][3]=(Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+80 );
//****2
assign shape[2][0]=((Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+60 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ) );
assign shape[2][1]=((Coord_X<X+20&& Coord_X>X && Coord_Y>Y && Coord_Y<Y+60) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y && Coord_Y<Y+20 ) );
assign shape[2][2]=((Coord_X<X+60 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+60 && Coord_X>X+40 && Coord_Y>=Y+20 && Coord_Y<Y+40 ) );
assign shape[2][3]=((Coord_X<X+40 && Coord_X>X+20 && Coord_Y>Y && Coord_Y<Y+60) || (Coord_X<X+40 && Coord_X>X && Coord_Y>=Y+40 && Coord_Y<Y+60 ) );
//***3 ** ta inja check shod
assign shape[3][0]=((Coord_X<X+60 && Coord_X>X+40 && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+60 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ) );
assign shape[3][1]=((Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+60) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y+40 && Coord_Y<Y+60 ) );
assign shape[3][2]=((Coord_X<X+60 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+20 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ) );
assign shape[3][3]=((Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y && Coord_Y<Y+60 ) );
//***4
assign shape[4][0]=((Coord_X<X+40 && Coord_X>X+20 && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+60 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[4][1]=((Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+60) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[4][2]=((Coord_X<X+60 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[4][3]=((Coord_X<X+40 && Coord_X>X+20 && Coord_Y>Y && Coord_Y<Y+60) || (Coord_X<X+20 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
//***5
assign shape[5][0]=((Coord_X<X+40 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+60 && Coord_X>X+20 && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[5][1]=((Coord_X<X+20 && Coord_X>X && Coord_Y>Y+20 && Coord_Y<Y+60) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y && Coord_Y<Y+40 ));
assign shape[5][2]=((Coord_X<X+40 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+60 && Coord_X>X+20 && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[5][3]=((Coord_X<X+20 && Coord_X>X && Coord_Y>Y+20 && Coord_Y<Y+60) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y && Coord_Y<Y+40 ));
//***6
assign shape[6][0]= ((Coord_X<X+60 && Coord_X>X+20 && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+40 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[6][1]= ((Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+40) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y+20 && Coord_Y<Y+60 ));
assign shape[6][2]= ((Coord_X<X+60 && Coord_X>X+20 && Coord_Y>Y && Coord_Y<Y+20) || (Coord_X<X+40 && Coord_X>X && Coord_Y>=Y+20 && Coord_Y<Y+40 ));
assign shape[6][3]= ((Coord_X<X+20 && Coord_X>X && Coord_Y>Y && Coord_Y<Y+40) || (Coord_X<X+40 && Coord_X>X+20 && Coord_Y>=Y+20 && Coord_Y<Y+60 ));
//end shapes


// **** pause
always @(posedge CLOCK_50)
begin
	//************pause
	if(keyOut=='b01001101 )
	pauseF<=1;
	else if(keyOut=='b00011011)
	pauseF<=0;
	if(pauseF==0)
	MYCLOCK = MYCLOCK + 1;
end
// *****

//assign LEDR[0] = MYCLOCK[24];
assign LEDR = X[17:0];

//*****next shape
assign inNextShape[0]=(Coord_X<260 && Coord_X>220 && Coord_Y>80 && Coord_Y<120);
assign inNextShape[1]=(Coord_X<280 && Coord_X>200 && Coord_Y>80 && Coord_Y<100 );
assign inNextShape[2]=((Coord_X<220 && Coord_X>200 && Coord_Y>80 && Coord_Y<100) || (Coord_X<260 && Coord_X>200 && Coord_Y>=100 && Coord_Y<120 ) );
assign inNextShape[3]=((Coord_X<260 && Coord_X>240 && Coord_Y>80 && Coord_Y<100) || (Coord_X<260 && Coord_X>200 && Coord_Y>=100 && Coord_Y<120 ) );
assign inNextShape[4]=((Coord_X<240 && Coord_X>220 && Coord_Y>80 && Coord_Y<100) || (Coord_X<260 && Coord_X>200 && Coord_Y>=100 && Coord_Y<120 ));
assign inNextShape[5]=((Coord_X<240 && Coord_X>200 && Coord_Y>80&& Coord_Y<100) || (Coord_X<260 && Coord_X>220 && Coord_Y>=100 && Coord_Y<120 ));
assign inNextShape[6]= ((Coord_X<260 && Coord_X>220 && Coord_Y>80 && Coord_Y<100) || (Coord_X<240 && Coord_X>200 && Coord_Y>=100 && Coord_Y<120 ));
//


//***** draw
always @(Coord_X , Coord_Y) 
begin
	if(Coord_X%20==0 || Coord_Y%20==0)
		data = 'b1111111110111111;
	else if( shape[Type][Flip] )
		data=color[Type];
	else if(Coord_X>=180 && Coord_X<=300 && Coord_Y>=60 && Coord_Y<=160)
	begin
		if(inNextShape[preType])
			data=color[preType];
		else 
			data= 'h0000;
	end
	else if (Coord_X <= 400 || Coord_X >= 600 || Coord_Y<20 || Coord_Y>420)begin
		data = 'hBBF0;
	end
	else
		data = 'h0000;
end
//****


//**** flip 
integer FlowY, FrightX;
always @(posedge MYCLOCK[22])
begin
	if (Y==20)
		Flip=0;
	if (keyOut == 'b01110101 )begin
		case(Type)
			   0: FlowY = Y+40;
			   1: FlowY = (Flip==1 || Flip==3) ? Y+20 : Y+80;
			   2: FlowY = (Flip==1 || Flip==3) ? Y+40 : Y+60;
			   3: FlowY = (Flip==1 || Flip==3) ? Y+40 : Y+60;
			   4: FlowY = (Flip==1 || Flip==3) ? Y+40 : Y+60;
			   5: FlowY = (Flip==1 || Flip==3) ? Y+40 : Y+60;
			   6: FlowY = (Flip==1 || Flip==3) ? Y+40 : Y+60;
		endcase
		case(Type)
			   0: FrightX = X+40;
			   1: FrightX = (Flip==1 || Flip==3) ? X+80 : X+20;
			   2: FrightX = (Flip==1 || Flip==3) ? X+60 : X+40;
			   3: FrightX = (Flip==1 || Flip==3) ? X+60 : X+40;
			   4: FrightX = (Flip==1 || Flip==3) ? X+60 : X+40;
			   5: FrightX = (Flip==1 || Flip==3) ? X+60 : X+40;
			   6: FrightX = (Flip==1 || Flip==3) ? X+60 : X+40;
		endcase
		if ( X>=400 && FrightX<=600 && Y>=20 && FlowY<=420 )
			Flip = (Flip+1);
	end
end
//*******

// **** set X
always @(posedge MYCLOCK[21])
begin
	if(SW[0] || Y==20)
	begin
		X = 500;
	end
	else if(keyOut == 'b01101011 && 400<X)//left
		X = X-20;
	else if(keyOut == 'b01110100 )begin //right
		case(Type)
			0: rightX = X+40;
			1: rightX = (Flip==0 || Flip==2) ? X+80 : X+20;
			2: rightX = (Flip==0 || Flip==2) ? X+60 : X+40;
			3: rightX = (Flip==0 || Flip==2) ? X+60 : X+40;
			4: rightX = (Flip==0 || Flip==2) ? X+60 : X+40;
			5: rightX = (Flip==0 || Flip==2) ? X+60 : X+40;
			6: rightX = (Flip==0 || Flip==2) ? X+60 : X+40;
		endcase
		if ( rightX<600 )
			X = X+20;
	end
end
// *****


// **** set Y
always @(posedge MYCLOCK[21])
begin
	myrand = (myrand*23)+7;
	case(Type)
       0: lowY = Y+40;
       1: lowY = (Flip==0 || Flip==2) ? Y+20 : Y+80;
       2: lowY = (Flip==0 || Flip==2) ? Y+40 : Y+60;
       3: lowY = (Flip==0 || Flip==2) ? Y+40 : Y+60;
       4: lowY = (Flip==0 || Flip==2) ? Y+40 : Y+60;
       5: lowY = (Flip==0 || Flip==2) ? Y+40 : Y+60;
       6: lowY = (Flip==0 || Flip==2) ? Y+40 : Y+60;
	endcase
	
	if(SW[0])
		Y <= 20;
	else if(keyOut=='b00101101)
	begin
		Y<=20;
		preType<={myrand}%7;
		myrand = (myrand*37)%7;
		Type<=myrand;
	end
	else if (lowY>=420)begin
		Y=20;
		preType <= {myrand}%7;
		Type<=preType;
	end
	else if( keyOut=='b01110010 )
		begin
			Y<=Y+20;
		end
	else if ( MYCLOCK[24] && comedown == 1 )
		begin
		Y=Y+20;
		comedown=0;
		end
	else if ( MYCLOCK[24] == 0 )
		comedown=1;
end
// *****

// Show memory on the LEDs and 7-seg display
assign  mVGA_R = {SRAM_DQ[15:12], 6'b0} ;
assign  mVGA_G = {SRAM_DQ[11:8], 6'b0} ;
assign  mVGA_B = {SRAM_DQ[7:4], 6'b0} ;
			
endmodule //top module