`timescale 1ns / 1ps


module D_ff_IM(input clk, input reset, input d, output reg q);
	always@(reset or negedge clk)
	if(reset)
		q=d;
endmodule

module register_IM(input clk, input reset, input [15:0] d_in, output [15:0] q_out);
	D_ff_IM dIM0 (clk, reset, d_in[0], q_out[0]);
	D_ff_IM dIM1 (clk, reset, d_in[1], q_out[1]);
	D_ff_IM dIM2 (clk, reset, d_in[2], q_out[2]);
	D_ff_IM dIM3 (clk, reset, d_in[3], q_out[3]);
	D_ff_IM dIM4 (clk, reset, d_in[4], q_out[4]);
	D_ff_IM dIM5 (clk, reset, d_in[5], q_out[5]);
	D_ff_IM dIM6 (clk, reset, d_in[6], q_out[6]);
	D_ff_IM dIM7 (clk, reset, d_in[7], q_out[7]);
	D_ff_IM dIM8 (clk, reset, d_in[8], q_out[8]);
	D_ff_IM dIM9 (clk, reset, d_in[9], q_out[9]);
	D_ff_IM dIM10 (clk, reset, d_in[10], q_out[10]);
	D_ff_IM dIM11 (clk, reset, d_in[11], q_out[11]);
	D_ff_IM dIM12 (clk, reset, d_in[12], q_out[12]);
	D_ff_IM dIM13 (clk, reset, d_in[13], q_out[13]);
	D_ff_IM dIM14 (clk, reset, d_in[14], q_out[14]);
	D_ff_IM dIM15 (clk, reset, d_in[15], q_out[15]);
endmodule

module mux16to1( input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15, input [3:0] Sel, output reg [15:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or outR8 or outR9 or outR10 or outR11 or outR12 or outR13 or outR14 or outR15 or Sel)
	case (Sel)
				4'b0000: outBus=outR0;
				4'b0001: outBus=outR1;
				4'b0010: outBus=outR2;
				4'b0011: outBus=outR3;
				4'b0100: outBus=outR4;
				4'b0101: outBus=outR5;
				4'b0110: outBus=outR6;
				4'b0111: outBus=outR7;
				4'b1000: outBus=outR8;
				4'b1001: outBus=outR9;
				4'b1010: outBus=outR10;
				4'b1011: outBus=outR11;
				4'b1100: outBus=outR12;
				4'b1101: outBus=outR13;
				4'b1110: outBus=outR14;
				4'b1111: outBus=outR15;
	endcase
endmodule

module IM(  input clk, input reset, input [4:0] pc_5bits, output [15:0] IR );
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15;
	register_IM rIM0 (clk, reset, 16'b0001_001_010_000_000,Qout0); 		//sub $r0, $r1, $r2
	register_IM rIM1 (clk, reset, 16'b0011_111_011_000_011, Qout1); 	//addic $r3, $r7, 3
	register_IM rIM2 (clk, reset,  16'b0001_010_011_001_000, Qout2); 	//sub $r1, $r2, $r3
	register_IM rIM3 (clk, reset, 16'b0001_011_100_010_000, Qout3); 	//sub $r2, $r3, $r4
	register_IM rIM4 (clk, reset, 16'b0011_000_010_000_010, Qout4); 	//addic $r2, $r0, 2
	register_IM rIM5 (clk, reset,  16'b0000_000_000_000_000, Qout5); 	//add $r0, $r0, $r0
	register_IM rIM6 (clk, reset,16'b0000_001_011_000_000, Qout6); 	//add $r0, $r1, $r3
	register_IM rIM7 (clk, reset, 16'b0000_000_000_000_000, Qout7);		//add $r0, $r0, $r0
	register_IM rIM8 (clk, reset, 16'b0101_010_011_000_000, Qout8); 	//mul $r2 ,$r3
	register_IM rIM9 (clk, reset, 16'b0100_010_100_000_001, Qout9); 		//shift $r4, $r2, 1
	register_IM rIM10 (clk, reset, 16'b0010_111_011_111_000, Qout10); 	//addc $r7, $r7, $r3
	register_IM rIM11 (clk, reset, 16'b0000_011_010_101_000, Qout11); 	//add $r5, $r3, $r2
	register_IM rIM12 (clk, reset, 16'b0111_000_000_110_000, Qout12); //mflo $r6
	register_IM rIM13 (clk, reset, 16'b0110_000_000_000_000, Qout13); // mfhi $r0
	register_IM rIM14 (clk, reset, 16'b0001_011_010_001_000, Qout14); 	// sub $r1, $r3,$2
	register_IM rIM15 (clk, reset, 16'b0000_000_000_000_000, Qout15); 	
	mux16to1 mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc_5bits[4:1],IR);
endmodule
//Instruction Memory Design Ends

//Register File Design Starts
module D_ff_reg (input clk, input reset, input regWrite, input decOut1b, input d, output reg q);
	always @ (negedge clk)
		begin
			if(reset==1)
				q=1;
			else
				if(regWrite == 1 && decOut1b==1)
					begin
						q=d;
					end
		end
endmodule


module register16bit_RS( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	D_ff_reg d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff_reg d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff_reg d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff_reg d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff_reg d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff_reg d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff_reg d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff_reg d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff_reg d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff_reg d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff_reg d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff_reg d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff_reg d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff_reg d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff_reg d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
endmodule

module registerSet( input clk, input reset, input regWrite, input [7:0] decOut, input [15:0] writeData,
output [15:0] outR0, output [15:0] outR1, output [15:0] outR2, output [15:0] outR3,
	output [15:0] outR4,output [15:0] outR5, output [15:0] outR6, output [15:0] outR7);

	register16bit_RS rs0( clk, reset, regWrite, decOut[0], writeData, outR0 );
	register16bit_RS rs1( clk, reset, regWrite, decOut[1], writeData, outR1 );
	register16bit_RS rs2( clk, reset, regWrite, decOut[2], writeData, outR2 );
	register16bit_RS rs3( clk, reset, regWrite, decOut[3], writeData, outR3 );
	
	register16bit_RS rs4( clk, reset, regWrite, decOut[4], writeData, outR4 );
	register16bit_RS rs5( clk, reset, regWrite, decOut[5], writeData, outR5 );
	register16bit_RS rs6( clk, reset, regWrite, decOut[6], writeData, outR6 );
	register16bit_RS rs7( clk, reset, regWrite, decOut[7], writeData, outR7 );
	
endmodule

module decoder( input [2:0] destReg, output reg [7:0] decOut);
always @(destReg)
	case(destReg)
	3'd0: decOut=8'b0000_0001;
	3'd1: decOut=8'b0000_0010;
	3'd2: decOut=8'b0000_0100;
	3'd3: decOut=8'b0000_1000;
	3'd4: decOut=8'b0001_0000;
	3'd5: decOut=8'b0010_0000;
	3'd6: decOut=8'b0100_0000;
	3'd7: decOut=8'b1000_0000;
	endcase
endmodule

module mux8to1( input [15:0] outR0, input [15:0] outR1, input [15:0] outR2, input [15:0] outR3, input [15:0] outR4, input [15:0] outR5, 
input [15:0] outR6, input [15:0] outR7, input [2:0] Sel, output reg [15:0] outBus );

always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,Sel)
	case(Sel)
		3'd0:outBus=outR0;
		3'd1:outBus=outR1;
		3'd2:outBus=outR2;
		3'd3:outBus=outR3;
		3'd4:outBus=outR4;
		3'd5:outBus=outR5;
		3'd6:outBus=outR6;
		3'd7:outBus=outR7;
	endcase
endmodule

module registerFile(input clk, input reset, input regWrite, input [2:0] rs, input [2:0] rt,input [2:0] rd, 
input [15:0] writeData, output [15:0] outR0, output [15:0] outR1);
	
	wire [7:0] decOut;
	wire [15:0] outR00,outR11, outR2, outR3, outR4, outR5, outR6, outR7;
	decoder dec(rd,decOut);
	registerSet rSet( clk, reset, regWrite,decOut, writeData,outR00,outR11, outR2, outR3, outR4, outR5, outR6, outR7);
	mux8to1 mux8_1_1( outR00, outR11,outR2,outR3,outR4, outR5, outR6,outR7, rs, outR0 );
	mux8to1 mux8_1_2( outR00, outR11,outR2,outR3,outR4, outR5, outR6,outR7, rt, outR1 );
endmodule
//Register File Design Ends

module adder(input [15:0] in1, input [15:0] in2, output reg [15:0] adder_out);
	always@(in1 or in2)
		adder_out = in1 +in2;
endmodule

module signExt6to16( input [5:0] offset, output reg [15:0] signExtOffset);
	always@(offset)
	begin
			signExtOffset={{10{offset[5]}},offset[5:0]};
	end
endmodule

module zeroExt4to16( input [3:0] offset, output reg [15:0] zeroExtOffset);
	always@(offset)
			zeroExtOffset={{12{1'b0}},offset[3:0]};
endmodule

module zeroExt1to16( input cFlag, output reg [15:0] zeroExtcFlag);
	always@(cFlag)
		zeroExtcFlag={{15{1'b0}},cFlag};
endmodule

module mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3, input [15:0] in4, input [1:0] sel, output reg [15:0] muxOut);
always@(in1 or in2 or in3 or in4 or sel)
		case(sel)
		2'b00:muxOut=in1;
		2'b01:muxOut=in2;
		2'b10:muxOut=in3;
		2'b11:muxOut=in4;
		endcase
endmodule


module mux2to1_3bits(input [2:0] in1, input [2:0] in2, input sel, output reg [2:0] muxout);
	 always@(in1 or in2 or sel)
	 begin
		case(sel)
			0 : muxout = in1;
			1 : muxout = in2;
		endcase
	 end
endmodule

module alu(input [15:0] aluIn1, input [15:0] aluIn2,input [1:0] aluOp,output reg carry, output reg [15:0] aluOut1,output reg [15:0] aluOut2);
	always@(aluIn1 or aluIn2 or aluOp)
	begin
		case(aluOp)
			2'b00: {carry,aluOut1}=aluIn1 + aluIn2;
			2'b01: {carry,aluOut1}=aluIn1 - aluIn2;
			2'b10: {carry,aluOut1}=aluIn1 << aluIn2;
			2'b11: {aluOut2,aluOut1}=aluIn1 * aluIn2; 
		endcase
	end
endmodule

module D_ff(input clk, input reset, input regWrite,input d, output reg q);
	always@(negedge clk)
		begin
			if(reset)
				q=0;
			else
				if(regWrite == 1) begin q=d; end
		end
endmodule

//PC and pipeline registers uses register1bit, register2bit, register3bit and register16bit modules if required
module register1bit( input clk, input reset, input regWrite, input writeData, output outR );
	D_ff d0(clk, reset, regWrite, writeData, outR);
endmodule

module register2bit( input clk, input reset, input regWrite,input [1:0] writeData, output  [1:0] outR );
	D_ff d0(clk, reset, regWrite, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, writeData[1], outR[1]);
endmodule

module register3bit( input clk, input reset, input regWrite, input [2:0] writeData, output  [2:0] outR );
	D_ff d0(clk, reset, regWrite, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, writeData[2], outR[2]);
endmodule

module register16bit( input clk, input reset, input regWrite, input [15:0] writeData, output  [15:0] outR );
	D_ff d0(clk, reset, regWrite, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, writeData[15], outR[15]);
endmodule

module ctrlCkt(input [3:0] opcode,output reg [1:0] aluSrcB,output reg [1:0] aluOp,output reg regDest,
output reg hiWr,output reg loWr,output reg [1:0] toReg,output reg regWr);	

	always@(opcode)
	begin	
		case(opcode)
		4'd0: begin
				aluSrcB = 2'b00;
				aluOp = 2'b00;
				regDest = 1'b1;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b10;
				regWr = 1'b1;
			  end
		4'd1: begin
				aluSrcB = 2'b00;
				aluOp = 2'b01;
				regDest = 1'b1;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b10;
				regWr = 1'b1;
			  end
		4'd2: begin
				aluSrcB = 2'b00;
				aluOp = 2'b00;
				regDest = 1'b1;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b11;
				regWr = 1'b1;
			  end
		4'd3: begin
				aluSrcB = 2'b01;
				aluOp = 2'b00;
				regDest = 1'b0;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b11;
				regWr = 1'b1;
			  end
		4'd4: begin
				aluSrcB = 2'b10;
				aluOp = 2'b10;
				regDest = 1'b0;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b10;
				regWr = 1'b1;
			  end	 
		4'd5: begin
				aluSrcB = 2'b00;
				aluOp = 2'b11;
				regDest = 1'b0;
				hiWr = 1'b1;
				loWr = 1'b1;
				toReg = 2'b11;
				regWr = 1'b0;
			  end		
		4'd6: begin
				aluSrcB = 2'b00;
				aluOp = 2'b00;
				regDest = 1'b1;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b00;
				regWr = 1'b1;
			  end
		4'd7: begin
				aluSrcB = 2'b00;
				aluOp = 2'b00;
				regDest = 1'b1;
				hiWr = 1'b0;
				loWr = 1'b0;
				toReg = 2'b01;
				regWr = 1'b1;
			  end
			  
		endcase
	end
	
endmodule

//Module uses register16bit
module IF_ID(input clk, input reset,input regWrite,input [15:0] instr, output [15:0] p0_intr);

register16bit regIF_ID_0( clk ,reset, regWrite, instr, p0_intr );

endmodule

//Module uses register1bit,register2bit,register3bit and register16bit
module ID_EX1(input clk, input reset,input regWrite,input [15:0] regOut1,input [15:0] regOut2,input [15:0] sExtOut,input [15:0] zExtOut,
input [2:0] inst_Rt,input [2:0] inst_Rd,input [1:0] ctrl_aluSrcB,input [1:0] ctrl_aluOp,input ctrl_regDest,
input ctrl_hiWr,input ctrl_loWr,input [1:0] ctrl_toReg,input ctrl_regWr,output [15:0] p1_regOut1,
output [15:0] p1_regOut2,output [15:0] p1_sExtOut,output  [15:0] p1_zExtOut,
output  [2:0] p1_inst_Rt,output [2:0] p1_inst_Rd,output  [1:0] p1_aluSrcB,output  [1:0] p1_aluOp,output  p1_regDest,
output  p1_hiWr,output  p1_loWr,output [1:0] p1_toReg,output  p1_regWr);

register16bit regID_EX1_0(  clk, reset, regWrite, regOut1, p1_regOut1  );
register16bit regID_EX1_1(  clk, reset, regWrite, regOut2, p1_regOut2 );
register16bit regID_EX1_2(  clk, reset, regWrite, sExtOut, p1_sExtOut );
register16bit regID_EX1_3(  clk, reset, regWrite, zExtOut, p1_zExtOut );

register3bit regID_EX1_4(  clk,  reset, regWrite, inst_Rt, p1_inst_Rt );
register3bit regID_EX1_5(  clk,  reset, regWrite, inst_Rd, p1_inst_Rd );

register2bit regID_EX1_6(  clk,  reset, regWrite, ctrl_toReg	, p1_toReg );
register2bit regID_EX1_7(  clk,  reset, regWrite, ctrl_aluSrcB	, p1_aluSrcB );
register2bit regID_EX1_8(  clk,  reset, regWrite, ctrl_aluOp	, p1_aluOp );

register1bit regID_EX1_9 (  clk, reset,regWrite, ctrl_hiWr		, p1_hiWr  );
register1bit regID_EX1_10(  clk, reset,regWrite, ctrl_loWr		, p1_loWr );
register1bit regID_EX1_11(  clk, reset,regWrite, ctrl_regDest	, p1_regDest );
register1bit regID_EX1_12(  clk, reset,regWrite, ctrl_regWr	, p1_regWr );
	
endmodule

//Module uses register1bit,register2bit,register3bit and register16bit
module EX1_EX2(input clk, input reset,input regWrite, input carryOut, input [15:0] loOut,input [15:0] hiOut,
   input [15:0] aluOut,input [1:0] p1_toReg,input  p1_regWr,input [2:0] destReg,output p2_carryOut, output [15:0] p2_loOut,output [15:0] p2_hiOut,
   output [15:0] p2_aluOut,output [1:0] p2_toReg,output p2_regWr,output [2:0] p2_destReg);

register16bit regEX1_EX2_0(  clk, reset, regWrite, loOut	, p2_loOut  );
register16bit regEX1_EX2_1(  clk, reset, regWrite, hiOut	, p2_hiOut );
register16bit regEX1_EX2_2(  clk, reset, regWrite, aluOut	, p2_aluOut );

register1bit regEX1_EX2_3 (  clk, reset, regWrite, p1_regWr	, p2_regWr  );
register1bit regEX1_EX2_4 (  clk, reset, regWrite, carryOut	, p2_carryOut );
 
register3bit regEX1_EX2_5 (  clk,  reset, regWrite, destReg, p2_destReg );
  
register2bit regEX1_EX2_6 (  clk,  reset, regWrite, p1_toReg, p2_toReg );

endmodule

//Module uses register1bit,register2bit,register3bit and register16bit
module EX2_WB(input clk, input reset,input regWrite,input [15:0] p2_aluOut,input [15:0] adderOut,input [15:0] p2_loOut,input [15:0] p2_hiOut,
input [2:0] p2_destReg,input [1:0] p2_toReg,input  p2_regWr,output [15:0] p3_aluOut,output [15:0] p3_adderOut,output [15:0] p3_loOut,output [15:0] p3_hiOut,
output [2:0] p3_destReg,output [1:0] p3_toReg,output p3_regWr);

register16bit regEX2_WB_0(  clk, reset, regWrite, p2_aluOut	, p3_aluOut  );
register16bit regEX2_WB_1(  clk, reset, regWrite, adderOut	, p3_adderOut );
register16bit regEX2_WB_2(  clk, reset, regWrite, p2_hiOut	, p3_hiOut );
register16bit regEX2_WB_3(  clk, reset, regWrite, p2_loOut	, p3_loOut );

register1bit regEX2_WB_4 (  clk, reset, regWrite, p2_regWr	, p3_regWr );
  
register3bit regEX2_WB_5 (  clk,  reset, regWrite, p2_destReg, p3_destReg );

register2bit regEX2_WB_6 (  clk,  reset, regWrite, p2_toReg, p3_toReg );

endmodule

//TopModule
module pipeline(input clk, input reset, output [15:0] Result );

wire [15:0] pc_out,IR,p0_intr,adder_out,adder_out1,outR0,outR1,signExtOffset,zeroExtcFlag,zeroExtOffset,p1_regOut1,p1_regOut2,p1_sExtOut,p1_zExtOut,muxOut,aluOut1,aluOut2,out_lo,out_hi,p2_loOut,p2_hiOut,p2_aluOut,p3_aluOut,p3_adderOut,p3_loOut,p3_hiOut;
wire [2:0] p3_destReg,p1_inst_Rt,p1_inst_Rd,muxout,p2_destReg;
wire [1:0] toReg,aluOp,aluSrcB,p1_aluSrcB,p1_aluOp,p1_toReg,p2_toReg,p3_toReg;
wire regDest,hiWr,loWr,regWr,p1_regDest,p1_hiWr,p1_loWr,p1_regWr,carry,p2_carryOut,p2_regWr,p3_regWr;

register16bit regPC_0( clk, reset, 1'b1,  adder_out, pc_out );
adder adder_0(16'd2, pc_out, adder_out);
IM IM_0( clk, reset, pc_out[4:0] , IR );

IF_ID IF_ID_0( clk, reset,1'b1, IR , p0_intr);

ctrlCkt ctrlCkt_0(p0_intr[15:12], aluSrcB, aluOp, regDest, hiWr,loWr, toReg, regWr);
registerFile registerFile_0(clk, reset, p3_regWr, p0_intr[11:9], p0_intr[8:6],p3_destReg, Result, outR0,  outR1);
signExt6to16 signExt6to16_0( p0_intr[5:0] , signExtOffset);
zeroExt4to16 zeroExt4to16_0( p0_intr[3:0] , zeroExtOffset);

ID_EX1 ID_EX1_0(clk,reset,1'b1,outR0,outR1,signExtOffset,zeroExtOffset,p0_intr[8:6],p0_intr[5:3],aluSrcB,aluOp,regDest,hiWr,loWr,toReg,regWr,p1_regOut1,p1_regOut2,p1_sExtOut,p1_zExtOut,
p1_inst_Rt,p1_inst_Rd,p1_aluSrcB,p1_aluOp,p1_regDest,p1_hiWr,p1_loWr,p1_toReg,p1_regWr);

mux4to1_16bits mux4to1_16bits_0(p1_regOut2,p1_sExtOut,p1_zExtOut,16'd0, p1_aluSrcB,muxOut);
alu alu_0(p1_regOut1, muxOut,p1_aluOp,carry, aluOut1,aluOut2);
mux2to1_3bits mux2to1_3bits_0(p1_inst_Rt,p1_inst_Rd, p1_regDest, muxout);
register16bit lo_0(  clk, reset, p1_loWr, aluOut1, out_lo );
register16bit hi_0(  clk, reset, p1_hiWr, aluOut2, out_hi );

EX1_EX2 EX1_EX2_0(clk,reset,1'b1,carry,out_lo,out_hi,aluOut1,p1_toReg,p1_regWr,muxout,p2_carryOut,p2_loOut,p2_hiOut,p2_aluOut,p2_toReg,p2_regWr,p2_destReg);

zeroExt1to16 zeroExt1to16_0( p2_carryOut,zeroExtcFlag);
adder adder_1(p2_aluOut, zeroExtcFlag,adder_out1);

EX2_WB EX2_WB_0(clk,reset,1'b1,p2_aluOut,adder_out1,p2_loOut,p2_hiOut,p2_destReg,p2_toReg,p2_regWr,p3_aluOut,p3_adderOut,p3_loOut,p3_hiOut,p3_destReg,p3_toReg,p3_regWr);

mux4to1_16bits mux4to1_16bits_1(p3_hiOut,p3_loOut,p3_aluOut,p3_adderOut,p3_toReg,Result);

endmodule

module pipelineTestBench;
	reg clk;
	reg reset;
	wire [15:0] Result;
	pipeline uut (.clk(clk), .reset(reset), .Result(Result));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#10  reset=0;	
		
		#190 $finish; 
	end
endmodule




