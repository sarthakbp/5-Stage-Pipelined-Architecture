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

module mux16to1( input [15:0] outR0,outR1,outR2,outR3,outR4,
					outR5,outR6,outR7,outR8,outR9,
					outR10,outR11,outR12,outR13,outR14,
					outR15, input [3:0] Sel, output reg [15:0] outBus );
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
	register_IM rIM0 (clk, reset, 16'b0001_001_010_001_000,Qout0); 		//sub $r1, $r1, $r2
	register_IM rIM1 (clk, reset, 16'b0010_010_011_001_000, Qout1); 	//addc $r1, $r2, $r3
	register_IM rIM2 (clk, reset,  16'b0000_001_011_010_000, Qout2); 	//add $r2, $r1, $r3
	register_IM rIM3 (clk, reset, 16'b0000_001_010_011_000, Qout3); 	//add $r3, $r1, $r2
	register_IM rIM4 (clk, reset, 16'b0011_010_100_000_001, Qout4); 	//shift $r4, $r2, 1
	register_IM rIM5 (clk, reset,  16'b0000_011_100_111_000, Qout5); 	//add $r7, $r3, $r4
	register_IM rIM6 (clk, reset,16'b0001_111_010_101_000, Qout6); 	//sub $r5, $r7, $r2
	register_IM rIM7 (clk, reset, 16'b0011_011_110_000_010, Qout7);		//shift $r6, $r3,2 
	register_IM rIM8 (clk, reset, 16'b0011_011_110_000_001, Qout8); 	//shift $r6 ,$r3,1
	register_IM rIM9 (clk, reset, 16'b0001_110_101_000_000, Qout9); 		//sub $r0, $r6,$r5
	register_IM rIM10 (clk, reset, 16'b0000_000_000_000_000, Qout10); 	
	register_IM rIM11 (clk, reset, 16'b0000_000_000_000_000, Qout11); 	
	register_IM rIM12 (clk, reset, 16'b0000_000_000_000_000, Qout12); 
	register_IM rIM13 (clk, reset, 16'b0000_000_000_000_000, Qout13); 
	register_IM rIM14 (clk, reset, 16'b0000_000_000_000_000, Qout14); 
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

module mux2to1_6bits(input [5:0] in1, input [5:0] in2, input sel, output reg [5:0] muxout);
	 always@(in1 or in2 or sel)
	 begin
		case(sel)
			0 : muxout = in1;
			1 : muxout = in2;
		endcase
	 end
endmodule

module mux2to1_16bits(input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxout);
	 always@(in1 or in2 or sel)
	 begin
		case(sel)
			0 : muxout = in1;
			1 : muxout = in2;
		endcase
	 end
endmodule


module alu(input [15:0] aluIn1, input [15:0] aluIn2,input [1:0] aluOp,output reg carry, output reg [15:0] aluOut1);
	always@(aluIn1 or aluIn2 or aluOp)
	begin
		case(aluOp)
			2'b00: {carry,aluOut1}=aluIn1 + aluIn2;
			2'b01: {carry,aluOut1}=aluIn1 - aluIn2;
			2'b10: {carry,aluOut1}=aluIn1 << aluIn2;
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
//PC and pipeline registers uses register1bit, register2bit, register3bit, register4bit and register16bit modules if required
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

module register4bit( input clk, input reset, input regWrite, input [3:0] writeData, output  [3:0] outR );
	D_ff d0(clk, reset, regWrite, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, writeData[3], outR[3]);
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

module ctrlCkt(input [3:0] opcode,output reg aluSrcB,output reg [1:0] aluOp,output reg regDest,
output reg toReg,output reg regWr);	
	always@(opcode)
	begin
		case(opcode)
		4'd0:
			begin
			aluSrcB=1'b0;
			aluOp=2'b00;
			regDest=1'b1;
			toReg=1'b1;
			regWr=1'b1;
			end
		4'd1:
			begin
			aluSrcB=1'b0;
			aluOp=2'b01;
			regDest=1'b1;
			toReg=1'b1;
			regWr=1'b1;
			end
		4'd2:
			begin
			aluSrcB=1'b0;
			aluOp=2'b00;
			regDest=1'b1;
			toReg=1'b0;
			regWr=1'b1;
			end
		4'd3:
			begin
			aluSrcB=1'b1;
			aluOp=2'b10;
			regDest=1'b0;
			toReg=1'b1;
			regWr=1'b1;
			end
	endcase
	end
endmodule

//Module uses register16bit
module IF_ID(input clk, input reset,input regWrite,input [15:0] instr, output [15:0] p0_intr);

   register16bit register16bit_0( clk, reset, regWrite, instr, p0_intr );
	
endmodule

//Module uses register1bit,register2bit,register3bit, register4bit and register16bit
module ID_EX1(input clk, input reset,input regWrite,input [15:0] regOut1,input [15:0] regOut2,input [15:0] zExtOut,
input [2:0] inst_Rs,input [2:0] inst_Rt,input [2:0] inst_Rd, input [3:0] inst_opcode,input ctrl_aluSrcB,input [1:0] ctrl_aluOp,input ctrl_regDest,
input ctrl_toReg,input ctrl_regWr,output [15:0] p1_regOut1,
output [15:0] p1_regOut2,output  [15:0] p1_zExtOut,output  [2:0] p1_inst_Rs,
output  [2:0] p1_inst_Rt,output [2:0] p1_inst_Rd,output [3:0] p1_opcode,output p1_aluSrcB,output  [1:0] p1_aluOp,output  p1_regDest,
output p1_toReg,output  p1_regWr);

	register16bit register16bit_1( clk, reset, regWrite, regOut1, p1_regOut1 );
	register16bit register16bit_2( clk, reset, regWrite, regOut2, p1_regOut2 );
	register16bit register16bit_3( clk, reset, regWrite, zExtOut, p1_zExtOut );
	
	register3bit register3bit_0(  clk,  reset, regWrite,inst_Rs,p1_inst_Rs );
	register3bit register3bit_1(  clk,  reset, regWrite,inst_Rt,p1_inst_Rt );
	register3bit register3bit_2(  clk,  reset, regWrite,inst_Rd,p1_inst_Rd );
	
	register4bit register4bit_0(  clk,  reset,  regWrite, inst_opcode, p1_opcode );
	
	register2bit register2bit_0(  clk,  reset, regWrite,ctrl_aluOp, p1_aluOp);
	
	register1bit register1bit_0(  clk, reset, regWrite, ctrl_regDest,p1_regDest);
	register1bit register1bit_1(  clk, reset, regWrite, ctrl_toReg,p1_toReg);
	register1bit register1bit_2(  clk, reset, regWrite, ctrl_regWr,p1_regWr);
	register1bit register1bit_3(  clk, reset, regWrite, ctrl_aluSrcB,p1_aluSrcB);
	
endmodule

//Module uses register1bit, register3bit and register16bit
module EX1_EX2(input clk, input reset,input regWrite, input carryOut, 
   input [15:0] aluOut,input p1_toReg,input  p1_regWr,input [2:0] destReg,output p2_carryOut,
   output [15:0] p2_aluOut,output p2_toReg,output p2_regWr,output [2:0] p2_destReg);

	register16bit register16bit_4( clk, reset, regWrite, aluOut, p2_aluOut );
	
	register3bit register3bit_3(  clk,  reset, regWrite, destReg , p2_destReg );
	
	register1bit register1bit_4(  clk, reset, regWrite, p1_toReg , p2_toReg);
	register1bit register1bit_5(  clk, reset, regWrite, p1_regWr , p2_regWr);
	register1bit register1bit_6(  clk, reset, regWrite, carryOut , p2_carryOut);
	
endmodule

//Module uses register1bit, register3bit and register16bit
module EX2_WB(input clk, input reset,input regWrite,input [15:0] p2_aluOut,input [15:0] adderOut,
input [2:0] p2_destReg,input p2_toReg,input  p2_regWr,output [15:0] p3_aluOut,output [15:0] p3_adderOut,
output [2:0] p3_destReg,output p3_toReg,output p3_regWr);
	
	register16bit register16bit_5( clk, reset, regWrite, p2_aluOut, p3_aluOut );
	register16bit register16bit_6( clk, reset, regWrite, adderOut, p3_adderOut );
	
	register3bit register3bit_4(  clk,  reset, regWrite, p2_destReg , p3_destReg );
	
	register1bit register1bit_7(  clk, reset, regWrite, p2_toReg , p3_toReg);
	register1bit register1bit_8(  clk, reset, regWrite, p2_regWr , p3_regWr);
	
endmodule

//Module uses register1bit, register3bit and register16bit
module WB(input clk, input reset,input regWrite,input [15:0] Result,
input [2:0] p3_destReg,input  p3_regWr,output [15:0] p4_result,
output [2:0] p4_destReg,output p4_regWr);
	
	register16bit register16bit_7( clk, reset, regWrite, Result, p4_result );
	
	register3bit register3bit_5(  clk,  reset, regWrite, p3_destReg , p4_destReg );
	
	register1bit register1bit_9(  clk, reset, regWrite, p3_regWr , p4_regWr);
	
endmodule 

//Hazard Unit
module hazard(input [2:0] p0_rt,input [2:0] p0_rs,input [2:0] p1_rd,input [3:0] p1_opcode,
output reg pcWr, output reg IF_IDWr, output reg flush);
	
	always@(p0_rt or p0_rs or p1_rd or p1_opcode)
		begin	
				if ((p1_opcode == 4'b0010) && ((p1_rd == p0_rs) || (p1_rd == p0_rt))) 
					begin
						pcWr = 1'b0;
						IF_IDWr = 1'b0;
						flush = 1'b0;
					end
					else
					begin
						pcWr = 1'b1;
						IF_IDWr = 1'b1;
						flush = 1'b1;
					end	
		end
	
endmodule

//Forwarding Unit
module forward(input [2:0] p1_rs, input [2:0] p1_rt, input [2:0] p2_destReg, input [2:0] p3_destReg, input [2:0] p4_destReg,
input p2_regWr, input p3_regWr,input p4_regWr, output reg [1:0] fwdA, output reg [1:0] fwdB); 
	
	always@(p1_rt or p1_rs or p2_destReg or p3_destReg or p4_destReg or p2_regWr or p3_regWr or p4_regWr)
		begin
			begin 
				fwdA = 2'b00;
				fwdB = 2'b00;
			end	 
			if ((p2_destReg == p1_rs) && p2_regWr)
				fwdA = 2'b01;
			if ((p2_destReg == p1_rt) && p2_regWr )
				fwdB = 2'b01;
			if ((((p3_destReg == p1_rs)) && p3_regWr) && (!((p2_destReg == p1_rs) && p2_regWr)))
				fwdA = 2'b10;
			if ((((p3_destReg == p1_rt)) && p3_regWr) && (!((p2_destReg == p1_rt) && p2_regWr)))
				fwdB = 2'b10;
			if ((((p4_destReg == p1_rs)) && p4_regWr) && ((!((p3_destReg == p1_rs) && p3_regWr) && (!((p2_destReg == p1_rs) && p2_regWr))) ))
				fwdA = 2'b11;
			if ((((p4_destReg == p1_rt)) && p4_regWr) && ((!((p3_destReg == p1_rt) && p3_regWr) && (!((p2_destReg == p1_rt) && p2_regWr))) ))		
				fwdB = 2'b11;
		end
	
endmodule

//TopModule
module pipelinehazards(input clk, input reset, output [15:0] Result );

wire [15:0] outPC,IR,adder_out,muxOut_alu1,p0_intr,outR0,outR1,muxOut_alu,zeroExtOffset,p1_regOut1,p1_regOut2,p1_zExtOut,p2_aluOut,p4_result,muxout_alu2,aluOut1,zeroExtcFlag,adder_out_1,p3_aluOut,p3_adderOut;
wire [5:0] muxout;
wire [3:0] p1_opcode;
wire [2:0] p3_destReg,p1_rd,p1_inst_Rs,p1_inst_Rt,p1_inst_Rd,muxout_destreg,p2_destReg, p4_destReg;
wire [1:0] aluOp,p1_aluOp,fwdA,fwdB;
wire p3_regWr,aluSrcB,regDest,toReg,regWr,flush,pcWr,IF_IDWr,p1_aluSrcB,p1_regDest,p1_toReg,p1_regWr,p2_regWr,p4_regWr,carry,p2_carryOut,p2_toReg,p3_toReg;
	

	
register16bit register16bit_8( clk, reset, pcWr, adder_out, outPC );
IM IM_0( clk, reset, outPC[4:0] , p0_intr );
adder adder_0( outPC, 16'd2, adder_out);

IF_ID IF_ID_0( clk, reset, IF_IDWr, p0_intr,IR);

registerFile registerFile_0(clk, reset, p3_regWr, IR[11:9], IR[8:6], p3_destReg , Result, outR0,  outR1);
zeroExt4to16 zeroExt4to16_0( IR[3:0], zeroExtOffset);
ctrlCkt ctrlCkt_0(IR[15:12], aluSrcB, aluOp,regDest,toReg,regWr);
mux2to1_6bits mux2to1_6bits_0(6'b0,{regWr,toReg,regDest,aluOp,aluSrcB}, flush, muxout);

ID_EX1 ID_EX1_0(clk, reset, 1'b1,outR0,outR1,zeroExtOffset,IR[11:9],IR[8:6],IR[5:3], IR[15:12],muxout[0],muxout[2:1],muxout[3],muxout[4],muxout[5],p1_regOut1,p1_regOut2,p1_zExtOut,p1_inst_Rs,p1_inst_Rt,p1_inst_Rd,p1_opcode,p1_aluSrcB,p1_aluOp,p1_regDest,p1_toReg,p1_regWr);

mux4to1_16bits mux4to1_16bits_0(p1_regOut1,p2_aluOut,Result,p4_result,fwdA, muxOut_alu);
mux4to1_16bits mux4to1_16bits_1(p1_regOut2,p2_aluOut,Result,p4_result,fwdB, muxOut_alu1);
mux2to1_16bits mux2to1_16bits_0(muxOut_alu1, p1_zExtOut,p1_aluSrcB, muxout_alu2);
alu alu_0(muxOut_alu, muxout_alu2,p1_aluOp,carry, aluOut1);
mux2to1_3bits mux2to1_3bits_0(p1_inst_Rt,p1_inst_Rd,p1_regDest, muxout_destreg);

EX1_EX2 EX1_EX2_0(clk,reset,1'b1, carry, aluOut1,p1_toReg, p1_regWr,muxout_destreg,p2_carryOut,p2_aluOut,p2_toReg,p2_regWr,p2_destReg);

zeroExt1to16 zeroExt1to16_0( p2_carryOut,zeroExtcFlag);
adder adder_1(zeroExtcFlag,p2_aluOut, adder_out_1);

EX2_WB EX2_WB_0(clk,reset,1'b1,p2_aluOut,adder_out_1,p2_destReg,p2_toReg,p2_regWr,p3_aluOut,p3_adderOut,p3_destReg,p3_toReg,p3_regWr);

mux2to1_16bits mux2to1_16bits_1(p3_adderOut,p3_aluOut,p3_toReg,Result);

WB WB_0(clk,reset,1'b1,Result,p3_destReg,p3_regWr,p4_result,p4_destReg,p4_regWr);

hazard hazard_0(IR[8:6], IR[11:9], p1_inst_Rd, p1_opcode,pcWr, IF_IDWr, flush);
forward forward_0(p1_inst_Rs, p1_inst_Rt, p2_destReg, p3_destReg, p4_destReg,p2_regWr,p3_regWr,p4_regWr,fwdA,fwdB);
	
endmodule

module pipelinehazardsTestBench;
	reg clk;
	reg reset;
	wire [15:0] Result;
	pipelinehazards uut (.clk(clk), .reset(reset), .Result(Result));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#10  reset=0;	
		
		#150 $finish; 
	end
endmodule

