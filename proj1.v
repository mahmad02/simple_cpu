module half_adder(A, B, S, C);
    //what are the input ports.
    input A;
    input B;
    //What are the output ports.
    output S;
    output C;
    //reg S,C; NOT NEEDED will cause error
     //Implement the Sum and Carry equations using Structural verilog
     xor x1(S,A,B); //XOR operation (S is output not a input)
     and a1(C,A,B); //AND operation 
 endmodule

//Declare the ports for the full adder module
module full_adder( A, B, Cin, S, Cout); // (Port names)

    //what are the input ports.
    input A;
    input B;
    input Cin;

    //What are the output ports.
    output S;
    output Cout;
    //reg  A,B,Cin; //(Doesn't need to be reg, it will cause an error
    wire Sa1, Ca1, Ca2;    
     //Two instances of half adders used to make a full adder
     half_adder a1(.A(A),.B(B),.S(Sa1),.C(Ca1)); //S to Sa1 to match diagram
     half_adder a2(.A(Sa1),.B(Cin),.S(S),.C(Ca2));
     or  o1(Cout,Ca1,Ca2);
 endmodule

module my16bitadder(output [15:0] O, output c, input [15:0] A, B, input S);
//wires for connecting output to input
	wire Cout0in1, Cout1in2, Cout2in3, Cout3in4, Cout4in5, Cout5in6, Cout6in7, Cout7in8,
 	Cout8in9, Cout9in10, Cout10in11, Cout11in12, Cout12in13, Cout13in14, Cout14in15;

	full_adder Add0 ( A[0], B[0], S, O[0], Cout0in1); 
	full_adder Add1 ( A[1], B[1], Cout0in1, O[1], Cout1in2); 
	full_adder Add2 ( A[2], B[2], Cout1in2, O[2], Cout2in3); 
	full_adder Add3 ( A[3], B[3], Cout2in3, O[3], Cout3in4); 
	full_adder Add4 ( A[4], B[4], Cout3in4, O[4], Cout4in5); 
	full_adder Add5 ( A[5], B[5], Cout4in5, O[5], Cout5in6); 
	full_adder Add6 ( A[6], B[6], Cout5in6, O[6], Cout6in7); 
	full_adder Add7 ( A[7], B[7], Cout6in7, O[7], Cout7in8); 
	full_adder Add8 ( A[8], B[8], Cout7in8, O[8], Cout8in9); 
	full_adder Add9 ( A[9], B[9], Cout8in9, O[9], Cout9in10); 
	full_adder Add10 ( A[10], B[10], Cout9in10, O[10], Cout10in11); 
	full_adder Add11 ( A[11], B[11], Cout10in11, O[11], Cout11in12);
	full_adder Add12 ( A[12], B[12], Cout11in12, O[12], Cout12in13); 
	full_adder Add13 ( A[13], B[13], Cout12in13, O[13], Cout13in14); 
	full_adder Add14 ( A[14], B[14], Cout13in14, O[14], Cout14in15);  
	full_adder Add15 ( A[15], B[15], Cout14in15, O[15], c);
endmodule

module my8bitmultiplier(output reg[15:0] O, output reg Done, output Cout, input [7:0] A, B, input Load,Clk,Reset,Cin);
reg[1:0] state;
reg[15:0] A_reg, B_reg, A_temp, O_reg, B_temp;
wire[15:0] O_temp;

my16bitadder dut1(O_temp, Cout, A_temp, B_temp, Cin);

always@(posedge Clk)
begin
if(Reset)
state <= 0;
else
case(state)
	0: if(Load)begin
		A_reg <= A; 
		B_reg <= B; 
		O_reg <= A; 
		state <= 1;
                Done <= 0;
                O <= 0;
		end
	1: begin
		A_temp <= A_reg; 
		B_temp <= O_reg; 
		B_reg <= B_reg - 1; 
		state <= 2;
		end
	2: begin
		O_reg <= O_temp;
		if(B_reg>1) state <= 1;
		else begin
			state <= 3; 
			Done <= 1;
			O <= O_temp;
			end
		end
	3: begin
		Done <= 0; 
		state <= 0;
		end

endcase
end
endmodule

module ram(q, we, d, addr);
output reg [15:0] q;
input we;
input [15:0] d;
input [7:0] addr;

reg [15:0] MEM [0:255];

	always @ (*) begin
		if (we==1) 
			MEM[addr]<=d;
		else 
			q <= MEM[addr];	
end
endmodule

module alu(output reg [15:0] Rout, output reg mult_done, input [15:0] A, B, input [1:0]opAlu,input loadMult, input clk, input rst);
wire [15:0]Add_reg;
wire Cout;
wire Zed = 0, Uno = 1;
wire [15:0] mult_out;
wire donez;
my16bitadder ad1(Add_reg, Cout, A, B, Zed);
my8bitmultiplier mulman(mult_out, donez, Cout, A[7:0] ,B[7:0], loadMult, clk, rst, Zed); 
always @ (*) begin
case (opAlu)
2'd0: Rout<= A | B;
2'd1: Rout<=Add_reg;
2'd2: if(donez) begin Rout<=mult_out; mult_done<=donez; end
2'd3: Rout<=~A;
endcase
end
//A => 16 bit input 1
//B => 16 bit input 2
//opAlu, 2 bit input, 0 = or, 1 = a+b, 2 = Mul, 3 = negate
//Rout => 16 bit output
endmodule




module ctr (clk, rst, zflag, opcode, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, MemRW,loadMult,mult_done);
        input clk;
        input rst;
        input zflag;
        input [7:0]opcode;
        output reg muxPC;
        output reg muxMAR;
        output reg muxACC;
        output reg loadMAR;
        output reg loadPC;
        output reg loadACC;
        output reg loadMDR;
        output reg loadIR;
        output reg [1:0]opALU;
        output reg MemRW;
	output reg loadMult;
	input mult_done;
	reg [7:0] state;
	
	//reg mult_done;
	//wire loadMult;

parameter Fetch_1       = 5'b0_0000; 
parameter Fetch_2       = 5'b0_0001;
parameter Fetch_3       = 5'b0_0010; 
parameter Decode       = 5'b0_0011;
parameter ExecADD_1    = 5'b0_0100; 
parameter ExecADD_2    = 5'b0_0101;
parameter ExecOR_1     = 5'b0_0110; 
parameter ExecOR_2     = 5'b0_0111;
parameter ExecLoad_1   = 5'b0_1000; 
parameter ExecLoad_2   = 5'b0_1001;
parameter ExecStore_1  = 5'b0_1010; 
parameter ExecJump     = 5'b0_1011;
parameter ExecNegate_1 = 5'b0_1100; 
parameter ExecNegate_2 = 5'b0_1101;
parameter ExecMult_1   = 5'b0_1110; 
parameter ExecMult_2   = 5'b0_1111;
parameter ExecMult_3   = 5'b1_0000; 
parameter ExecMult_4   = 5'b1_0001;
parameter ExecMult_5   = 5'b1_0010; 
parameter ExecMult_6   = 5'b1_0011;

parameter op_add=8'b001;
parameter op_or= 8'b010;
parameter op_jump=8'b011;
parameter op_jumpz=8'b100;
parameter op_load=8'b101;
parameter op_store=8'b110;
parameter op_mull=8'b1001;
parameter op_neg=8'b1010;



always@(*) begin
if(rst) begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 0;
	opALU =0;
        loadIR = 0;
        MemRW = 0; end
else  begin
case (state)
Fetch_1: 
	begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 1;
        loadPC = 1;
        loadACC = 0;
        loadMDR = 0;
        loadIR = 0;
        MemRW = 0;
	end
Fetch_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        loadIR = 0;
        MemRW = 0;
	end
Fetch_3: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 0;
        loadIR = 1;
        MemRW = 0;
	end
Decode: begin
	muxPC = 0;
        muxMAR = 1;
        muxACC = 0;
        loadMAR = 1;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 0;
        loadIR = 0;
        MemRW = 0; end
//LOAD
ExecLoad_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        loadIR = 0;
        MemRW = 1;
	end

ExecLoad_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 1;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
        loadIR = 0;
        MemRW = 0;
	end

//ADDITION
ExecADD_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        loadIR = 0;
	opALU = 1'b0;
        MemRW = 0;
	end
ExecADD_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 1;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
        loadIR = 0;
        opALU = 1'b1;
        MemRW = 0;
	end

//OR
ExecOR_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        loadIR = 1;
        //opALU = 0;
        MemRW = 0;
	end
ExecOR_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 1;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
        loadIR = 0;
        opALU = 0;
        MemRW = 0;
	end

//LOAD
ExecLoad_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        loadIR = 0;
        MemRW = 1;
	end

ExecLoad_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 1;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
        loadIR = 0;
        MemRW = 0;
	end

//STORE
ExecStore_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 0;
        loadIR = 0;
        MemRW = 1;
	end

//JUMP
ExecJump: begin
	muxPC = 1;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 1;
        loadACC = 0;
        loadMDR = 0;
        loadIR = 0;
        MemRW = 0;
	end

//NEGATE
ExecNegate_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        opALU = 2'd3;
        loadIR = 0;
        MemRW = 0;
	end

ExecNegate_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 1;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
        opALU = 2'd3;
        loadIR = 0;
        MemRW = 0;
	end

ExecMult_1: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 0;
        loadMDR = 1;
        //opALU = 2'd2;
        loadIR = 0;
        MemRW = 0; end
ExecMult_2: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
	loadMult = 1;
        opALU = 2'd2;
        loadIR = 0;
        MemRW = 0; end 
ExecMult_3: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 0;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
	loadMult = 0;
        opALU = 2'd2;
        loadIR = 0;
        MemRW = 0; end 
ExecMult_5: begin
	muxPC = 0;
        muxMAR = 0;
        muxACC = 1;
        loadMAR = 0;
        loadPC = 0;
        loadACC = 1;
        loadMDR = 0;
	loadMult = 0;
        opALU = 2'd2;
        loadIR = 0;
        MemRW = 0; end 


endcase
end end

always@(posedge clk) begin
if(rst)
state<=Fetch_1;

else begin
case (state)
Fetch_1: 
	begin
	state <= Fetch_2;
	end
Fetch_2: begin
	state <= Fetch_3;
	end
Fetch_3: begin
	state <= Decode;
	end
Decode: begin
	
		case(opcode) 
			op_add: begin
				state <= ExecADD_1;  end
			op_or: begin
				state <= ExecOR_1;  end
			op_jump:
				state <= ExecJump;
			op_jumpz:
				if (zflag) 
					state <= ExecJump;
				else 
					state <= Fetch_1;
			op_load:
				state <= ExecLoad_1;
			op_store:
				state <= ExecStore_1;
			op_mull: begin
				state <= ExecMult_1; end
			op_neg: begin
				state <= ExecNegate_1;  end
		
		 
		endcase
	end
//ADDITION
ExecADD_1: begin
	state <= ExecADD_2;
	end
ExecADD_2: begin
	state <= Fetch_1;
	end

//OR
ExecOR_1: begin
	state <= ExecOR_2;
	end
ExecOR_2: begin
	state <= Fetch_1;
	end

//LOAD
ExecLoad_1: begin
	state <= ExecLoad_2;
	end

ExecLoad_2: begin
	state <= Fetch_1;
	end

//STORE
ExecStore_1: begin
	state <= Fetch_1;
	end

//JUMP
ExecJump: begin
	state <= Fetch_1;
	end

//NEGATE
ExecNegate_1: begin
	state <= ExecNegate_2;
	end

ExecNegate_2: begin
	state <= Fetch_1;
	end
ExecMult_1: begin
	state <=ExecMult_2; end 
ExecMult_2: begin
	state <=ExecMult_3; end 
ExecMult_3: begin
	state <=ExecMult_4; end 
ExecMult_4: begin
	if (mult_done)
	state <=ExecMult_5; 
	else state <= ExecMult_4; end 
ExecMult_5: begin
	state <=Fetch_1; end 


endcase
end
end

endmodule


module registers(clk, rst, PC_reg, PC_next, IR_reg, IR_next, ACC_reg, ACC_next,  
           MDR_reg, MDR_next, MAR_reg, MAR_next, zflag_reg, zflag_next );

input wire clk;
input wire rst;
output reg  [7:0]PC_reg; 
input wire  [7:0]PC_next;
 
output reg  [15:0]IR_reg;  
input wire  [15:0]IR_next;  

output reg  [15:0]ACC_reg;  
input wire  [15:0]ACC_next;  

output reg  [15:0]MDR_reg;  
input wire  [15:0]MDR_next;  

output reg  [7:0]MAR_reg;  
input wire  [7:0]MAR_next;  

output reg zflag_reg;
input wire zflag_next;

always@(posedge clk)
begin
if(rst) begin
PC_reg <= 0; 
IR_reg <= 0; 
ACC_reg <= 0; 
MDR_reg <= 0; 
MAR_reg <= 0;  
zflag_reg<=0; end
else
begin
PC_reg <= PC_next; 
IR_reg <= IR_next; 
ACC_reg <= ACC_next; 
MDR_reg <= MDR_next; 
MAR_reg <= MAR_next; 
zflag_reg<= zflag_next; end

end
endmodule


module datapath( clk, rst,  muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC,
           loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ,loadMult,mult_done);

	  input clk;
          input  rst;
          input  muxPC;
          input  muxMAR;
          input  muxACC;
          input  loadMAR;
          input  loadPC;
          input  loadACC;
          input  loadMDR;

          input  loadIR;
          input  [1:0]opALU; 
          output   zflag;
	  output reg loadMult;
	  output reg mult_done;
          output   [7:0]opcode;
          output   [7:0]MemAddr;
          output   [15:0]MemD;
          input   [15:0]MemQ;

reg  [7:0]PC_next;
reg  [15:0]IR_next;  
reg  [15:0]ACC_next;  
reg  [15:0]MDR_next;  
reg  [7:0]MAR_next;  
reg zflag_next;

wire  [7:0]PC_reg;
wire  [15:0]IR_reg;  
wire  [15:0]ACC_reg;  
wire  [15:0]MDR_reg;  
wire  [7:0]MAR_reg;  
wire zflag_reg;
	  
///reg  loadMult;
wire mult_doness;
wire  [15:0]ALU_out;  

reg [7:0]jumpAdd;
reg [15:0]MAYCC_next;
reg [15:0] MARhold;

alu alu1(ALU_out, mult_doness, ACC_reg, MDR_reg, opALU,loadMult,clk,  rst);

registers REGIS1(clk, rst, PC_reg, PC_next, IR_reg, IR_next, ACC_reg, ACC_next,  
           MDR_reg, MDR_next, MAR_reg, MAR_next, zflag_reg, zflag_next );
always @(*) begin
	if (opALU ==2'b10) 
	loadMult =1;
	else
	loadMult =0;
end


always @(*) begin
	if (mult_doness) 
	mult_done = mult_doness;
	else
	mult_done =0; end

always @ (*) begin
	if (muxPC) 
	jumpAdd= IR_reg[15:8];
	else 
	jumpAdd=PC_reg+1;

	if (loadPC)
	PC_next = jumpAdd;
	else
	PC_next = PC_reg;
end


always @ (*) begin
	if (loadIR)
	IR_next= MDR_reg;
	else
	IR_next= IR_reg; 
end

always @ (*) begin
	if (muxACC)
	MAYCC_next = ALU_out;
	else
	MAYCC_next = MDR_reg;
	
	if (loadACC)
	ACC_next = MAYCC_next;
	else
	ACC_next = ACC_reg;
end

always @ (*) begin
	if (loadMDR)
	MDR_next = MemQ;
	else 
	MDR_next = MDR_reg;	
end

always @ (*) begin
	if (muxMAR)
	MARhold = IR_reg[15:8];
	else 
	MARhold = PC_reg;

	if (loadMAR)
	MAR_next = MARhold;		//FIX
	else
	MAR_next = MAR_reg; //lkook at
end

always @ (*) begin
	if (ACC_reg ==0)
	zflag_next = 1'b1;
	else
	zflag_next = 1'b0;
end


//always @(*) begin
assign zflag = zflag_reg;
assign opcode = IR_reg[7:0];
assign MemAddr = MAR_reg;
assign MemD = ACC_reg;
//end

//one instance of ALU
// one instance of register.
endmodule 

module proj1( clk, rst, MemRW_IO, MemAddr_IO, MemD_IO );

           input clk;
           input rst;
	   output MemRW_IO;
	   output [7:0]MemAddr_IO;
           output [15:0]MemD_IO;
 
wire [15:0]MemD, MemQ;
wire [7:0]opcode,MemAddr;
wire [1:0]opALU;
wire loadPC,loadMAR,zflag,muxPC,muxMAR,MemRW,loadACC,loadMDR,loadIR,muxACC,loadMult,mult_done;
//assign MemRW = 1;


//one instance of memory
ram raam(MemQ, MemRW, MemD, MemAddr);

//one instance of controller
ctr str1(clk, rst, zflag, opcode, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, MemRW,loadMult,mult_done);

//one instance of datapath1
datapath datapath1( clk, rst, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ,loadMult,mult_done );

//these are just to observe the signals.
assign MemAddr_IO = MemAddr;
assign MemD_IO = MemD;
assign MemRW_IO = MemRW;

endmodule



module proj1_tb;
reg clk,rst;
wire MemRW_IO;
wire [7:0]MemAddr_IO;
wire [15:0]MemD_IO;


proj1 dut( clk, rst, MemRW_IO, MemAddr_IO, MemD_IO );


always @(proj1_tb.dut.str1.state)
case (proj1_tb.dut.str1.state)
0: $display($time," Fetch_1");
1: $display($time," Fetch_2");
2: $display($time," Fetch_3");
3: $display($time," Decode");
4: $display($time," ExecADD_1");
5: $display($time," ExecADD_2");
6: $display($time," ExecOR_1");
7: $display($time," op_Or1");
8: $display($time," op_Load0");
9: $display($time," op_Load1");
10: $display($time," op_Store");
11: $display($time," op_Jump");
12: $display($time," op_Negate0");
13: $display($time," op_Negate1");
14: $display($time," op_Mult0");
15: $display($time," op_Mult1");
16: $display($time," op_Mult2");
17: $display($time," op_Mult3");
18: $display($time," op_Mult4");
19: $display($time," op_Mult5");


endcase // case(state)
always 
      #5  clk =  !clk; 
		
initial begin
clk=1'b0;
rst=1'b1;
$readmemh("memory.list", proj1_tb.dut.raam.MEM);
#10 rst=1'b0;
#1000 
$display("Final value\n");
$display("0x000e %h\n",proj1_tb.dut.raam.MEM[16'h000e]);
$finish;
end
endmodule

module alu_tb;
reg [15:0] a,b;
reg [1:0] opalu;
reg clk,rst,load;
wire [15:0]out;
wire mult_done;

alu myaludut(out,mult_done,a, b, opalu ,load, clk,  rst);
always #5 clk = ~clk;
initial begin
rst=1;
load =1;
clk=0;
opalu=0;
a = 8'd10; 
b = 8'd10;
#10 rst = 0;
#10 opalu = 1;
#20 opalu = 2;
#200 opalu = 3;
#200 $display (" A = %d, B = %d, O = %d", a, b, out);
#10 $finish;
end
endmodule


