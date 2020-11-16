`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/05/13 17:21:25
// Design Name: 
// Module Name: new_cpu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module cpu(
    input clk,rst,//clock,reset
    output [9:0]_control,
    output zf
    );
wire [31:0] next_pc, ex_addr, j_addr, beq_addr;
reg [31:0] pc, addr, mips, alu_b, alu_res, reg_a, reg_b, RegIn, MemOut;
wire [5:0] op, funct;
wire PcSrc;
wire [4:0] rs, rt, rd, shamt, RegDst;
reg [9:0] control;
reg [2:0] alu_op;

wire cf,of,sf;

assign op = mips[31:26];
assign rs = mips[25:21];
assign rt = mips[20:16];
assign rd = mips[15:11];
assign ex_addr = mips[15] ? {16'hffff, mips[15:0]} : {16'h0000, mips[15:0]};
assign PcSrc = zf & control[2];
assign j_addr = {next_pc[31:28], mips[25:0], 2'b00};
assign _control = control;
mux2 _RegDst(RegDst, rt, td, control[8]);
mux2 _ALUSrc(alu_b, reg_b, ex_addr, control[7]); 
mux2 _MemtoReg(RegIn, alu_res, MemOut, control[6]);

alu _ALU(alu_res, zf, cf, of, sf, reg_a, alu_b, alu_op);

register_file (clk, rs, reg_a, rt, reg_b, RegDst, RegIn); 
ins instruct(pc[9:2],mips);
data DataMemory(alu_res[9:2], reg_b, alu_res[9:2], clk, control[3], MemOut);

always @*
begin
    case(control[1:0])
    2'b00:alu_op=3'b000;
    2'b01:alu_op=3'b001;
    endcase
end


//next pc
alu _next_pc(.y(next_pc), .a(pc), .b(32'd4), .m(3'b000));
alu _beq_addr(.y(beq_addr), .a(next_pc), .b({ex_addr[29:0], 2'b00}), .m(3'b000));

 
//pc
always @(posedge clk,posedge rst)
begin
    if(rst)
        pc <= 32'd0;
    else if(control[9] == 1)
        pc <= j_addr;
    else if(PcSrc == 1)
        pc <= ex_addr;
    else 
        pc <= next_pc;
end


//Control Unit
//control = {Jump, RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp[1:0]} 
always @*
begin
    case(mips[31:26])
    6'b000000:
    begin//add
        control <= 9'b100100000;
    end
    6'b001000:
    begin//addi
        control <= 9'b010100000;
    end
    6'b100011:
    begin//lw
        control <= 9'b011110000;
    end         
    6'b101011:
    begin//sw
        control <= 9'b010001000;
    end     
    6'b000100:
    begin//beq
        control <= 9'b000000101;
    end      
    6'b000010:
    begin//j
        control <= 9'b000000100;
    end
    endcase
end


endmodule


