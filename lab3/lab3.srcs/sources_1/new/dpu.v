`timescale 1ns / 1ps 
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2020/05/12 20:36:29
// Design Name:
// Module Name: DBU
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


module register_file		//32 x WIDTH寄存器堆
#(parameter WIDTH = 32) 	//数据宽度
(input clk,					//时钟（上升沿有效）
input [4:0] ra0,			//读端口0地址
output [WIDTH-1:0] rd0, 	//读端口0数据
input [4:0] ra1, 			//读端口1地址
output [WIDTH-1:0] rd1, 	//读端口1数据
input [4:0] wa, 			//写端口地址
input we,					//写使能，高电平有效
input [WIDTH-1:0] wd 		//写端口数据
);
reg [4:0] addr_reg1, addr_reg2;
reg [WIDTH-1:0] men[0:WIDTH - 1];
initial
  $readmemh( "C:/Users/xujh2649/Desktop/data.txt", mem );
assign rd0 = men[addr_reg1];
assign rd1 = men[addr_reg2];

always@(posedge clk)
begin
    addr_reg1 = ra0;
    addr_reg2 = ra1;
    if(we != 0)
        men[wa] <= wd;
end
endmodule

module get_edge(
input clk, rst,
input y,
output p
);
localparam S0 = 2'd0;
localparam S1 = 2'd1;
localparam S2 = 2'd2;

reg [1:0] state,next_state;

//output logic
assign p = (state == S1);

//state logic
always @(posedge rst, posedge clk) 
    if(rst)
        state <= S0;
    else
        state <= next_state;

//next state logic
always @* 
begin
    next_state = state;
    case(state)
        S0: if(y) next_state = S1;
        S1: if(y) next_state = S2;
            else next_state = S0;
        S2: if(!y) next_state = S0;
        default: next_state = S0;
    endcase
end

endmodule

module DBU(
         input clk,
         input succ,
         input step,
         input rst,
         input [ 2: 0 ] sel,
         input m_rf,
         input inc,
         input dec,
         output reg [ 15: 0 ] led
       );
wire edg_step, edg_inc, edg_dec, clk_10, locked;
reg [ 7: 0 ] m_rf_addr, n_m_rf_addr, next_an;
reg [ 31: 0 ] outdata;
reg clk_cpu;
wire [ 31: 0 ] m_data, rf_data, pc_in, pc_out, instr, rf_rd1, rf_rd2, alu_y, m_rd;
wire RegDst, jump, Branch, MemtoReg, Memwe, ALUSrc, RegWrite, zf;
wire [ 2: 0 ] alu_op;
wire [9:0] control;
cpu cpu1( clk_cpu, rst, control, zf);

assign jump = control[9];
assign RegDst = control[8];
assign ALUSrc = control[7];
assign MemtoReg = control[6];
assign RegWrite = control[5];
assign Branch = control[2];
assign alu_op = control[1:0];

get_edge edg_st( clk, rst, step, edg_step );
get_edge edg_incc( clk, rst, inc, edg_inc );
get_edge edg_decc( clk, rst, dec, edg_dec );
always @( posedge clk )
  begin
    if ( edg_inc )
      begin
        n_m_rf_addr = m_rf_addr + 8'd1;
      end
    else if ( edg_dec )
      begin
        n_m_rf_addr = m_rf_addr - 8'd1;
      end
    else
      begin
        n_m_rf_addr = m_rf_addr;
      end
  end

always @( posedge clk )
  begin
    if ( rst )
      begin
        m_rf_addr <= 8'd0;
      end
    else
      begin
        m_rf_addr <= n_m_rf_addr;
      end
  end
always @ *
  begin
    if ( succ )
      begin
        clk_cpu = clk;
      end
    else
      begin
        clk_cpu = edg_step;
      end
  end
always @ *
  begin
    if ( rst )
      begin
        led = 16'd0;
      end
    else
      begin
        case ( sel )
          3'b000:
            begin
              if ( m_rf )
                begin
                  outdata = m_data;
                end
              else
                begin
                  outdata = rf_data;
                end
              led[ 7: 0 ] = m_rf_addr;
            end
          3'b001:
            begin
              outdata = pc_in;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
          3'b010:
            begin
              outdata = pc_out;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
          3'b011:
            begin
              outdata = instr;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
          3'b100:
            begin
              outdata = rf_rd1;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
          3'b101:
            begin
              outdata = rf_rd2;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
          3'b110:
            begin
              outdata = alu_y;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
          3'b111:
            begin
              outdata = m_rd;
              led[ 11: 0 ] = { jump, Branch, RegDst, RegWrite, 1'b1, MemtoReg, Memwe, alu_op, ALUSrc, zf };
            end
        endcase
      end
  end


endmodule