`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/06/2020 05:25:30 PM
// Design Name: 
// Module Name: func
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

/*
module  ram_16x8    	//16x8位单端口RAM
(input  clk, 			//时钟（上升沿有效）
input en, we,			//使能，写使能
input [3:0] addr,       //地址
input [7:0] din,		//输入数据
output [7:0] dout,	    //输出数据
output [7:0] mem1
);
reg [3:0] addr_reg;
reg [3:0] mem[0:2];

//初始化RAM的内容
initial
$readmemh("C:\Users\xujh2649\Desktop\data.txt", mem); 

assign dout = mem[addr_reg];
assign mem1 = mem[0];

always@(posedge clk) 
begin
    if(en) 
    begin
        addr_reg <= addr;
        if(we)
            mem[addr] <= din;
    end
end
endmodule
*/

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

module fifo
(input clk, rst,	    //时钟（上升沿有效）、异步复位（高电平有效）
input [7:0] din,	    //入队列数据
input en_in, 		    //入队列使能，高电平有效
input en_out,		    //出队列使能，高电平有效
output [7:0] dout, 	    //出队列数据
output reg [4:0] count	//队列数据计数
);
localparam S0 = 2'd0;       //empty queue
localparam S1 = 2'd1;       //
localparam S2 = 2'd2;       //full queue

wire edge_in, edge_out;
wire [3:0] dest;
reg [1:0] state, next_state;
reg [4:0] next_count;
reg [3:0] queue_head, queue_tail, next_queue_head, next_queue_tail;

get_edge edge1(clk,rst,en_in,edge_in);
get_edge edge2(clk,rst,en_out,edge_out);
dist_mem_gen_0 dist(dest, din, clk, edge_in, dout);
//ram_16x8 ram(clk, 1, edge_in, dest, din, dout, mem1);

assign dest = edge_in ? queue_tail : queue_head;
 
//state logic
always @(posedge rst, posedge clk) 
    if(rst)
        state <= S0;
    else
        state <= next_state;

//next state logic
always@(posedge clk, posedge rst)
begin
    if(rst)
        next_state <= S0;
    else
    begin
        if(edge_in == 1)
        begin
            case(state)
            S0: next_state <= S1;
            S1: if(count == 5'd15) next_state <= S2;
            default: next_state <= next_state;
            endcase
        end
        if(edge_out == 1)
        begin
            case(state)
            S2: next_state <= S1;
            S1: if(count == 5'd1) next_state <= S0;
            default: next_state <= next_state;
            endcase
        end
    end
end        

//count logic
always @(posedge rst, posedge clk) 
    if(rst)
        count <= 5'd0;
    else
        count <= next_count;

//next count logic
always @(posedge clk, posedge rst)
begin
    if(rst) next_count = 5'd0;
    else
    begin
        if(edge_in ==  1)
            if(state != S2)
                next_count <= next_count + 5'd1;
        if(edge_out == 1)
            if(state != S0)
                next_count <= next_count - 5'd1;
    end
end

//queue_head logic
always @(posedge clk, posedge rst)
begin
    if(rst) queue_head <= 4'd0;
    else queue_head <= next_queue_head;
end

always @(posedge clk, posedge rst)
begin
    if(rst) next_queue_head <= 4'd0;
    else
    begin
        if(edge_out == 1)
            if(state != S0)
                next_queue_head <= queue_head + 4'd1;
    end
end

//queue_tail logic
always @(posedge clk, posedge rst)
begin
    if(rst) queue_tail <= 4'd0;
    else queue_tail <= next_queue_tail;
end

always @(posedge clk, posedge rst)
begin
    if(rst) next_queue_tail <= 4'd0;
    else
    begin
        if(edge_in == 1)
            if(state != S2)
                next_queue_tail <= queue_tail + 4'd1;
    end
end
endmodule
