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
