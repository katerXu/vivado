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


module register_file		//32 x WIDTH�Ĵ�����
#(parameter WIDTH = 32) 	//���ݿ��
(input clk,					//ʱ�ӣ���������Ч��
input [4:0] ra0,			//���˿�0��ַ
output [WIDTH-1:0] rd0, 	//���˿�0����
input [4:0] ra1, 			//���˿�1��ַ
output [WIDTH-1:0] rd1, 	//���˿�1����
input [4:0] wa, 			//д�˿ڵ�ַ
input we,					//дʹ�ܣ��ߵ�ƽ��Ч
input [WIDTH-1:0] wd 		//д�˿�����
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
