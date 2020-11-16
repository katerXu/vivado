`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/29/2020 05:20:59 PM
// Design Name: 
// Module Name: sim
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

module sort_sim(

       );
reg clk, rst;
reg [3:0] x0, x1, x2, x3;
wire [3:0] s0, s1, s2, s3;
wire done;

parameter PERIOD = 10,   	//time per cycle
          CYCLE = 20;		//total cycles

sort SORT(s0, s1, s2, s3, done, x0, x1, x2, x3, clk, rst);

initial
begin
    clk = 0;
    repeat (3 * CYCLE)
    #(PERIOD/2) clk = ~clk;
    $finish;
end

initial
  begin
    rst = 1;
    #PERIOD 
    rst = 0;

    #( PERIOD * 9 ) 
    rst = 1;
    #PERIOD rst = 0;
  end

initial
  begin
    x0 = 4;
    x1 = 15;
    x2 = 2;
    x3 = 9;

  end
endmodule

