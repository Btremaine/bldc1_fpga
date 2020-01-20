
//-------------------------------------------------------------------------
//  Project  : BLDC Motor control
//  Module   : dshot.v

//  Parent   : Top_BLDC.v

//  Children : none
//  Description:

//     This module implements protocol Dshot output for running an ESC
//     ref: https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/
//          https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
//
//  Parameters: 
//    dshot150, define for transmission speed.
//    dshot300, define for transmission speed.
//    dshot600, define for transmission speed.
//  Notes:
//        1. Wdog timer will repeat transmission on timeout unless 0
//

`include "timescale.v"

//-----------------------------------------------------------------------
module dshot #(parameter WDOG=100000) (
           // Inputs
           input rstn,              // synchronized reset
           input sys_clk,           // designed for 50Mc
           input [15:0] inpt,       // throttle command, unsigned
		   input status,            // debug bit to ESC
		   input enable,            // module enable
		   input sample,            // rising edge starts process   
           // Outputs
           output outp              // serial data output
) ;

reg [15:0] raw_data = 0;            // 16-bit word             
reg [15:0] counter;                 // state machine counter
reg [3:0]  chksum;                  // chksum on dshot word
reg sample_dly;
reg flag;
reg [4:0] i=0;
wire [19:0] Nrpt;

assign Nrpt = WDOG;                 // if not 0, repeat every Nrpt clks
localparam MAX_STATE = 4'h8;

// choose protocol rate based on sys_clk
`undef dshot150
`undef dshot300
`define dshot600

// !!! ** counts based on using down counter
`ifdef dshot600
   localparam N1 = 20;
   localparam N0 = 52;
   localparam Np = 83;
`endif

`ifdef dshot300
   localparam N1 =  42;
   localparam N0 = 104;
   localparam Np = 167;
`elsif dshot150
   localparam N1 =  85;
   localparam N0 = 210;
   localparam Np = 334;
`endif


reg [7:0] tx_state;			// state machine index
reg [7:0] send_bit;
reg [9:0] Nt;               // data value for bit
reg [9:0] per_cntr;         // counter for overall bit period
reg ser_out;                // output data register
reg Wdog;                   // watch-dog flag
reg [18:0] Wwd_cnt;

wire rst;
wire setFlag;
wire clrFlag;

// state machine states
localparam IDLE = 0;
localparam s_XMIT = 1;
localparam XMIT = 2;
// state machine states
localparam X0 = 0;
localparam X1 = 1;

assign rst = ~rstn;
assign outp = ser_out;
assign clrFlag = ((send_bit == X0) && (tx_state == XMIT))? 1'b1 : 1'b0;
assign setFlag = (counter[4:0] == 4'd1)? 1'b1 : 1'b0;

// ============================================================================
    always @ (posedge sys_clk, posedge rst) begin
        if(rst) begin
                raw_data <= 16'h0000;
                counter <= 0;
				chksum <= 0;
        end
        else begin
            if ((sample & !sample_dly) || Wdog)
                counter <= MAX_STATE;
            else if (counter !=0)
                counter <= counter-1;
            if(!enable)
            begin
                counter <= 0;
            end else begin
            // logic to build dhot word
            //
                case (counter)
                    4'd8: begin
				          // limit data width 
                          raw_data <= inpt & 11'h7FF;  // 11 data bits max				  
                          end
			        4'd7: begin
				          // update raw_data 
                          raw_data <= inpt << 5;   // data to bits [15:5]	
					      end
					4'd6: begin // append debug bit
						  raw_data[4] <= status;   // ESC debug bit
						  end
				    // calculate checksum
					4'd5: begin
					      chksum <= raw_data[15:12];
						  end
					4'd4: begin
					      chksum <= chksum ^ raw_data[11:8];
					      end
					4'd3: begin
						  chksum <= chksum ^ raw_data[7:4];
						  end
					4'd2: begin
                          raw_data[3:0] <= chksum;
                          end
					4'd1: begin
                            // unused state
                          end
					4'd0: begin
                            // don't do anything
                          end	 
                    default: ;    // don't do anything
                endcase
			end
        end
    end 
// -----------------------------------------------------------------
    always @ (posedge sys_clk) begin     // flag to control transmit
        if(rst)
            flag <= 0;
        else begin
            if(clrFlag)
                flag <= 1'b0;
            else if (setFlag)
                flag <= 1'b1;
        end
    end
// -----------------------------------------------------------------
    always @ (posedge sys_clk) begin
        if(rst)
            sample_dly <= 0;
        else
           sample_dly <= sample;
    end
// -----------------------------------------------------------------
// transmit 16-bit word, *MSB* first when flag rises
// "1" is high for N1 counts
// "0" is high for N0 counts
// period is Np counts
//
//         |    "1"    |  "0"       |    "1"      |
//         *********   *****        ***********   **
//         *       *   *   *        *         *   *
//  ********       *****   **********         *****
//               N1      N0                 N1
//
// -----------------------------------------------------------------
    always @ (posedge sys_clk) 
	if(rst)
    begin
		tx_state <= 0;
		send_bit <= 0;
		per_cntr <= 0;
		ser_out <= 0;
		Nt <= 0;
	end
	else begin
    
      case (tx_state)	
	  
	    // idle state
        IDLE:
	      begin
		    if(flag) 
			   tx_state <= s_XMIT;
		  end // case: IDLE
		
        // start of transmission		
		s_XMIT:	 
		  begin
		    i <= 16;
		  	tx_state <= XMIT;
            send_bit <= X0;			
		  end // case s_XMIT
		  
		// do transmission of all bits
		XMIT:
		  begin
		     case (send_bit)
			 
			 X0:
			 begin
				if (i == 0) begin
				   send_bit <= X0;
				   tx_state <= IDLE;
				end else 
				begin
				   i <= i - 1;
			       Nt <= (raw_data[i-1])? N1 : N0;
                   per_cntr <= Np;		
                   ser_out <= 1'b1;	
                   send_bit <= X1;	
                end				   
			 end
			 
			 X1:
			 begin
			    if(per_cntr < 1)
				   send_bit <= X0;
				else begin
				   per_cntr <= per_cntr - 1;
				   ser_out <= (per_cntr > Nt)? 1'b1 : 1'b0;
				end
			 end			 
			 endcase
		  
		  end       		
      endcase		
	end
	
// -------------------------------------------------------------------
// watch-dog timer clock
    always @ (posedge sys_clk) begin
        if(rst || flag)
		begin
            Wwd_cnt <= Nrpt;
			Wdog <= 0;
	    end
        else
            begin
		     if( Wwd_cnt==1)
		      begin
		        Wdog <= 1'b1;
			    Wwd_cnt <= Nrpt;
		      end
			  else
			    if(Wwd_cnt > 1)
				begin
			      Wwd_cnt <= Wwd_cnt - 1;	
				  Wdog <= 1'b0;
				end				  
		    end
    end


endmodule

















