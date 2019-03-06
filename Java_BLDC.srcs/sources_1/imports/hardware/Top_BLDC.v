`include "..\include\timescale.v"
`include "..\include\defines.v"
///////////////////////////////////////////////////////////////////////
// Company: 	Tremaine Consulting Group.
// Engineer: 	Brian Tremaine
// 
// Create Date:    	Jan 4, 2019 
// Design Name: 
// Module Name:    	Top_BLDC
// Project Name:   	 
// Target Devices:	Xilinx XC7A50T-2FGG484C  	
// Tool versions: 
// Description:    	Inner & outer control loops for BLDC
//
// Dependencies:    Debug file built Prysm 024-00300-03 Bali-2 board
//						
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
///////////////////////////////////////////////////////////////////////
module Top_BLDC(
			// inputs
			input FPGA_CLK_OSC, //--- system clock 25Mc (xtal)
			input Fg,    //---------- Fg input ~360Hz, raw         : FA_1
			// outputs, 
			output pwm1o, //--------- pwm1 coarse control          : FA_2		
			output D0, // ----------- D0 tp: ~360Hz, fgIn_sync     : FA_3
			output D2, // ----------- D2 tp: 360Hz Ref             : FA_4  				
			output Fref60,// -------- Fref60, 60Hz Ref             : KIN_STEP_M01 			
			output pwm2o, //--------- pwm2 fine control            : KIN_STEP_M02
			output D1, // ----------- LED D1 tp: ~60Hz, feedback   : KIN_STEP_M03
			output D3, //------------ LED D3 tp: pd_error          : KIN_STEP_M04
			output Test_LED1, //------ debug TP                    : Test_LED1 (probe inverted)
			output Test_LED2  // ----- debug TP                    : Test LED2 (probe inverted)
    );
///////////////////////////////////////////////////////////////////////
wire sp_clk;
wire clk_100;
wire clk_200;
wire RESET_N;  // active LOW reset

`ifdef BENCH        // defined in defines.v for testbench simulation
wire dcm_lock ;
assign dcm_lock = 1'b1;

reg clk3;
reg clk2;

initial 
  begin
  clk3 <= 0;
  clk2 <= 0;
  end 
  
  always @ (posedge CLK1)
     clk3 <= ~clk3 ;
	  
  always @ (posedge CLK1, negedge CLK1)  // wrong, need 2x clock for sim
     clk2 <= ~clk2 ;
	  	  
 assign sp_clk = clk3 ;	// 50MHz
 assign CLK2x = clk2 ;  // 200MHz
 //------------ DCM with 2x ----- not in test bench-------------------
`else
//----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG
assign RESET_N = 1'b1;
  clk_wiz_0 instance_name
   (
    // Clock out ports
    .clk_100x(clk_100),      // output clk_100x
    .clk_50x(sp_clk),        // output clk_50x
    .clk_200x(clk_200),      // output clk_200x
    // Status and control signals
    .reset(~RESET_N),        // input reset
    .locked( ),              // output locked
   // Clock in ports
    .clk_in1(FPGA_CLK_OSC));         // input clk_in1 (25MHz)
// INST_TAG_END ------ End INSTANTIATION Template ---------
`endif 
// --------------------------------------------------------------------
parameter FSZE = 6;        // bits allocated to fraction for pwm
parameter FFPT = 6;        // floating point bits for compensator
parameter Af = 6'h3d;      // 61
parameter Bf = 6'h30;      // 48
parameter K0 = 7'h7f;      // 127

parameter Ki = 7'h03;      //  3 unsigned
parameter Kp = 7'h10;      // 16 unsigned
parameter K02= 9'h020;     // 32 unsigned

parameter DLIM_sp = 16'h7FFF;	// input speed error limit
parameter DLIM_ph = 16'h7FFF;	// input phase error limit
parameter INT_LIM = 200000;	// integrator absolute limit

parameter WIN= 9'h1FF;			// window width 9'h1FF is max
parameter WIDTH=17;
parameter WIDTH_ERR=22;
parameter Nref = 833333; 	// 20'hCB735 Fref60 period, 60.0Hz using 50MHz clock
parameter N0 = 8000; 		// 13'h1F40 pwm period 25kHz
parameter M0 = 3800; 		// pwm default center point @Me==0
parameter N1 = 555555;      // divider 200MHz to 360Hz in fractional divider
parameter N360 = 138888;    // divider 50MHz for 360Hz inner loop debug

reg [16:0] m0;
reg [FFPT-1:0]    af;		// lead coefficient
reg [FFPT-1:0]    bf;		// lag coefficient
reg [2+FFPT-1:0]  k0;		// gain coefficient
//
reg [FFPT-1:0]    ki;		// integrator coefficient
reg [FFPT-1:0]    kp;		// position coefficient
reg [4+FFPT-1:0]  k02;		// gain coefficient

reg [WIDTH-1:0] no;			// pwm frequency divider
reg [15:0] dlim_sp;         // 
reg [15:0] dlim_ph;         //
reg [27:0 ] intlim;         // set width %%%%%%%%%%%%%%%%%%
reg [9:0]  win_width;			    // set window width the same for both loops
reg [WIDTH_ERR-2:0] win_delay_360;	// set window delay for inner loop
reg [WIDTH_ERR-2:0] win_delay_60;	// set window delay for outer loop
reg [20:0] M;					    // Fref60 counter
reg [20:0] M360;                    // Ref360 inner loop debug

wire [WIDTH-1:0] Me;	   // lead-lag comp out
wire [24-1:0] Me2;      // PI comp out
wire sync_rst_n;
wire [WIDTH_ERR-1:0] Err;  // signed speed error
wire [WIDTH_ERR-1:0] Err2; // signed phase error
wire Venable;
wire sample;
wire sample2;
wire pd_error;
wire win_open;
wire fgIn_sync;
wire Fref360;
wire Ref360;
wire fgFb;
wire fg60;
wire [WIDTH-1:0] pwm_in;
wire pwm1;
wire pwm2;
wire [WIDTH-1:0] dphase;

// instantiate rst_gen
   rst_gen rst_gen1(
    // inputs
    .reset_n    	(RESET_N),	    // reset from IBUFF
    .clk        	(sp_clk), 		 // master clock source
    // outputs
    .sync_rst_n    (sync_rst_n) 	 // synchronized reset
    );
	
// =============== inner loop control =========================================

// instantiate debounce module
	debounce #(.DELAY(500))	debounce1(
    // Inputs
    .sync_rst_n	(sync_rst_n), 	    // synchronized reset
    .clk			(sp_clk),		// master clock source
    .inpt			(Fg),			// raw input signal
    // Outputs
    .outp			(fgIn_sync) 	// debounced output        
    ) ;
		
// instantiate counter (div-by-6), used for feedback in outer loop	
	down_cnt_sync #(.WIDTH(4)) down_cnt_sync1(
	   // inputs
		.sys_clk    (sp_clk),			// master clock source
		.clk_in		(fgIn_sync),		// input source clock
		.sync_rst_n (sync_rst_n),		// synchronized reset
		.N			(4'h6),				// divide ratio
		// outputs
		.count		(),					// reg count value
		.q_out		(fg60)				// output, ~60Hz
       );	
	
/// instantiate phase detector for inner speed control loop
    phase_det phase_det1(
	   // inputs
	   .clk        		(sp_clk),		// master clock source
	   .sync_rst_n 		(sync_rst_n),	// synchronized reset
	   .ref_phase  		(Fref360),		// input raw vsync
	   .fb_phase   		(fgIn_sync),    // input feedback phase
	   .delay_len  		(win_delay_360),// user reg, window delay count
	   .width_win  		(win_width),	// user reg, window width count
	   // outputs
	   .sample_out      (sample),
	   .err        		(Err),          // error count
	   .pd_error   		(pd_error),     // (RST signal, lost vsync)
	   .win             (win_open)
    );

// instantiate compensation for inner speed control loop
	lead_lag_comp #(.WIDTH_U(18), .WIDTH_ERR(22), .WIDTH_LIM(16), 
                  .WIDTH_FP(29), .fsze(FSZE)) lead_lag_comp1 (
	// input
	.sys_clk				(sp_clk),		// system clock
	.rst					(~sync_rst_n),	// active high reset
	.err					(Err),			// signed error, Q.0
	.dlim					(dlim_sp),		// error limit
	.af						(af),			// af = a * (2<<fsze), Q.fsze
    .bf						(bf),			// bf = b * (2<<fsze), Q.fsze 
	.k0						(k0),           // kf = ko * (2<<fsze), Q.fsze
	.enable					(Venable),
	.process                (sample),
	// outputs
	.uk						(Me)
    );

// instantiate fractional pwm
	frac_pwm_dual #(.WIDTH(17), .fsze(FSZE)) frac_pwm_dual1(
	// inputs
	.sys_clk			    (clk_200),		// fast clock
	.sync_rst_n             (sync_rst_n),
	.N					    (m0),			// default centerpoint at Me==0 (3800/8000 == 47.5%)
	.No                     (no),			// pwm period divide (50Mc/8000 == 25kHz)
	.mf				        (pwm_in),	    // signed fractional divide ...yyyyy.xxxx, pwm_in
	// outputs
	.q1_out			        (pwm1),			// pwm output
	.q2_out                 (pwm2)
	);	
	
// =========== outer loop control =============================================

// instantiate phase detector for outer phase control loop
    phase_det phase_det2(
	// inputs
	.clk        		(sp_clk),		// master clock source
	.sync_rst_n 		(sync_rst_n),	// synchronized reset
	.ref_phase  		(Fref60),		// input fref vsync
	.fb_phase   		(fg60),         // input feedback phase
	.delay_len  		(win_delay_60),	// user reg, window delay count
	.width_win  		(win_width),	// user reg, window width count
	// outputs
	.sample_out         (sample2),
	.err        		(Err2),        // phase error count
	.pd_error   		(),            // 
	.win                ()
    );

// instantiate compensation for outer PI control loop
	pi_comp #(.WIDTH_U(24), .WIDTH_ERR(22), .WIDTH_LIM(16), 
                  .WIDTH_FP(30), .fsze(6)) pi_comp1 (
	// input
	.sys_clk			(sp_clk),	      // system clock
	.rst				(~sync_rst_n),	  // active high reset
	.err				(Err2),           // signed error, Q.0
	.dlim				(dlim_ph),        // phase error limit
	.intlim             (intlim),         // integrator limit
	.ki					(ki),		      // ki  * (2**fsze), Q.fsze
    .kp					(kp),			  // kp  * (2**fsze), Q.fsze 
	.k0					(k02),            // k0  * (2**fsze), Q.fsze
	.enable				(Venable),
	.process			(sample2),
	// outputs
	.uk					(Me2)
    ); 
	 
// instantiate fractional divider for 'vco' in outer loop
	frac_divider #(.WIDTH(21), .FSZE(3)) frac_divider1 (
	// inputs
	.sys_clk				(clk_200),		// 200MHz
	.sync_rst_n				(sync_rst_n),	// active low reset
	.N						(N1),			// integer divider
	.mf						(Me2),		    // fractional divide ...yyyyy.xxx
	// outputs
	.count					(),
	.q_out                  (Fref360)
    );


assign Venable = 1'h1;  // set comp Venable
assign pwm1o = pwm1;	// set pwmo 
assign pwm2o = pwm2;    // set pwm2o

assign D0 = fgFb;       // feedback from motor after debounce
assign D1 = fg60;
assign D2 = Fref360;    // ref 360Hz Fref360, debug Ref360
assign D3 = pd_error;
assign Test_LED1 = pd_error;
assign Test_LED2 = win_open;

assign pwm_in = (~Me+1);			// polarity for fractional pwm
assign dphase = (~Me2+1);           // polarity for fractional divider
assign fgFb = fgIn_sync;
assign Fref60 = (M < (Nref>>1) ) ? 1'b1: 1'b0;      // generate Fref60
assign Ref360 = (M360 < (N360>>1) ) ? 1'b1: 1'b0;   // generate Ref360


	always @ (posedge sp_clk) begin
		m0 <= M0;
		af <= Af;
		bf <= Bf;
		k0 <= K0;
		ki <= Ki;
		kp <= Kp;
		k02 <= K02;
        dlim_sp <= DLIM_sp;
		dlim_ph <= DLIM_ph;
		intlim <= INT_LIM;
		no <= N0;
		win_width <= WIN;
		win_delay_360 <= N360 - (WIN>>1); //
		win_delay_60 <= Nref - (WIN>>1);
	end

	// Fref60 counter (generates 60Hz ref clock for outer loop
	always @(posedge sp_clk) begin
		if(~sync_rst_n) begin
		   M <= Nref;
		end else begin
		   if (M==1)
			  M <= Nref;
			else
			  M <= M - 1'b1;
		end
	end

// Inner loop debug	
//    // Ref360 for inner loop debug
//	always @(posedge sp_clk) begin
//		if(~sync_rst_n) begin
//		   M360 <= N360;
//		end else begin
//		   if (M360==1)
//			  M360 <= N360;
//			else
//			  M360 <= M360 - 1'b1;
//		end
//	end
	
	
    // --------------------------------------------------------------
			
endmodule
