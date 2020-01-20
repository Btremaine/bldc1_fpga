///////////////////////////////////////////////////////////////////////
// Company:     Tremaine Consulting Group.
// Engineer:    Brian Tremaine
//
// Create Date:     Jan 4, 2019
// Design Name:
// Module Name:     Top_BLDC
// Project Name:
// Target Devices:  Xilinx XC7A50T-2FGG484C
// Tool versions:
// Description:     Inner & outer control loops for BLDC
//
// Dependencies:    Debug file built Prysm 024-00300-03 Bali-2 board
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////

  //: Modify 10/23/19 for optical detector & BLHeli board using dshot600
  //  POC board modified to put optical feedback on Fg pin
  // ***** Must set N1 (reg 323) to 0x24FDB            *****
  //       Must set M0 (reg 322) to nominally 0x130
  //       Must set ki (reg 328) to nominally 1

`include "timescale.v"
`include "reg_defines.v"

module Top_BLDC (
            // inputs
            input        clk_12_5,
            input        clk_50,     // --
            input        clk_200,    // --
            input        Fg,         // --------- Fg input 6*Fpoly, raw         : FA_1
            input        TP5,        // feedback (not used)                     : FPGA_TEST6
            input        reset,      // system reset or bldc reset
            input        mems_vsync_sel,
            input        mems_vsync,
            input [31:0] java_bldc_nref_xq ,
            input [15:0] java_bldc_N0_xq   ,
            input [15:0] java_bldc_M0_xq   ,
            input [31:0] java_bldc_N1_xq   ,
            input [31:0] java_bldc_N360_xq ,
            input [15:0] java_bldc_Af_xq   ,
            input [15:0] java_bldc_Bf_xq   ,
            input [15:0] java_bldc_K0_xq   ,
            input [15:0] java_bldc_Ki_xq   ,
            input [15:0] java_bldc_Kp_xq   ,
            input [15:0] java_bldc_K02_xq  ,
            input [15:0] java_bldc_win_width_xq,
            input [15:0] java_bldc_cf_xq  ,
            input [15:0] java_bldc_klpf_xq  ,
            input [15:0] java_bldc_kif_xq  ,
            // outputs,
            output       pwm1o,      // --------- pwm1 coarse control          : FA_2      dshot
            output       pwm2o,      // --------- pwm2 fine control            : FA_3      not used
            output       Vsync,      // --------- Fpoly, Outer Ref             : FPGA_TEST1
            output       TP1,        // ---------                              : FPGA_TEST2
            output       TP2,        // ---------                              : FPGA_TEST3
            output       TP3,        // ---------                              : FPGA_TEST4
            output       TP4         // ---------                              : FPGA_TEST5

    );
///////////////////////////////////////////////////////////////////////
wire RESET_N = !reset;  // active LOW reset
// --------------------------------------------------------------------

parameter MODPH = 1'h0;    // true to output modulo Pi phase

//parameter FSZE     = 6;          // bits allocated to fraction for pwm
parameter FFPT     = 6;          // floating point bits for compensator
parameter DLIM_sp  = 19'h3FFFF;  // input speed error limit
parameter DLIM_ph  = 18'h1FFFF;  // input phase error limit
parameter INT_LIM1 = 28'h1FFFFF; // integrator absolute limit outer loop
parameter INT_LIM2 = 28'h7FFFF;  // integrator absolute limit inner loop
parameter WIDTH    = 19;
parameter WIDTH_ERR= 19;  // was 22


reg [27:0 ] intlim1 = INT_LIM1;    //
reg [27:0 ] intlim2 = INT_LIM2;
reg [18:0] win_delay_360 = `JAVA_BLDC_N360_DATA - (`JAVA_BLDC_WIN_WIDTH_DATA>>4);  // set window delay for inner loop
reg [18:0] win_delay_60  = `JAVA_BLDC_NREF_DATA - (`JAVA_BLDC_WIN_WIDTH_DATA>>1);  // set window delay for outer loop
reg [20:0] M360       = `JAVA_BLDC_N360_DATA;          // Ref360 inner loop debug
reg [20:0] Ref60_cnt  = (`JAVA_BLDC_NREF_DATA >> 1);
reg [15:0] cf;       // lag2 coefficient lead_lag_comp
reg [15:0] klpf;     // low pass filter, unity gain pi_comp

reg Fref_poly = 1'b0;

wire sample_sync;
wire signed [WIDTH-1:0] Me;    // lead-lag comp out
wire signed [24-1:0] Me2;      // PI comp out
wire [15:0] outp;       // lead-lag dshot data
wire sync_rst_n;
wire signed [WIDTH_ERR-1:0] Err;  // signed speed error
wire signed [WIDTH_ERR-1:0] Err2; // signed phase error
wire sample;
wire sample2;
wire done;
wire done_srv;
wire fgIn_sync;
wire Fref6x;
wire fgFb;
wire fg60;
wire [WIDTH-1:0] pwm_in;
wire pwm1;
wire pwm2;
wire signed [WIDTH-1:0] dphase;
wire select_mod;
wire Fpoly;
wire feedback;
wire telem;
wire [15:0] dshot_srv;
wire [15:0] dshot_in;
reg [15:0] dshot_sm;


`undef F60HZ
`ifdef F60HZ
    wire clk_pd_outer = clk_50;
    wire clk_vco = clk_200;
    wire clk_pd_inner = clk_50;
`else
    wire clk_pd_outer = clk_12_5;  // corrected
    wire clk_vco = clk_50;
    wire clk_pd_inner = clk_12_5;  // corrected
`endif

// instantiate rst_gen
   rst_gen rst_gen1(
    // inputs
    .reset_n        (RESET_N),      // reset from IBUFF
    .clk            (clk_50),        // master clock source
    // outputs
    .sync_rst_n    (sync_rst_n)      // synchronized reset
    );

reg Fg_1q = 1'b0;
reg Fg_2q = 1'b0;

always @ (posedge clk_50) begin
  Fg_1q <= Fg;
  Fg_2q <= Fg_1q;
end

// assign feedback = TP5;
assign feedback = ~Fg_2q;

// =============== inner loop control =========================================

    debounce #(.DELAY(255)) debounce1(
    // Inputs
    .sync_rst_n (sync_rst_n),       // synchronized reset
    .clk            (clk_50),       // master clock source
    .inpt           (feedback),     // raw input signal
    // Outputs
    .outp           (fgIn_sync),     // debounced output
    .dbl            ()
    ) ;

// instantiate counter (div-by-11), used for feedback in outer loop
    down_cnt_sync #(.WIDTH(4)) down_cnt_sync1(
       // inputs
        .sys_clk    (clk_50),           // master clock source
        .clk_in     (fgIn_sync),           // input source clock
        .sync_rst_n (sync_rst_n),       // synchronized reset
        .N          (4'h0b),             // divide ratio
        // outputs
        .count      (),                 // reg count value
        .q_out      (fg60)              // output, ~30Hz
       );
// instantiate freq detector for inner speed control loop
    freq_det #(.WIDTH_TMR(20),.WIDTH_ERR(21)) freq_det1(
       // inputs
       .clk             (clk_50),       // master clock source
       .rstn            (sync_rst_n),   // synchronized reset
       .ref_freq        (Fref6x),       // input ref to inner loop
       .fb_freq         (fgIn_sync),    // input feedback freq
       // outputs
       .sample_out      (sample),
       .err             (Err)           // signed error count
    );
    // put sample in slow clock domain
    pulse_sync pulse_sync1 (
    .in_clk         (clk_50),
    .in_rst         (~sync_rst_n),
    .in_pulse       (sample),
    .out_clk        (clk_pd_inner),
    .out_rst        (),
    .out_pulse      (sample_sync)
    );

// instantiate compensation for inner speed control loop
    lead_lag_comp #(.WIDTH_U(19),
                    .WIDTH_ERR(19),
                    .WIDTH_LIM(19),
                    .WIDTH_FP(31),
                    .fsze(FFPT)) lead_lag_comp1 (
    // input
    .sys_clk                (clk_pd_inner), // system clock
    .rst                    (~sync_rst_n),  // active high reset
    .err                    (pwm_in),       // signed error, Q.0
    .dlim                   (DLIM_sp),      // error limit
    .intlim                 (intlim2),      // integrator limit
    .af                     (java_bldc_Af_xq),           // af = a * (2<<fsze), Q.fsze
    .bf                     (java_bldc_Bf_xq),           // bf = b * (2<<fsze), Q.fsze
    .cf                     (java_bldc_cf_xq),           // cf = c * (2<<fsze), Q.fsze
    .k0                     (java_bldc_K0_xq),           // kf = ko * (2<<fsze), Q.fsze
    .kif                    (java_bldc_kif_xq),          // kif= ki * 2<<<11
    .enable                 (!reset),
    .process_in             (sample_sync),
    .offset                 (java_bldc_M0_xq),           // output offset error
    // outputs
    .uk                     (Me),
    .outp                   (dshot_srv),                 // 16-bit unsigned data
    .sample_out             (done_srv)                   // compatible with dshot
    );

    // instantiate dshot for driving BLHeli ESC
    dshot #(.WDOG(100000) ) dshot1 (
    // input
    .sys_clk        (clk_50),                                // system clock 50Mc
    .rstn           (sync_rst_n),                            // active low reset
    .inpt           (dshot_in),                              // throttle data
    .status         (telem),                                 // telem bit
    .enable         (1'b1),                                  //
    .sample         (done),                                  // start process
    // outputs
    .outp           (pwm1)                                   // serial data out
    );

assign pwm_in = (~Err+1);      // polarity for lead/lag and dshot

// =========== outer loop control =============================================

// instantiate phase detector for outer phase control loop
    phase_det #(.WIDTH_TMR(21), .WIDTH_ERR(22)) phase_det2(
    // inputs
    .clk                (clk_pd_outer), // master clock source
    .sync_rst_n         (sync_rst_n),   // synchronized reset
    .ref_phase          (Fpoly),       // input fref vsync
    .fb_phase           (fg60),         // input feedback phase
    .delay_len          (win_delay_60), // user reg, window delay count
    .width_win          (java_bldc_win_width_xq), // user reg, window width count
    .mod_pi             (select_mod),   // flag to calculate modulo Pi phase
    // outputs
    .sample_out         (sample2),
    .err                (Err2),         // phase error count
    .pd_error           (),             //
    .win                ()
    );

// instantiate compensation for outer PI control loop
    pi_comp #(.WIDTH_U(24), .WIDTH_ERR(22), .WIDTH_LIM(18),
                  .WIDTH_FP(30), .fsze(FFPT)) pi_comp1 (
    // input
    .sys_clk            (clk_pd_outer),   // system clock
    .rst                (~sync_rst_n),    // active high reset
    .err                (Err2),           // signed error, Q.0
    .dlim               (DLIM_ph),        // phase error limit
    .intlim             (intlim1),         // integrator limit
    .ki                 (java_bldc_Ki_xq),             // ki  * (2**fsze), Q.fsze
    .kp                 (java_bldc_Kp_xq),             // kp  * (2**fsze), Q.fsze
    .k0                 (java_bldc_K02_xq),            // k0  * (2**fsze), Q.fsze
    .klpf               (java_bldc_klpf_xq),           // low pass filter pole, gain unity
    .enable             (!reset),
    .process_in         (sample2),
    // outputs
    .uk                 (Me2)
    );

// instantiate fractional divider for 'vco' in outer loop
    frac_divider #(.WIDTH(24), .FSZE(3)) frac_divider1 (
    // inputs
    .sys_clk                (clk_vco),         // 50MHz
    .sync_rst_n             (sync_rst_n),      // active low reset
    .N                      (java_bldc_N1_xq), // integer divider, divide by 24FDB (151515
    .mf                     (Me2),             // fractional divide ...yyyyy.xxx
    // outputs
    .count                  (),
    .q_out                  (Fref6x)
    );

assign select_mod = MODPH;
assign pwm1o = pwm1;          // set pwmo, this is now dshot150
assign pwm2o = 1'b0;          // not used

assign TP1 = Fpoly;           // reference freq
assign TP2 = fg60;            // feedback freq
assign TP3 = fgFb;            // debounced Fg
assign TP4 = Fref6x;          // reference Fg

assign dphase = (~Me2+1);     // polarity for fractional divider
assign fgFb = fgIn_sync;

// scale signed Me[18:0] to unsigned range [48 : 2047], 2000 throttle
// values. Values [0 : 47] are reserved.
localparam MSB = WIDTH-1;
localparam DSHOT_ARM= 16'h0000;
localparam DSHOT100 = 16'd2047;
localparam DSHOT000 = 16'd0048;
localparam DSHOT005 = 16'd0102;
localparam DSHOT010 = 16'd0205;
localparam DSHOT015 = 16'd0307;

localparam strt_IDLE = 0;
localparam strt_arm = 1;
localparam done_arm = 2;
localparam strt_00 = 3;
localparam done_00 = 4;
localparam strt_05 = 5;
localparam done_05 = 6;
localparam strt_10 = 7;
localparam done_10 = 8;
localparam strt_15 = 9;
localparam done_15 =10;
localparam start_servo = 11;

reg [3:0] strt_cnt ;
reg [3:0] startup ;
reg done_sm;
reg cl_loop;
reg telemr;

localparam N750US = 17'd37500; // period of each state
localparam NARM = 16'd1000;    // time to repeat armm (sm counts) 1.5s total
localparam Nstrt= 16'd1500;    // time to spinup before handover to cl_loop, 2s total

reg Fsm_ck;
reg [17:0] Fsm_cnt;
reg [15:0] Narm;

// --------------------------------------------------------------------
assign done = (cl_loop)? done_srv : done_sm;
assign dshot_in = (cl_loop)? dshot_srv : dshot_sm;
assign telem = telemr ;
//
// --------------------------------------------------------------------
// state machine clock 750us
    always @(posedge clk_50) begin
        if(~sync_rst_n) begin
           Fsm_ck <= 1'b0;
           Fsm_cnt <= (N750US >>1 );
        end else begin
           if (Fsm_cnt == 1) begin
              Fsm_cnt <= (N750US >> 1);
              Fsm_ck <= !Fsm_ck;
           end else begin
              Fsm_cnt <= Fsm_cnt - 1'b1;
           end
        end
    end

   always @(posedge Fsm_ck, negedge sync_rst_n ) begin
        if(~sync_rst_n) begin
           done_sm <= 0;
           cl_loop <= 0;
           startup <= strt_IDLE;
           Narm <= NARM;
        end else begin
           case (startup)
             strt_IDLE:
               begin
                 dshot_sm <= strt_arm;
                 telemr <= 1'b0;
                 startup <= strt_arm;
               end
             strt_arm:
               begin
                 dshot_sm = DSHOT_ARM;
                 telemr <= 1'b0;
                 done_sm <= 1'b1;
                 startup <= done_arm;
               end
             done_arm:
               begin
                 done_sm <= 1'b0;
                 if (Narm>0)
                 begin
                    Narm <= Narm - 1;
                    startup <= strt_arm;
                 end
                 else
                    startup <= strt_00;
               end
             strt_00:
               begin
                 dshot_sm = DSHOT000;
                 telemr <= 1'b0;
                 done_sm <= 1'b1;
                 startup <= done_00;
               end
             done_00:
               begin
                 done_sm <= 1'b0;
                 startup <= strt_05;
               end
             strt_05:
               begin
                 dshot_sm <= DSHOT005;
                 telemr <= 1'b0;
                 done_sm <= 1'b1;
                 startup <= done_05;
               end
             done_05:
               begin
                 done_sm <= 1'b0;
                 startup <= strt_10;
               end
             strt_10:
               begin
                 dshot_sm <= DSHOT010;
                 telemr <= 1'b0;
                 done_sm <= 1'b1;
                 startup <= done_10;
             end
             done_10:
               begin
                 done_sm <= 1'b0;
                 startup <= strt_15;
                 Narm <= Nstrt;         // spin up for 2.25sec
               end
             strt_15:
              begin
                 dshot_sm <= DSHOT015;
                 telemr <= 1'b0;
                 done_sm <= 1'b1;
                 startup <= done_15;
               end
             done_15:
               begin
                 done_sm <= 1'b0;
                 if (Narm>0)
                 begin
                    Narm <= Narm - 1;
                    startup <= strt_15;
                 end
                 else
                 begin
                    startup <= start_servo;
                 end
               end
             start_servo:
               begin
                 done_sm <= 1'b0;
                 cl_loop <= 1'b1;
                 startup <= start_servo; // loop here till reset
               end
             default:  ; // do nothing
           endcase
        end
   end

// --------------------------------------------------------------------
// generate Fref_poly
assign Vsync = Fpoly;
assign Fpoly = (mems_vsync_sel)? mems_vsync : Fref_poly ;

    always @ (posedge clk_pd_outer) begin
        intlim1        <= INT_LIM1;
        intlim2        <= INT_LIM2;
        win_delay_360 <= java_bldc_N360_xq - (java_bldc_win_width_xq>>4);    // inner
        win_delay_60  <= `JAVA_BLDC_NREF_DATA - (java_bldc_win_width_xq>>1); // outer
    end

    // Fref counter (generates Fpoly ref clock for outer loop
    always @(posedge clk_pd_outer) begin
        if(~sync_rst_n) begin
           Fref_poly <= 1'b0;
           Ref60_cnt <= (java_bldc_nref_xq >> 1);
        end else begin
           if (Ref60_cnt == 1) begin
              Ref60_cnt <= (java_bldc_nref_xq >> 1);
              Fref_poly <= !Fref_poly;
           end else begin
              Ref60_cnt <= Ref60_cnt - 1'b1;
           end
        end
    end

// ----------------------------------------------------------------------------------
// debug
//
/*
ila_top_bldc (
    .clk(clk_50),               //  1, input wire clk
    .probe0(Me),                // 19
    .probe1(Err2),              // 19
    .probe2(Me2),               // 24
    .probe3(Err),               // 19
    .probe4(pwm_in),            // 19
    .probe5(dshot_srv),         // 16
    .probe6(done)               //  1
);
*/
endmodule
