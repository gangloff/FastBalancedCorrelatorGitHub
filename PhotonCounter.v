`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// PhotonCounter
//
// Count pulses and measure the distribution with respect to some sync signal. The top
// level structure is as follows:
// - photon pulses move through a series of 64 latches clocked at clk2x
// - on sync signal, the data on the latches is saved
//	- there are 64 counters which increment whenever there is a photon in corresponding latch
// - the data is shifted out of the counters when logger starts
// - logger sends the data out to a pipe, to be picked up by the PC
//
//  In many ways, this is a ridiculous design, but it's very fast.
////////////////////////////////////////////////////////////////////////////////

// Modified by Alexei on 06/11/2013

`define nbins 26 // number of time bins in the shift register
`define binsize 16  // size of counters associated with each time bin (for signals slower than photon count rate)

module PhotonCounter(

   // Opal Kelly Host Interface pins:
   input  wire [7:0]  hi_in,
   output wire [1:0]  hi_out,
   inout  wire [15:0] hi_inout,
   
   input  wire 		 clk_in,   // input clock from external PLL is assigned to this pin
   output wire [7:0]  led,      // outputs to LEDs for testing
	
	// incoming and outgoing signals:
   input  wire		  pmt_in1,     // PMT1 pulses
	input  wire		  pmt_in2,     // PMT2 pulses
   input  wire		  sync_in,    // sync clock
   output wire      pmt_out,
   output wire		  reset_out);

///////////////////////////////////////////////////////////////////////////////
// Clock signals
wire clk, clk2x, locked;
///////////////////////////////////////////////////////////////////////////////
// OK wires
wire [15:0] TrigIn40, WireIn00, WireIn01;

wire [15:0] PipeData1;
wire		PipeWrite1, PipeEmpty1, PipeFull1;
wire [15:0] PipeData2;
wire		PipeWrite2, PipeEmpty2, PipeFull2;


wire        ti_clk;
wire [30:0] ok1;
wire [16:0] ok2, ok2_1, ok2_2, ok2_pipe, ok2_ep00;


///////////////////////////////////////////////////////////////////////////////
// Reset signal
wire ok_reset, power_on_reset;	  
reg reset, reset_buf;
assign ok_reset = TrigIn40[0];
SRL16 #(.INIT(16'hFFFF)) reset_sr (.D(1'b0), .CLK(clk), .Q(power_on_reset), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1));
always @(posedge clk) begin
	reset_buf <= ok_reset | ~locked | power_on_reset;
	reset <= reset_buf;
end
	
///////////////////////////////////////////////////////////////////////////////
// PLL loop. Takes 150MHz input on clk_in, generates 150MHz clk and 300MHz clk2x
// This creates a lot of issues later due to clock boundary crossing and such
// But is unfortunatelly necessary - I can't clock the whole design at 300MHz
pll pllloop (.CLKIN_IN(clk_in), .RST_IN(1'b0), .CLKIN_IBUFG_OUT(), .CLK0_OUT(clk), .CLK2X_OUT(clk2x),.LOCKED_OUT(locked));

reg [31:0] clk2x_counter;    // used to be [3:0]
reg [31:0] clk_divide;        // used to be [15:0] before update to larger ClkDividors
reg set_clk, set_clk_det;

always @(posedge clk) begin
	if (reset) clk_divide <= 32'h0;     // used to be 16'h0
	else if (TrigIn40[4]) clk_divide <= {WireIn01[15:0],WireIn00[15:0]};     // used to be [15:0] before update to larger ClkDividors
end

//generate adjustable clock
always @(posedge clk2x) begin
	if(reset | (clk2x_counter == clk_divide) )
		clk2x_counter <= 32'h0;   // used to be 16'h0
	else 
		clk2x_counter <= clk2x_counter + 32'h1;   // used to be 4'h1 before update to larger ClkDividors
	set_clk_det <= (clk2x_counter == clk_divide);
	set_clk <= set_clk_det;
end


///////////////////////////////////////////////////////////////////////////////
// Generate 1clk long sync1x and sync2x signal
reg [3:0] sync_buf;
reg sync2x, notsync2x, sync2x_det, notsync2x_det, sync2x_free, sync2x_det_free, sync2x_1x;
reg sync1x;
reg gmeas_f;
reg [7:0] sync_div, sync_counter;
reg sync_en; 
reg sync_en_det;

// g_2 measurmement sync is on a photon arrival!
//always @(posedge clk)
//	if (reset) gmeas_f <= 1'b0;
//	else if (TrigIn40[3]) gmeas_f <= WireIn00[0];

always @(posedge clk2x)
	sync_buf <= {sync_buf[2:0], sync_in};//gmeas_f ? pmt_in : sync_in};

always @(posedge clk2x)	begin
	sync2x_det <= sync_en & sync_buf[2] & (~sync_buf[3]);
//	notsync2x_det <= ~sync_en | (~sync_buf[2] | sync_buf[3]);
	
	//expand sync if clk is slowed
	sync2x <= sync2x_det | ((~set_clk) & sync2x);
//	notsync2x <= notsync2x_det & (set_clk | notsync2x);

	//generate unimpeded sync signal for counting
	sync2x_det_free <= sync_buf[2] & (~sync_buf[3]);
	sync2x_free <= sync2x_det_free;
	
	sync2x_1x <= sync2x_det;
end

reg sync1x_reg;
always @(posedge clk) begin
	sync1x <= (sync2x_1x | sync2x_det);
end	


always @(posedge clk) 
	if (reset) sync_div <= 8'b0;
	else if (TrigIn40[5]) sync_div <= WireIn00[7:0];

always @(posedge clk2x) begin
	if(reset) sync_counter <= 8'b0;
	else if(sync2x_free) sync_counter <= (sync_counter == sync_div) ? 8'h0 : sync_counter + 8'h1;
	sync_en_det <= (sync_counter == sync_div);
	sync_en <= sync_en_det;
end


///////////////////////////////////////////////////////////////////////////
// Paralelizelize input by sending it down a series of 128 latches

//reg [7:0] photons[`nbins-1:0];
//reg [7:0] photons_sync[`nbins-1:0];

reg [`nbins-1:0] photons1, photons_sync1;
reg [`nbins-1:0] photons2, photons_sync2;

reg [3:0] pmt_buf1;
reg pmt1, pmt_det1, pmt_free1;
reg [3:0] pmt_buf2;
reg pmt2, pmt_det2, pmt_free2;
//reg [8:0] i;

always @(posedge clk2x) begin
	pmt_buf1 <= {pmt_buf1[2:0], pmt_in1};
	pmt_buf2 <= {pmt_buf2[2:0], pmt_in2};
end
	

//watch for pmt edge within set_clk period	
always @(posedge clk2x) begin
	pmt_det1 <= pmt_buf1[2] & (~pmt_buf1[3]);
	pmt_free1 <= pmt_det1;
	pmt1 <= pmt_det1 | ((~set_clk) & pmt1);  //extend pmt for slowed clk
//	if(set_clk) pmt <= pmt_det;
//	else pmt <= pmt | pmt_det;

	pmt_det2 <= pmt_buf2[2] & (~pmt_buf2[3]);
	pmt_free2 <= pmt_det2;
	pmt2 <= pmt_det2 | ((~set_clk) & pmt2);  //extend pmt for slowed clk

end

/*
always @(posedge clk2x)
   if(pmt_free) timebin_counter <= timebin_counter + 8'h1;
//   if(pmt_free) timebin_counter <= ({8{~set_clk}} & (timebin_counter + 8'h1));
*/

always @(posedge clk2x) begin
	if(set_clk) begin
	   photons1 <= {photons1[`nbins-2:0] & {(`nbins-1){~sync2x}}, pmt1};
		photons2 <= {photons2[`nbins-2:0] & {(`nbins-1){~sync2x}}, pmt2};
	end
end
	
always @(posedge clk2x) begin
  if(sync2x & set_clk) begin
     photons_sync1 <= photons1;
	  photons_sync2 <= photons2;
  end
end
  
//////////////////////////////////////////////////////////////////////
// Output a 100ns pulse. Output reset, delayed somewhat, active until PMT_in falls
reg [3:0] pmt_strecher;
reg pmt1x;

always @(posedge clk)
	pmt1x <= pmt_free1 | pmt_det1;

always @(posedge clk)
   if (pmt1x & ~|pmt_strecher) pmt_strecher <= 4'hF;
	else if (|pmt_strecher) pmt_strecher <= pmt_strecher - 4'b1;

assign pmt_out = |pmt_strecher;

assign reset_out = 1'b0;

////////////////////////////////////////////////////////////////////////////////					  
// Log photons on sync
wire [15:0] phcount1;
wire [15:0] phcount2;
wire count_ld1;
wire count_ld2;
wire [((`nbins-1)*(`binsize)-1):0] databus1;
wire [((`nbins-1)*(`binsize)-1):0] databus2;
reg clear_photons, clear_det;

phcounter counter1[(`nbins-1):0] (.count_o({databus1, phcount1[`binsize-1:0]}), .clk_i(clk), .reset_i(reset), .photon_i(photons_sync1), .sync_i(sync1x), .ld_i(count_ld1), .count_i({`binsize'h0, databus1}), .clear_i(clear_photons | TrigIn40[1] | TrigIn40[5])); //MUX
phcounter counter2[(`nbins-1):0] (.count_o({databus2, phcount2[`binsize-1:0]}), .clk_i(clk), .reset_i(reset), .photon_i(photons_sync2), .sync_i(sync1x), .ld_i(count_ld2), .count_i({`binsize'h0, databus2}), .clear_i(clear_photons | TrigIn40[1] | TrigIn40[5])); //MUX

// Uncomment the following line for binsize < 16
//assign phcount[15:`binsize] = {(16-`binsize){1'b0}};

///////////////////////////////////////////////////////////////////////////////
// Generate an data output trigger. A simple counter that activates a trigger
// signal whenever it reaches a certain value
reg trigger_f, log_f1, log_f2;
reg [15:0] max_count_f;
reg [31:0] counter_f;
wire trigger;

assign trigger = (counter_f[31:16] > max_count_f) & ~trigger_f;

//	Running at 150Mhz, this gives ~100ms integration time
always @(posedge clk)
	if (reset) max_count_f <= 16'hE5;  //229
	else if (TrigIn40[1]) max_count_f <= WireIn00;

always @(posedge clk)
	if (reset | TrigIn40[1] | TrigIn40[5]) counter_f <=  32'h0;
	else 
		if (trigger_f) counter_f <= 32'h0;
		else counter_f <= counter_f + 32'h1;

//always @(posedge clk) trigger_f <= TrigIn40[1];
always @(posedge clk) begin
	trigger_f <= trigger;
	log_f1 <= trigger_f & PipeEmpty1;
	log_f2 <= trigger_f & PipeEmpty2;
	clear_det <= trigger_f;
	clear_photons <= clear_det;
end

///////////////////////////////////////////////////////////////////////////////
// Send data out
logger log1(.data_o(PipeData1), .ld_o(count_ld1), .wr_o(PipeWrite1), .clk_i(clk), .reset_i(reset), .start_i(log_f1), .data_i(phcount1));
logger log2(.data_o(PipeData2), .ld_o(count_ld2), .wr_o(PipeWrite2), .clk_i(clk), .reset_i(reset), .start_i(log_f2), .data_i(phcount2));

///////////////////////////////////////////////////////////////////////////////
// led
//reg sync_diode, pmt_diode, ld_diode;
//always @(posedge clk) 
//	if (sync1x) sync_diode = ~sync_diode;
//always @(posedge clk) 
//	if (pmt1x)	pmt_diode = ~pmt_diode;
//always @(posedge clk) 
//	if (count_ld) ld_diode = ~ld_diode;

//assign led = ~{sync_diode, ~sync_diode, pmt_diode, ~pmt_diode, PipeFull, PipeEmpty, ld_diode, 1'b0};
assign led = ~{8'b0};//clk_divide,gmeas_f,1'b0,delay_latch,removefromqueue_f};//~{clk_divide,4'b0};//,removefromqueue_f,1'b0,gmeas_f};

///////////////////////////////////////////////////////////////////////////////
// Okie, now the Opal Kelly interface

parameter nOK = 2;
wire [17*nOK-1:0]  ok2x;

okHost okHI(.hi_in(hi_in), .hi_out(hi_out), .hi_inout(hi_inout),
                     .ti_clk(ti_clk), .ok1(ok1), .ok2(ok2));

wire        _piperead1;
wire [15:0] _pipedata1;
wire        _piperead2;
wire [15:0] _pipedata2;
							 
pipefifo_v6 pipeoutfifo1 (.din(PipeData1), .rd_clk(ti_clk), .rd_en(_piperead1),
								.wr_clk(clk), .wr_en(PipeWrite1), .dout(_pipedata1),
								.empty(PipeEmpty1), .full(PipeFull1));

pipefifo_v6 pipeoutfifo2 (.din(PipeData2), .rd_clk(ti_clk), .rd_en(_piperead2),
								.wr_clk(clk), .wr_en(PipeWrite2), .dout(_pipedata2),
								.empty(PipeEmpty2), .full(PipeFull2));								
							 
okPipeOut pipeOuta0 (.ok1(ok1), .ok2(ok2x[0*17 +: 17]), .ep_addr(8'ha0),
                     .ep_datain(_pipedata1), .ep_read(_piperead1));
							
okPipeOut pipeOutaf (.ok1(ok1), .ok2(ok2x[1*17 +: 17]), .ep_addr(8'haf),
                    .ep_datain(_pipedata2), .ep_read(_piperead2));							

okTriggerIn ep40 (.ok1(ok1), .ep_addr(8'h40), .ep_clk(clk), .ep_trigger(TrigIn40));//.ok2(ok2), 
okWireIn ep00 (.ok1(ok1), .ep_addr(8'h00), .ep_dataout(WireIn00));//.ok2(ok2), 
okWireIn ep01 (.ok1(ok1), .ep_addr(8'h01), .ep_dataout(WireIn01));
//okWireOut ep20 (.ok1(ok1), .ok2(ok2_ep20), .ep_addr(8'h14), .ep_datain(WireOut20));

okWireOR #(.N(nOK)) okor (.ok2(ok2), .ok2s(ok2x));

endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////
// phcounter
//
// Internals of a single photon counter. A counter, and a output register. The output is constantly shifted
// out, but loaded only on ld signal.
//////////////////////////////////////////////////////////////////////////////////////////////////////
module phcounter(count_o, clk_i, reset_i, photon_i, sync_i, ld_i, count_i, clear_i);
	output reg	 	[`binsize-1:0]	count_o;
	input wire 							clk_i;
	input wire 							reset_i;
	input wire							photon_i;
	input wire 							sync_i;
	input wire 							ld_i;
	input wire							clear_i;
	input wire		[`binsize-1:0]	count_i;
	
	reg		full_flag;
	reg [`binsize-1:0] 	count_f;
	reg 		photon_f, sync_f, syncdly_f;

	// Latch	 for speed (if you do not latch sync, it will fail - photon signal sometimes misses by a clock at 333MHz)
	always @(posedge clk_i) begin
		photon_f <= photon_i;
		syncdly_f <= sync_i;
      sync_f <= syncdly_f;
	end

	always @(posedge clk_i)
		if (reset_i | clear_i) count_f <= `binsize'h0;
		else if (sync_f & photon_f &(~full_flag)) count_f <= count_f + `binsize'b1;
		
	always @(posedge clk_i)
		if (reset_i | clear_i) full_flag <= 1'b0;
		else if (&count_f) full_flag <= 1'b1;
				
		
	always @(posedge clk_i)
		if (reset_i) count_o <= `binsize'h0;
		else 
			if (ld_i) count_o <= count_f;
			else count_o <= count_i;
 
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////
// logger circuit
//
// Logger module. Itis started by start siggnal, and proceeds to load data and send it to the pipe.
//////////////////////////////////////////////////////////////////////////////////////////////////////
module logger(data_o, ld_o, wr_o, clk_i, reset_i, start_i, data_i);
	output reg [15:0] data_o;
	output reg wr_o;
	output reg ld_o;
	input wire clk_i;
	input wire reset_i;
	input wire start_i;
	input wire [15:0] data_i; 


	// State machine definitions
	localparam IDLE 		= 2'h0;
	localparam HEADER 	= 2'h1;
	localparam STREAM 	= 2'h2;
	localparam TERMIN 	= 2'h3;
	
	// This line makes synthesis happy
	// synthesis attribute INIT of state_f is "R"		
	reg 	[1:0] 	state_f;
	reg		[`nbins-1:0]	samplecount_f;

	// The combinatorial part, defines motion through states
	function [1:0] next_state;
		input [1:0] 	state;		 	
		input 			start;
		input 			done;
		
		case (state)
			IDLE: 
				if (start) next_state = HEADER;
				else next_state = IDLE;
			HEADER:
				next_state = STREAM;
			STREAM:
				if (done) next_state = TERMIN;
				else next_state = STREAM;
			TERMIN:
				next_state = IDLE;
			default:
				next_state = IDLE;
		endcase
	endfunction

	// The sequential part - defines actions at a given state
	always @(posedge clk_i)
		if (reset_i) begin
			state_f <= 2'h0;
			wr_o <= 1'b0;
			data_o <= 16'h0;
			samplecount_f <= (1<<`nbins) - 1;        // 01111111...11     number of ones = nbins
		end else begin 
			state_f <= next_state(state_f, start_i, ~samplecount_f[0]);
			case (state_f)
				HEADER: begin
					ld_o <= 1'b0;
					wr_o <= 1'b1;
					data_o <= 16'hFEED;
					samplecount_f <= samplecount_f >> 1;
				end
				STREAM: begin
					samplecount_f <= samplecount_f >> 1;
					data_o <= {data_i[7:0], data_i[15:8]};
				end
				TERMIN: begin
					data_o <= 16'h0FED;
				end
				default: begin
					ld_o <= start_i;
					wr_o <= 1'b0;
					samplecount_f <= (1<<`nbins) - 1;
					data_o <= 16'h00;
				end
					
			endcase
		end


endmodule
