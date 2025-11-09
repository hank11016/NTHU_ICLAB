//==================================================================================================
//  Note:          Use only for teaching materials of IC Design Lab, NTHU.
//  Copyright: (c) 2025 Vision Circuits and Systems Lab, NTHU, Taiwan. ALL Rights Reserved.
//==================================================================================================

module enigma(clk, srst_n, load, encrypt, crypt_mode, table_idx, code_in, code_out, code_valid);
input clk;         // clock 
input srst_n;      // synchronous reset (active low)
input load;        // load control signal (level sensitive). 0/1: inactive/active
input encrypt;     // encrypt control signal (level sensitive). 0/1: inactive/active
input crypt_mode;  // 0: encrypt; 1:decrypt;
input [2-1:0] table_idx; // table_idx indicates which rotor to be loaded 
						             // 2'b00: rotorA
						             // 2'b01: plugboard
						             // 2'b10: rotorB
input [6-1:0] code_in;	// When load is active, then code_in is input of rotors. 
						// When encrypy is active, then code_in is input of code words.
output reg [6-1:0] code_out;   // encrypted code word
output reg code_valid;         // 0: non-valid code_out; 1: valid code_out 

// ======== Declarations ======== //

wire XOR_whitening_en;
wire [5:0] XOR_whitening_in;
wire [5:0] XOR_whitening_out;
wire XOR_whitening_out_valid;

wire [5:0] rotorA_ls_count, rotorA_for_dout;
wire rotorA_for_out_valid;
wire rotorA_we;

wire [5:0] for_bit_sw_out, inv_bit_sw_out;

wire [5:0] rotorA_inv_dout;
wire rotorA_inv_out_valid;

reg load_ff, encrypt_ff, crypt_mode_ff;
reg [1:0] table_idx_ff;
reg [5:0] code_in_ff;

wire [5:0] rotorB_for_din, rotorB_for_dout;
wire [5:0] rotorB_inv_dout, rotorB_inv_din;
wire rotorB_for_in_valid, rotorB_inv_in_valid;
wire rotorB_for_out_valid, rotorB_inv_out_valid;

wire bit_sw_inv_out_valid, bit_sw_inv_in_valid;

wire [5:0] XOR_whitening_real_dout;
wire [5:0] bit_sw_dout;

wire [5:0] rotorB_sel_dout_ls0, rotorB_sel_dout_ls1, rotorB_sel_dout_ls2, rotorB_sel_dout_ls3;

wire [1:0] encrypt_mode, decrypt_mode;
wire [5:0] rotorB_for_din_ls0, rotorB_for_din_ls1, rotorB_for_din_ls2, rotorB_for_din_ls3;
reg rotorB_inv_out_valid_ff;
reg [5:0] rotorB_inv_dout_ff;

wire [1:0] for_mode_ff;
wire [5:0] inv_text_in, inv_text_out;


// ======== Input ff ========== //


always @ (posedge clk) begin
  {load_ff, encrypt_ff, crypt_mode_ff, table_idx_ff, code_in_ff} <= {load, encrypt, crypt_mode, table_idx, code_in};
end


// ============ Plugboard ============= //

wire [5:0] plugboard_for_din, plugboard_inv_din; 
wire [5:0] plugboard_for_dout, plugboard_inv_dout;
wire plugboard_for_out_valid, plugboard_inv_out_valid;

assign plugboard_for_din = code_in_ff;
assign plugboard_inv_din = rotorA_inv_dout;

plugboard U_plugboard(
	.clk(clk),
	.srst_n(srst_n),
	.plugboard_for_din(plugboard_for_din), // = code_in_ff
	.plugboard_inv_din(plugboard_inv_din), // from rotorA
	.plugboard_for_dout(plugboard_for_dout),
	.plugboard_inv_dout(plugboard_inv_dout),
	.table_idx_ff(table_idx_ff),
	.load_ff(load_ff),
	.encrypt_ff(encrypt_ff),
	.plugboard_inv_in_valid(rotorA_inv_out_valid),
	.plugboard_for_out_valid(plugboard_for_out_valid),
	.plugboard_inv_out_valid(plugboard_inv_out_valid)
);



// ======== Rotor A (forward & inverse)========== //
wire [5:0] rotorA_for_dout0, rotorA_for_dout1, rotorA_for_dout2, rotorA_for_dout3;


rotorA_whole #(
  .LS_COUNT_DELAY(2)
)U_rotorA_whole(
  // forward
  .clk(clk),
  .srst_n(srst_n),
  .rotorA_for_din(plugboard_for_dout), // write input (=code_in_ff in part123)
  .code_in_ff(code_in_ff),
  .rotorA_for_dout0(rotorA_for_dout0), // read output for address + 0
  .rotorA_for_dout1(rotorA_for_dout1), // read output for address + 1
  .rotorA_for_dout2(rotorA_for_dout2), // read output for address + 2
  .rotorA_for_dout3(rotorA_for_dout3), // read output for address + 3
  .rotorA_for_out_valid(rotorA_for_out_valid),

  // control signals
  .table_idx_ff(table_idx_ff),
  .crypt_mode_ff(crypt_mode_ff),
  .encrypt_ff(plugboard_for_out_valid),
  .load_ff(load_ff),
  .inverse_path_LSB(inv_text_out[1:0]),// the least significant 2-bit of the input of the inverse pass


  // inverse
  .rotorA_inv_din(inv_text_out), // write input (=code_in_ff in part123)
  .rotorA_inv_dout_real(rotorA_inv_dout), // read output

  // control signals
	.rotorA_inv_in_valid(bit_sw_inv_out_valid),
	.rotorA_inv_out_valid(rotorA_inv_out_valid)
);

// ========= Bit switching (forward) ========= //
//reg [5:0] rotorB_for_din_ff, rotorB_inv_dout_ff;

wire [1:0] for_text_out_mode;
wire bit_sw_for_out_valid;
wire [5:0] for_text_out_mode00_ls0, for_text_out_mode00_ls1, for_text_out_mode00_ls2, for_text_out_mode00_ls3;
wire [5:0] for_text_out_mode01_ls0, for_text_out_mode01_ls1, for_text_out_mode01_ls2, for_text_out_mode01_ls3;
wire [5:0] for_text_out_mode10_ls0, for_text_out_mode10_ls1, for_text_out_mode10_ls2, for_text_out_mode10_ls3;
wire [5:0] for_text_out_mode11_ls0, for_text_out_mode11_ls1, for_text_out_mode11_ls2, for_text_out_mode11_ls3;

bit_sw_for_mode U_bit_sw_for_mode(
	.clk(clk),
  .srst_n(srst_n),
	.for_text_in(rotorA_for_dout0),
	.for_text_out_mode(for_text_out_mode),

  // control signals
  .bit_sw_for_in_valid(rotorA_for_out_valid), 
  .bit_sw_for_out_valid(bit_sw_for_out_valid)
); 

bit_sw_for_top U_bit_sw_for_top (
    // inputs
    .for_text_in_ls0 (rotorA_for_dout0),
    .for_text_in_ls1 (rotorA_for_dout1),
    .for_text_in_ls2 (rotorA_for_dout2),
    .for_text_in_ls3 (rotorA_for_dout3),

    // outputs
    .for_text_out_mode00_ls0 (for_text_out_mode00_ls0),
    .for_text_out_mode00_ls1 (for_text_out_mode00_ls1),
    .for_text_out_mode00_ls2 (for_text_out_mode00_ls2),
    .for_text_out_mode00_ls3 (for_text_out_mode00_ls3),

    .for_text_out_mode01_ls0 (for_text_out_mode01_ls0),
    .for_text_out_mode01_ls1 (for_text_out_mode01_ls1),
    .for_text_out_mode01_ls2 (for_text_out_mode01_ls2),
    .for_text_out_mode01_ls3 (for_text_out_mode01_ls3),

    .for_text_out_mode10_ls0 (for_text_out_mode10_ls0),
    .for_text_out_mode10_ls1 (for_text_out_mode10_ls1),
    .for_text_out_mode10_ls2 (for_text_out_mode10_ls2),
    .for_text_out_mode10_ls3 (for_text_out_mode10_ls3),

    .for_text_out_mode11_ls0 (for_text_out_mode11_ls0),
    .for_text_out_mode11_ls1 (for_text_out_mode11_ls1),
    .for_text_out_mode11_ls2 (for_text_out_mode11_ls2),
    .for_text_out_mode11_ls3 (for_text_out_mode11_ls3)
);


// ========= mode selection ========= //
assign encrypt_mode = for_text_out_mode;
assign decrypt_mode = rotorB_inv_dout_ff[1:0];

bit_sw_sel U_bit_sw_sel (
  .clk(clk),
  // data inputs 
  .din_mode00_ls0 (for_text_out_mode00_ls0),
  .din_mode01_ls0 (for_text_out_mode01_ls0),
  .din_mode10_ls0 (for_text_out_mode10_ls0),
  .din_mode11_ls0 (for_text_out_mode11_ls0),

  .din_mode00_ls1 (for_text_out_mode00_ls1),
  .din_mode01_ls1 (for_text_out_mode01_ls1),
  .din_mode10_ls1 (for_text_out_mode10_ls1),
  .din_mode11_ls1 (for_text_out_mode11_ls1),

  .din_mode00_ls2 (for_text_out_mode00_ls2),
  .din_mode01_ls2 (for_text_out_mode01_ls2),
  .din_mode10_ls2 (for_text_out_mode10_ls2),
  .din_mode11_ls2 (for_text_out_mode11_ls2),

  .din_mode00_ls3 (for_text_out_mode00_ls3),
  .din_mode01_ls3 (for_text_out_mode01_ls3),
  .din_mode10_ls3 (for_text_out_mode10_ls3),
  .din_mode11_ls3 (for_text_out_mode11_ls3),

  // selected outputs
  .dout_ls0 (rotorB_for_din_ls0),
  .dout_ls1 (rotorB_for_din_ls1),
  .dout_ls2 (rotorB_for_din_ls2),
  .dout_ls3 (rotorB_for_din_ls3),

  // control signal
  .enable        (rotorB_for_out_valid),
  .crypt_mode_ff (crypt_mode_ff),
  .encrypt_mode  (encrypt_mode),
  .decrypt_mode  (decrypt_mode)
);

// ============ rotorB ============= //
assign rotorB_for_in_valid = bit_sw_for_out_valid; // rotorA, bit sw, rotorB in same cycle
assign rotorB_inv_in_valid = XOR_whitening_out_valid;
assign rotorB_inv_din = XOR_whitening_real_dout;

// wire [5:0] rotorB_for_din_ls0, rotorB_for_din_ls1, rotorB_for_din_ls2, rotorB_for_din_ls3;
wire [5:0] rotorB_for_dout_ls0, rotorB_for_dout_ls1, rotorB_for_dout_ls2, rotorB_for_dout_ls3;

rotorB_for_4r1w U_rotorB_for_4r1w(
	.clk(clk),
	.rotorB_for_din_ls0(rotorB_for_din_ls0),
  .rotorB_for_din_ls1(rotorB_for_din_ls1),
  .rotorB_for_din_ls2(rotorB_for_din_ls2), 
  .rotorB_for_din_ls3(rotorB_for_din_ls3),

  .rotorB_for_dout_ls0(rotorB_for_dout_ls0),
  .rotorB_for_dout_ls1(rotorB_for_dout_ls1),
  .rotorB_for_dout_ls2(rotorB_for_dout_ls2),
  .rotorB_for_dout_ls3(rotorB_for_dout_ls3),

  // input for rot_mode decision
  .rotorB_for_dout_if_encrypt(rotorB_for_dout_ls0),

  // inverse path
  .rotorB_inv_din        (rotorB_inv_din),
  .rotorB_inv_dout       (rotorB_inv_dout),

  // control signals
  .rotorB_for_in_valid   (rotorB_for_in_valid),
  .rotorB_inv_in_valid   (rotorB_inv_in_valid),
  .code_in_ff            (code_in_ff),
  .table_idx_ff          (table_idx_ff),
  .crypt_mode_ff         (crypt_mode_ff),
  .load_ff               (load_ff),

  // output control signals
  .rotorB_for_out_valid  (rotorB_for_out_valid),
  .rotorB_inv_out_valid  (rotorB_inv_out_valid)
);

// ========= XOR Whitening & sel ========= //

wire [5:0] XOR_whitening_out0, XOR_whitening_out1, XOR_whitening_out2, XOR_whitening_out3;

assign XOR_whitening_en = rotorB_for_out_valid;

xor_whitening_4merged_sel U_xor_whitening_4merged_sel(
	.clk(clk),
	.srst_n(srst_n),
  .XOR_whitening_in_ls0(rotorB_for_dout_ls0),
  .XOR_whitening_in_ls1(rotorB_for_dout_ls1),
  .XOR_whitening_in_ls2(rotorB_for_dout_ls2),
  .XOR_whitening_in_ls3(rotorB_for_dout_ls3),
  .dout(XOR_whitening_real_dout),
  .crypt_mode_ff(crypt_mode_ff),       
  .bit_sw_inv_dout(inv_text_out),
  .XOR_whitening_en(XOR_whitening_en),
  .XOR_whitening_out_valid(XOR_whitening_out_valid)
);

/*
xor_whitening_4merged U_xor_whitening_4merged(
	.clk(clk),
	.srst_n(srst_n),
  .XOR_whitening_in_ls0(rotorB_for_dout_ls0),
  .XOR_whitening_in_ls1(rotorB_for_dout_ls1),
  .XOR_whitening_in_ls2(rotorB_for_dout_ls2),
  .XOR_whitening_in_ls3(rotorB_for_dout_ls3),
	.XOR_whitening_out_ls0(XOR_whitening_out0),
	.XOR_whitening_out_ls1(XOR_whitening_out1),
	.XOR_whitening_out_ls2(XOR_whitening_out2),
	.XOR_whitening_out_ls3(XOR_whitening_out3),
	
  .XOR_whitening_en(XOR_whitening_en),
  .XOR_whitening_out_valid(XOR_whitening_out_valid)
);

// ======== rotorA select output to inv rotorB ========== //

rotorA_sel U_rotorA_sel(
  .din_ls0(XOR_whitening_out0),
  .din_ls1(XOR_whitening_out1),
  .din_ls2(XOR_whitening_out2),
  .din_ls3(XOR_whitening_out3),
  .dout(XOR_whitening_real_dout),

  // control signals
  .crypt_mode_ff(crypt_mode_ff),       
  .bit_sw_inv_dout(inv_text_out)
);
*/

// ========== rotorB FF (inverse) =========== //
always @ (posedge clk) begin
  rotorB_inv_out_valid_ff <= rotorB_inv_out_valid;
  rotorB_inv_dout_ff      <= rotorB_inv_dout;
end

// ======== Bit switching (inverse)========== //
assign inv_text_in = rotorB_inv_dout_ff;
assign bit_sw_inv_in_valid = rotorB_inv_out_valid_ff;

bit_sw_inv U_bit_sw_inv(
	.clk(clk),
	.srst_n(srst_n),
	.inv_text_in(inv_text_in),
  .for_text_out_mode(for_text_out_mode),
	.crypt_mode_ff(crypt_mode_ff),
	.sw_enable(rotorB_inv_out_valid_ff),
	.inv_text_out(inv_text_out),
  .bit_sw_inv_in_valid(bit_sw_inv_in_valid), 
  .bit_sw_inv_out_valid(bit_sw_inv_out_valid)
);

// ======== Rotor A (inverse)========== //

// 

// ============ Final result =============== //

always @ (posedge clk) begin
  code_out    <= plugboard_inv_dout;
  code_valid  <= plugboard_inv_out_valid;
end

endmodule

//==============================================================//
//======================= Modules ==============================//
//==============================================================//

module sram64x6 (
  input        clk,
  input        we,    // write enable (1:write, 0:read)
  input  [5:0] addr, 
  input  [5:0] din,
  output [5:0] dout
);
  reg [5:0] mem [0:63];
  //reg [5:0] addr_r;

  always @(posedge clk) begin
    if (we) mem[addr] <= din;
    //addr_r <= addr;
  end
  assign dout = mem[addr];

endmodule

module sram64x6_rotorA_for (
  input        clk,
  input        load,    // load enable
  input  [5:0] addr,
  input  [5:0] din,
  output [5:0] dout_add0,
  output [5:0] dout_add1,
  output [5:0] dout_add2,
  output [5:0] dout_add3
);

  // addr is the load addr
  // read addr0, addr1, ..., addr3

  reg [5:0] mem [0:66];
  //reg [5:0] addr_r;
  integer i;

  always @(posedge clk) begin
    if (load) begin
		mem[63] <= din;
		for (i=0; i<63; i=i+1)
			mem[i] <= mem[i+1];
	  end
  end

  always @* begin
    mem[64] = mem[0];
    mem[65] = mem[1];
    mem[66] = mem[2];
  end
  /*
  wire [5:0] addr0, addr1, addr2, addr3;
  assign addr0 = addr;
  assign addr1 = addr + 1;
  assign addr2 = addr + 2;
  assign addr3 = addr + 3;
  */
  assign dout_add0 = mem[addr];
  assign dout_add1 = mem[addr+1];
  assign dout_add2 = mem[addr+2];
  assign dout_add3 = mem[addr+3];

endmodule

module rotorA_whole #(
  parameter LS_COUNT_DELAY = 2
)(
  // forward
  input clk,
  input srst_n,
  input [5:0] rotorA_for_din, // read request (=code_in_ff in part123)
  input [5:0] code_in_ff,
  output reg [5:0] rotorA_for_dout0, // read output for address + 0
  output reg [5:0] rotorA_for_dout1, // read output for address + 1
  output reg [5:0] rotorA_for_dout2, // read output for address + 2
  output reg [5:0] rotorA_for_dout3, // read output for address + 3
  output reg rotorA_for_out_valid,

  // control signals
  input [1:0] table_idx_ff,
  input crypt_mode_ff,
  input encrypt_ff,
  input load_ff,
  input [1:0] inverse_path_LSB, // the least significant 2-bit of the input of the inverse pass


  // inverse
  input [5:0] rotorA_inv_din, 
  output [5:0] rotorA_inv_dout_real, // read output

  // control signals
  input rotorA_inv_in_valid,
  output reg rotorA_inv_out_valid
);


// Declarations
reg rotorA_we;
reg [5:0] mem [0:66];
reg [5:0] rotorA_ls_count_next, rotorA_ls_count;
reg [5:0] rotorA_inv_dout; // read output
wire [5:0] rotorA_for_din_ls;
integer i;

// write enable
always @*
 rotorA_we = (table_idx_ff == 2'b00 && load_ff == 1'b1);

// load data
always @ (posedge clk)begin
  if (rotorA_we) begin
    mem[63]  <= code_in_ff;
    for(i=0; i<63; i=i+1)
      mem[i] <= mem[i+1];
  end
end

// for the prediction part
always @* begin
  for(i=0; i<3; i=i+1)
    mem[i+64] = mem[i];
end

// we have to know which cycles have a valid rotor output
always @ (posedge clk) begin
  rotorA_for_out_valid <= encrypt_ff;
end

// The left shifted read address
assign rotorA_for_din_ls = rotorA_for_din + rotorA_ls_count_next;

always @ (posedge clk) begin
  rotorA_for_dout0 <= mem[rotorA_for_din_ls];
  rotorA_for_dout1 <= mem[rotorA_for_din_ls+1];
  rotorA_for_dout2 <= mem[rotorA_for_din_ls+2];
  rotorA_for_dout3 <= mem[rotorA_for_din_ls+3];
end


// ======== rotor A left shift count ========
// procedure : 
// 1. If rotorA_we_next : load mode, then increment
// 2. If rotorA_for_out_valid, ls count += inverse_path_LSB/ rotorA_for_dout0[1:0]
//    (1) But due to the pipelined structure, we get inverse_path_LSB after N cycles
//    (2) N = 1 in version1, N = 2 in version2
//    (3) Update the ls count by current rotorA_for_dout0/ inverse_path_LSB

always @ * begin
	if (rotorA_we) rotorA_ls_count_next = rotorA_ls_count + 1;
	else if (rotorA_for_out_valid) rotorA_ls_count_next = (crypt_mode_ff) ? rotorA_ls_count + inverse_path_LSB : rotorA_ls_count + rotorA_for_dout0[1:0]; // 0: encrypt; 1:decrypt;
  else rotorA_ls_count_next = rotorA_ls_count;
end

// table's address & ls_count FF
always @ (posedge clk) begin
	if (~srst_n) begin
		rotorA_ls_count <= 6'd0;
	end
	else begin
		rotorA_ls_count <= rotorA_ls_count_next;
	end
end

// ================ Inverse ================ //

// Declarations
reg [5:0]  rotorA_addr_inv, rotorA_din_inv;
reg [5:0] rotorA_addr_inv_next, rotorA_din_inv_next;
wire [5:0] rotorA_inv_sram_dout;
reg [LS_COUNT_DELAY*6-1:0] ls_count_buffer, ls_count_buffer_next;
wire [5:0]ls_count_ff_encrypt, ls_count_ff_decrypt, ls_count_sel; 


// current left shift count need to be add to result
// for encrypt, ls_count must wait 2 cycles
// for decrypt, ls_count just need to wait 1 cycle

always @ (posedge clk) begin
  ls_count_buffer <= {ls_count_buffer[LS_COUNT_DELAY*6-1-6:0], rotorA_ls_count};
  rotorA_inv_out_valid <= rotorA_inv_in_valid;
end

assign ls_count_ff_encrypt = ls_count_buffer[LS_COUNT_DELAY*6-1-:6];
assign ls_count_ff_decrypt = ls_count_buffer[LS_COUNT_DELAY*6-1-6-:6];
assign ls_count_sel = (crypt_mode_ff) ? ls_count_ff_decrypt : ls_count_ff_encrypt;
assign rotorA_inv_dout_real = rotorA_inv_dout - ls_count_sel;

// get the inverse output
always @(posedge clk) begin
  case (rotorA_inv_din)
    mem[0]  : rotorA_inv_dout <= 6'd0;
    mem[1]  : rotorA_inv_dout <= 6'd1;
    mem[2]  : rotorA_inv_dout <= 6'd2;
    mem[3]  : rotorA_inv_dout <= 6'd3;
    mem[4]  : rotorA_inv_dout <= 6'd4;
    mem[5]  : rotorA_inv_dout <= 6'd5;
    mem[6]  : rotorA_inv_dout <= 6'd6;
    mem[7]  : rotorA_inv_dout <= 6'd7;
    mem[8]  : rotorA_inv_dout <= 6'd8;
    mem[9]  : rotorA_inv_dout <= 6'd9;
    mem[10] : rotorA_inv_dout <= 6'd10;
    mem[11] : rotorA_inv_dout <= 6'd11;
    mem[12] : rotorA_inv_dout <= 6'd12;
    mem[13] : rotorA_inv_dout <= 6'd13;
    mem[14] : rotorA_inv_dout <= 6'd14;
    mem[15] : rotorA_inv_dout <= 6'd15;
    mem[16] : rotorA_inv_dout <= 6'd16;
    mem[17] : rotorA_inv_dout <= 6'd17;
    mem[18] : rotorA_inv_dout <= 6'd18;
    mem[19] : rotorA_inv_dout <= 6'd19;
    mem[20] : rotorA_inv_dout <= 6'd20;
    mem[21] : rotorA_inv_dout <= 6'd21;
    mem[22] : rotorA_inv_dout <= 6'd22;
    mem[23] : rotorA_inv_dout <= 6'd23;
    mem[24] : rotorA_inv_dout <= 6'd24;
    mem[25] : rotorA_inv_dout <= 6'd25;
    mem[26] : rotorA_inv_dout <= 6'd26;
    mem[27] : rotorA_inv_dout <= 6'd27;
    mem[28] : rotorA_inv_dout <= 6'd28;
    mem[29] : rotorA_inv_dout <= 6'd29;
    mem[30] : rotorA_inv_dout <= 6'd30;
    mem[31] : rotorA_inv_dout <= 6'd31;
    mem[32] : rotorA_inv_dout <= 6'd32;
    mem[33] : rotorA_inv_dout <= 6'd33;
    mem[34] : rotorA_inv_dout <= 6'd34;
    mem[35] : rotorA_inv_dout <= 6'd35;
    mem[36] : rotorA_inv_dout <= 6'd36;
    mem[37] : rotorA_inv_dout <= 6'd37;
    mem[38] : rotorA_inv_dout <= 6'd38;
    mem[39] : rotorA_inv_dout <= 6'd39;
    mem[40] : rotorA_inv_dout <= 6'd40;
    mem[41] : rotorA_inv_dout <= 6'd41;
    mem[42] : rotorA_inv_dout <= 6'd42;
    mem[43] : rotorA_inv_dout <= 6'd43;
    mem[44] : rotorA_inv_dout <= 6'd44;
    mem[45] : rotorA_inv_dout <= 6'd45;
    mem[46] : rotorA_inv_dout <= 6'd46;
    mem[47] : rotorA_inv_dout <= 6'd47;
    mem[48] : rotorA_inv_dout <= 6'd48;
    mem[49] : rotorA_inv_dout <= 6'd49;
    mem[50] : rotorA_inv_dout <= 6'd50;
    mem[51] : rotorA_inv_dout <= 6'd51;
    mem[52] : rotorA_inv_dout <= 6'd52;
    mem[53] : rotorA_inv_dout <= 6'd53;
    mem[54] : rotorA_inv_dout <= 6'd54;
    mem[55] : rotorA_inv_dout <= 6'd55;
    mem[56] : rotorA_inv_dout <= 6'd56;
    mem[57] : rotorA_inv_dout <= 6'd57;
    mem[58] : rotorA_inv_dout <= 6'd58;
    mem[59] : rotorA_inv_dout <= 6'd59;
    mem[60] : rotorA_inv_dout <= 6'd60;
    mem[61] : rotorA_inv_dout <= 6'd61;
    mem[62] : rotorA_inv_dout <= 6'd62;
    mem[63] : rotorA_inv_dout <= 6'd63;
    default : rotorA_inv_dout <= rotorA_inv_dout; // 保持前值
  endcase
end

endmodule

module rotorA_inverse #(
  parameter LS_COUNT_DELAY = 2
)(
  input clk,
  input srst_n,
  input [5:0] rotorA_inv_din, 
  input  [5:0] rotorA_ls_count, // the left shift count
  output [5:0] rotorA_inv_dout, // read output

  // control signals
  input rotorA_we, // rotorA must share same we
  input [5:0] code_in_ff,
  input crypt_mode_ff,
  input rotorA_inv_in_valid,
  output reg rotorA_inv_out_valid
);

reg [5:0]  rotorA_addr_inv, rotorA_din_inv;
reg [5:0] rotorA_addr_inv_next, rotorA_din_inv_next;
wire [5:0] rotorA_inv_sram_dout;
reg [LS_COUNT_DELAY*6-1:0] ls_count_buffer, ls_count_buffer_next;
wire [5:0]ls_count_ff_encrypt, ls_count_ff_decrypt, ls_count_sel; 


// module connection
sram64x6 U_rotorA_inverse_buffer(
  .clk(clk),
  .we(rotorA_we),
  .addr(rotorA_addr_inv), 
  .din(rotorA_din_inv),
  .dout(rotorA_inv_sram_dout)
);

// rotorA din
always @ * begin
    rotorA_din_inv_next = rotorA_ls_count;
end

// address
always @ * begin
  if (rotorA_inv_in_valid) rotorA_addr_inv_next = rotorA_inv_din;
  else rotorA_addr_inv_next = code_in_ff;
end

// current left shift count need to be add to result
// for encrypt, ls_count must wait 2 cycles
// for decrypt, ls_count just need to wait 1 cycle

always @ (posedge clk) begin
  ls_count_buffer <= {ls_count_buffer[LS_COUNT_DELAY*6-1-6:0], rotorA_ls_count};
  rotorA_inv_out_valid <= rotorA_inv_in_valid;
end

assign ls_count_ff_encrypt = ls_count_buffer[LS_COUNT_DELAY*6-1-:6];
assign ls_count_ff_decrypt = ls_count_buffer[LS_COUNT_DELAY*6-1-6-:6];
assign ls_count_sel = (crypt_mode_ff) ? ls_count_ff_decrypt : ls_count_ff_encrypt;

assign rotorA_inv_dout = rotorA_inv_sram_dout - ls_count_sel;


// rotorA table's data FF
always @ (posedge clk) begin
	rotorA_din_inv 	<= rotorA_din_inv_next;
end

// table's address & ls_count FF
always @ (posedge clk) begin
	if (~srst_n) begin
		rotorA_addr_inv	<= 6'd0;
	end
	else begin
		rotorA_addr_inv	<= rotorA_addr_inv_next;
	end
end

endmodule

// the module only calculate the bit_sw mode of encrypt
module bit_sw_for_mode(
	input clk,
	input srst_n,
	input [5:0] for_text_in,
	output reg [1:0] for_text_out_mode,

  // control signals
  input bit_sw_for_in_valid, 
  output bit_sw_for_out_valid
);

reg [1:0] for_text_out_mode_next;

assign bit_sw_for_out_valid = bit_sw_for_in_valid;

always @* begin
  for_text_out_mode_next = for_text_out_mode;
	case({bit_sw_for_in_valid, for_text_out_mode})
		3'b100: for_text_out_mode_next = ~for_text_in[1:0];
		3'b101: for_text_out_mode_next = {for_text_in[4], for_text_in[5]};
		3'b110: for_text_out_mode_next = {for_text_in[0], for_text_in[1]};
		3'b111: for_text_out_mode_next = {for_text_in[4], for_text_in[3]};
	endcase
end

always @ (posedge clk) begin
	if (~srst_n) for_text_out_mode <= 0;
	else for_text_out_mode <= for_text_out_mode_next;
end

endmodule


module bit_sw_for_top(
	input [5:0] for_text_in_ls0,
  input [5:0] for_text_in_ls1,
  input [5:0] for_text_in_ls2,
  input [5:0] for_text_in_ls3,
	output [5:0] for_text_out_mode00_ls0,
  output [5:0] for_text_out_mode00_ls1,
  output [5:0] for_text_out_mode00_ls2,
  output [5:0] for_text_out_mode00_ls3,
  output [5:0] for_text_out_mode01_ls0, 
  output [5:0] for_text_out_mode01_ls1, 
  output [5:0] for_text_out_mode01_ls2, 
  output [5:0] for_text_out_mode01_ls3,
  output [5:0] for_text_out_mode10_ls0, 
  output [5:0] for_text_out_mode10_ls1, 
  output [5:0] for_text_out_mode10_ls2, 
  output [5:0] for_text_out_mode10_ls3,
  output [5:0] for_text_out_mode11_ls0, 
  output [5:0] for_text_out_mode11_ls1, 
  output [5:0] for_text_out_mode11_ls2, 
  output [5:0] for_text_out_mode11_ls3
);

// mode 00 : bitwise invert
assign for_text_out_mode00_ls0 = ~for_text_in_ls0;
assign for_text_out_mode00_ls1 = ~for_text_in_ls1;
assign for_text_out_mode00_ls2 = ~for_text_in_ls2;
assign for_text_out_mode00_ls3 = ~for_text_in_ls3;

// mode 01 : {0,1,2,3,4,5}
assign for_text_out_mode01_ls0 = {for_text_in_ls0[0], for_text_in_ls0[1], for_text_in_ls0[2], for_text_in_ls0[3], for_text_in_ls0[4], for_text_in_ls0[5]};
assign for_text_out_mode01_ls1 = {for_text_in_ls1[0], for_text_in_ls1[1], for_text_in_ls1[2], for_text_in_ls1[3], for_text_in_ls1[4], for_text_in_ls1[5]};
assign for_text_out_mode01_ls2 = {for_text_in_ls2[0], for_text_in_ls2[1], for_text_in_ls2[2], for_text_in_ls2[3], for_text_in_ls2[4], for_text_in_ls2[5]};
assign for_text_out_mode01_ls3 = {for_text_in_ls3[0], for_text_in_ls3[1], for_text_in_ls3[2], for_text_in_ls3[3], for_text_in_ls3[4], for_text_in_ls3[5]};

// mode 10 : {4,5,2,3,0,1}
assign for_text_out_mode10_ls0 = {for_text_in_ls0[4], for_text_in_ls0[5], for_text_in_ls0[2], for_text_in_ls0[3], for_text_in_ls0[0], for_text_in_ls0[1]};
assign for_text_out_mode10_ls1 = {for_text_in_ls1[4], for_text_in_ls1[5], for_text_in_ls1[2], for_text_in_ls1[3], for_text_in_ls1[0], for_text_in_ls1[1]};
assign for_text_out_mode10_ls2 = {for_text_in_ls2[4], for_text_in_ls2[5], for_text_in_ls2[2], for_text_in_ls2[3], for_text_in_ls2[0], for_text_in_ls2[1]};
assign for_text_out_mode10_ls3 = {for_text_in_ls3[4], for_text_in_ls3[5], for_text_in_ls3[2], for_text_in_ls3[3], for_text_in_ls3[0], for_text_in_ls3[1]};

// mode 11 : {2,1,0,5,4,3}
assign for_text_out_mode11_ls0 = {for_text_in_ls0[2], for_text_in_ls0[1], for_text_in_ls0[0], for_text_in_ls0[5], for_text_in_ls0[4], for_text_in_ls0[3]};
assign for_text_out_mode11_ls1 = {for_text_in_ls1[2], for_text_in_ls1[1], for_text_in_ls1[0], for_text_in_ls1[5], for_text_in_ls1[4], for_text_in_ls1[3]};
assign for_text_out_mode11_ls2 = {for_text_in_ls2[2], for_text_in_ls2[1], for_text_in_ls2[0], for_text_in_ls2[5], for_text_in_ls2[4], for_text_in_ls2[3]};
assign for_text_out_mode11_ls3 = {for_text_in_ls3[2], for_text_in_ls3[1], for_text_in_ls3[0], for_text_in_ls3[5], for_text_in_ls3[4], for_text_in_ls3[3]};

endmodule

module bit_sw_inv(
	input clk,
	input srst_n,
	input [5:0] inv_text_in,
  input [1:0] for_text_out_mode,
	input crypt_mode_ff,
	input sw_enable,
	output reg [5:0] inv_text_out,
  input bit_sw_inv_in_valid, 
  output bit_sw_inv_out_valid
);

reg [1:0] mode, mode_next;

assign bit_sw_inv_out_valid = bit_sw_inv_in_valid;

// Delay 1~2 cycle for the for_text_out
reg [1:0] for_mode_ff, for_mode_ff2;
always @ (posedge clk)
  {for_mode_ff2, for_mode_ff} <= {for_mode_ff, for_text_out_mode};

always @ * begin
	if (sw_enable) mode_next = (crypt_mode_ff) ? inv_text_in[1:0] : for_text_out_mode ;// 0: encrypt; 1:decrypt;
	else mode_next = mode;
end

always @* begin
	case(mode)
		2'b00: inv_text_out = ~inv_text_in;
		2'b01: inv_text_out = {inv_text_in[0], inv_text_in[1], inv_text_in[2], inv_text_in[3], inv_text_in[4], inv_text_in[5]};
		2'b10: inv_text_out = {inv_text_in[4], inv_text_in[5], inv_text_in[2], inv_text_in[3], inv_text_in[0], inv_text_in[1]};
		2'b11: inv_text_out = {inv_text_in[2], inv_text_in[1], inv_text_in[0], inv_text_in[5], inv_text_in[4], inv_text_in[3]};
	endcase
end

always @ (posedge clk) begin
	if (~srst_n) mode <= 0;
	else mode <= mode_next;
end


endmodule


module plugboard(
	input clk,
	input srst_n,
	input [5:0] plugboard_for_din, // = code_in_ff
	input [5:0] plugboard_inv_din, // from rotorA
	output reg [5:0] plugboard_for_dout,
	output reg [5:0] plugboard_inv_dout,

	input [1:0] table_idx_ff,
	input load_ff,
	input encrypt_ff,

	input plugboard_inv_in_valid,
	output reg plugboard_for_out_valid,
	output reg plugboard_inv_out_valid
);

localparam TEST_NUM = 32;

// Declarations
reg plugboard_we;
reg [5:0] mem [0:TEST_NUM-1];
integer i;

// write enable
always @*
	plugboard_we = (table_idx_ff[0] == 1'b1 && load_ff == 1'b1);

// load data
always @ (posedge clk)begin
  if (plugboard_we) begin
    mem[0] <= plugboard_for_din;
    for(i=1; i<TEST_NUM; i=i+1)
      mem[i] <= mem[i-1];
  end
end

// control signals 
always @ (posedge clk) begin
	plugboard_inv_out_valid 			    <= plugboard_inv_in_valid;
	plugboard_for_out_valid 			    <= encrypt_ff;
end

// pairing table (forward)
always @ (posedge clk) begin
  case (plugboard_for_din) 
    mem[0]  : plugboard_for_dout <= mem[1];
    mem[1]  : plugboard_for_dout <= mem[0];
    mem[2]  : plugboard_for_dout <= mem[3];
    mem[3]  : plugboard_for_dout <= mem[2];
    mem[4]  : plugboard_for_dout <= mem[5];
    mem[5]  : plugboard_for_dout <= mem[4];
    mem[6]  : plugboard_for_dout <= mem[7];
    mem[7]  : plugboard_for_dout <= mem[6];
    mem[8]  : plugboard_for_dout <= mem[9];
    mem[9]  : plugboard_for_dout <= mem[8];
    mem[10] : plugboard_for_dout <= mem[11];
    mem[11] : plugboard_for_dout <= mem[10];
    mem[12] : plugboard_for_dout <= mem[13];
    mem[13] : plugboard_for_dout <= mem[12];
    mem[14] : plugboard_for_dout <= mem[15];
    mem[15] : plugboard_for_dout <= mem[14];
    mem[16] : plugboard_for_dout <= mem[17];
    mem[17] : plugboard_for_dout <= mem[16];
    mem[18] : plugboard_for_dout <= mem[19];
    mem[19] : plugboard_for_dout <= mem[18];
    mem[20] : plugboard_for_dout <= mem[21];
    mem[21] : plugboard_for_dout <= mem[20];
    mem[22] : plugboard_for_dout <= mem[23];
    mem[23] : plugboard_for_dout <= mem[22];
    mem[24] : plugboard_for_dout <= mem[25];
    mem[25] : plugboard_for_dout <= mem[24];
    mem[26] : plugboard_for_dout <= mem[27];
    mem[27] : plugboard_for_dout <= mem[26];
    mem[28] : plugboard_for_dout <= mem[29];
    mem[29] : plugboard_for_dout <= mem[28];
    mem[30] : plugboard_for_dout <= mem[31];
    mem[31] : plugboard_for_dout <= mem[30];

    default : plugboard_for_dout <= plugboard_for_din;
  endcase
end

// pairing table (inverse)
always @ (posedge clk) begin
  case (plugboard_inv_din)
    mem[0]  : plugboard_inv_dout <= mem[1];
    mem[1]  : plugboard_inv_dout <= mem[0];
    mem[2]  : plugboard_inv_dout <= mem[3];
    mem[3]  : plugboard_inv_dout <= mem[2];
    mem[4]  : plugboard_inv_dout <= mem[5];
    mem[5]  : plugboard_inv_dout <= mem[4];
    mem[6]  : plugboard_inv_dout <= mem[7];
    mem[7]  : plugboard_inv_dout <= mem[6];
    mem[8]  : plugboard_inv_dout <= mem[9];
    mem[9]  : plugboard_inv_dout <= mem[8];
    mem[10] : plugboard_inv_dout <= mem[11];
    mem[11] : plugboard_inv_dout <= mem[10];
    mem[12] : plugboard_inv_dout <= mem[13];
    mem[13] : plugboard_inv_dout <= mem[12];
    mem[14] : plugboard_inv_dout <= mem[15];
    mem[15] : plugboard_inv_dout <= mem[14];
    mem[16] : plugboard_inv_dout <= mem[17];
    mem[17] : plugboard_inv_dout <= mem[16];
    mem[18] : plugboard_inv_dout <= mem[19];
    mem[19] : plugboard_inv_dout <= mem[18];
    mem[20] : plugboard_inv_dout <= mem[21];
    mem[21] : plugboard_inv_dout <= mem[20];
    mem[22] : plugboard_inv_dout <= mem[23];
    mem[23] : plugboard_inv_dout <= mem[22];
    mem[24] : plugboard_inv_dout <= mem[25];
    mem[25] : plugboard_inv_dout <= mem[24];
    mem[26] : plugboard_inv_dout <= mem[27];
    mem[27] : plugboard_inv_dout <= mem[26];
    mem[28] : plugboard_inv_dout <= mem[29];
    mem[29] : plugboard_inv_dout <= mem[28];
    mem[30] : plugboard_inv_dout <= mem[31];
    mem[31] : plugboard_inv_dout <= mem[30];
    default : plugboard_inv_dout <= plugboard_inv_din;
  endcase
end


endmodule

module rotorB_for_4r1w (
	input clk,
	input [5:0] rotorB_for_din_ls0,
  input [5:0] rotorB_for_din_ls1,
  input [5:0] rotorB_for_din_ls2, 
  input [5:0] rotorB_for_din_ls3,

  output [5:0] rotorB_for_dout_ls0,
  output [5:0] rotorB_for_dout_ls1,
  output [5:0] rotorB_for_dout_ls2,
  output [5:0] rotorB_for_dout_ls3,

  // input for rot_mode decision
  input [5:0] rotorB_for_dout_if_encrypt,

	input [5:0] rotorB_inv_din,
	output [5:0] rotorB_inv_dout,

	// control signals
	input rotorB_for_in_valid,
	input rotorB_inv_in_valid,
	input [5:0] code_in_ff,
	input [1:0] table_idx_ff,
	input crypt_mode_ff,
	input load_ff,

	// output control signals
	output rotorB_for_out_valid,
	output rotorB_inv_out_valid
);

wire load_sram; // load the 64 data
wire [1:0] rot_mode; // the stage1 rotor mode

// TODO simplfied : table_idx_ff[1] == 1;
// WARN : rot_mode decision ? be sure to consume dout0 if encrypt
// Thinking : if encrypt, then ls_count add chosen = 0
assign load_sram = (table_idx_ff[1] == 1'b1) && load_ff;
assign rot_mode = (crypt_mode_ff) ? rotorB_inv_din[1:0] : rotorB_for_dout_if_encrypt[1:0] ;

sram64x6_rotorB_4r1w U_sram64x6_rotorB_4r1w (
  .clk              (clk),

  // control signals
  .load             (load_sram),
  .rotorB_sram_din  (code_in_ff),
  .rot_mode         (rot_mode),
  .rotorB_in_valid  (rotorB_for_in_valid),

  // 4 read address
  .rotorB_for_addr0 (rotorB_for_din_ls0),
  .rotorB_for_addr1 (rotorB_for_din_ls1),
  .rotorB_for_addr2 (rotorB_for_din_ls2),
  .rotorB_for_addr3 (rotorB_for_din_ls3),

  // inv
  .rotorB_inv_din   (rotorB_inv_din),

  // 4 read data outputs
  .rotorB_for_dout0 (rotorB_for_dout_ls0),
  .rotorB_for_dout1 (rotorB_for_dout_ls1),
  .rotorB_for_dout2 (rotorB_for_dout_ls2),
  .rotorB_for_dout3 (rotorB_for_dout_ls3),

   // inv
  .rotorB_inv_dout  (rotorB_inv_dout)
);

assign rotorB_for_out_valid = rotorB_for_in_valid;
assign rotorB_inv_out_valid = rotorB_inv_in_valid;

endmodule


module sram64x6_rotorB_4r1w (
  input        clk,
  input        load,    // load enable

  input  [5:0] rotorB_sram_din,
  input  [5:0] rotorB_for_addr0,
  input  [5:0] rotorB_for_addr1,
  input  [5:0] rotorB_for_addr2,
  input  [5:0] rotorB_for_addr3,

  input  [5:0] rotorB_inv_din, 

  output [5:0] rotorB_for_dout0,
  output [5:0] rotorB_for_dout1,
  output [5:0] rotorB_for_dout2,
  output [5:0] rotorB_for_dout3,

  output reg [5:0] rotorB_inv_dout, // output address 

  input  [1:0] rot_mode,
  input        rotorB_in_valid
);


reg [5:0] mem [0:63];
//reg [5:0] addr_r;
integer i;

reg [5:0] mem_MUX [0:63];

// memory update
always @* begin
  case (rot_mode)
    2'b00: begin
      for (i = 0; i < 64; i = i + 1)
        mem_MUX[i] = mem[i ^ 6'd0]; 
    end
    2'b01: begin
      for (i = 0; i < 64; i = i + 1)
        mem_MUX[i] = mem[i ^ 6'd1]; 
    end
    2'b10: begin
      for (i = 0; i < 64; i = i + 1)
        mem_MUX[i] = mem[i ^ 6'd2];
    end
    2'b11: begin
      for (i = 0; i < 64; i = i + 1)
        mem_MUX[i] = mem[i ^ 6'd3]; 
    end
    default: begin
      for (i = 0; i < 64; i = i + 1)
        mem_MUX[i] = mem[i];
    end
  endcase
end


always @(posedge clk) begin
	if (load) begin
		mem[63] <= rotorB_sram_din;
		for (i = 0; i < 63; i = i + 1)
			mem[i] <= mem[i+1];
	end
	else if (rotorB_in_valid) begin
		mem[0]  <= mem_MUX[56];
		mem[1]  <= mem_MUX[61];
		mem[2]  <= mem_MUX[25];
		mem[3]  <= mem_MUX[17];
		mem[4]  <= mem_MUX[42];
		mem[5]  <= mem_MUX[48];
		mem[6]  <= mem_MUX[23];
		mem[7]  <= mem_MUX[43];
		mem[8]  <= mem_MUX[10];
		mem[9]  <= mem_MUX[28];
		mem[10] <= mem_MUX[58];
		mem[11] <= mem_MUX[24];
		mem[12] <= mem_MUX[21];
		mem[13] <= mem_MUX[29];
		mem[14] <= mem_MUX[18];
		mem[15] <= mem_MUX[38];
		mem[16] <= mem_MUX[26];
		mem[17] <= mem_MUX[13];
		mem[18] <= mem_MUX[57];
		mem[19] <= mem_MUX[6];
		mem[20] <= mem_MUX[22];
		mem[21] <= mem_MUX[47];
		mem[22] <= mem_MUX[8];
		mem[23] <= mem_MUX[40];
		mem[24] <= mem_MUX[54];
		mem[25] <= mem_MUX[2];
		mem[26] <= mem_MUX[32];
		mem[27] <= mem_MUX[63];
		mem[28] <= mem_MUX[14];
		mem[29] <= mem_MUX[34];
		mem[30] <= mem_MUX[60];
		mem[31] <= mem_MUX[55];
		mem[32] <= mem_MUX[49];
		mem[33] <= mem_MUX[16];
		mem[34] <= mem_MUX[9];
		mem[35] <= mem_MUX[44];
		mem[36] <= mem_MUX[5];
		mem[37] <= mem_MUX[3];
		mem[38] <= mem_MUX[53];
		mem[39] <= mem_MUX[46];
		mem[40] <= mem_MUX[51];
		mem[41] <= mem_MUX[39];
		mem[42] <= mem_MUX[30];
		mem[43] <= mem_MUX[11];
		mem[44] <= mem_MUX[15];
		mem[45] <= mem_MUX[4];
		mem[46] <= mem_MUX[36];
		mem[47] <= mem_MUX[59];
		mem[48] <= mem_MUX[50];
		mem[49] <= mem_MUX[19];
		mem[50] <= mem_MUX[35];
		mem[51] <= mem_MUX[52];
		mem[52] <= mem_MUX[62];
		mem[53] <= mem_MUX[1];
		mem[54] <= mem_MUX[37];
		mem[55] <= mem_MUX[7];
		mem[56] <= mem_MUX[12];
		mem[57] <= mem_MUX[45];
		mem[58] <= mem_MUX[31];
		mem[59] <= mem_MUX[27];
		mem[60] <= mem_MUX[41];
		mem[61] <= mem_MUX[20];
		mem[62] <= mem_MUX[0];
		mem[63] <= mem_MUX[33];
	end
end

// forward read
assign rotorB_for_dout0  = mem[rotorB_for_addr0 ];
assign rotorB_for_dout1  = mem[rotorB_for_addr1 ];
assign rotorB_for_dout2  = mem[rotorB_for_addr2 ];
assign rotorB_for_dout3  = mem[rotorB_for_addr3 ];

// ======================================
// ========= Inverse searching ==========
// ======================================

always @* begin
  rotorB_inv_dout = 6'd0; // default to avoid latch
  case (rotorB_inv_din) // synopsys parallel_case full_case
    mem[0]  : rotorB_inv_dout = 6'd0;
    mem[1]  : rotorB_inv_dout = 6'd1;
    mem[2]  : rotorB_inv_dout = 6'd2;
    mem[3]  : rotorB_inv_dout = 6'd3;
    mem[4]  : rotorB_inv_dout = 6'd4;
    mem[5]  : rotorB_inv_dout = 6'd5;
    mem[6]  : rotorB_inv_dout = 6'd6;
    mem[7]  : rotorB_inv_dout = 6'd7;
    mem[8]  : rotorB_inv_dout = 6'd8;
    mem[9]  : rotorB_inv_dout = 6'd9;
    mem[10] : rotorB_inv_dout = 6'd10;
    mem[11] : rotorB_inv_dout = 6'd11;
    mem[12] : rotorB_inv_dout = 6'd12;
    mem[13] : rotorB_inv_dout = 6'd13;
    mem[14] : rotorB_inv_dout = 6'd14;
    mem[15] : rotorB_inv_dout = 6'd15;
    mem[16] : rotorB_inv_dout = 6'd16;
    mem[17] : rotorB_inv_dout = 6'd17;
    mem[18] : rotorB_inv_dout = 6'd18;
    mem[19] : rotorB_inv_dout = 6'd19;
    mem[20] : rotorB_inv_dout = 6'd20;
    mem[21] : rotorB_inv_dout = 6'd21;
    mem[22] : rotorB_inv_dout = 6'd22;
    mem[23] : rotorB_inv_dout = 6'd23;
    mem[24] : rotorB_inv_dout = 6'd24;
    mem[25] : rotorB_inv_dout = 6'd25;
    mem[26] : rotorB_inv_dout = 6'd26;
    mem[27] : rotorB_inv_dout = 6'd27;
    mem[28] : rotorB_inv_dout = 6'd28;
    mem[29] : rotorB_inv_dout = 6'd29;
    mem[30] : rotorB_inv_dout = 6'd30;
    mem[31] : rotorB_inv_dout = 6'd31;
    mem[32] : rotorB_inv_dout = 6'd32;
    mem[33] : rotorB_inv_dout = 6'd33;
    mem[34] : rotorB_inv_dout = 6'd34;
    mem[35] : rotorB_inv_dout = 6'd35;
    mem[36] : rotorB_inv_dout = 6'd36;
    mem[37] : rotorB_inv_dout = 6'd37;
    mem[38] : rotorB_inv_dout = 6'd38;
    mem[39] : rotorB_inv_dout = 6'd39;
    mem[40] : rotorB_inv_dout = 6'd40;
    mem[41] : rotorB_inv_dout = 6'd41;
    mem[42] : rotorB_inv_dout = 6'd42;
    mem[43] : rotorB_inv_dout = 6'd43;
    mem[44] : rotorB_inv_dout = 6'd44;
    mem[45] : rotorB_inv_dout = 6'd45;
    mem[46] : rotorB_inv_dout = 6'd46;
    mem[47] : rotorB_inv_dout = 6'd47;
    mem[48] : rotorB_inv_dout = 6'd48;
    mem[49] : rotorB_inv_dout = 6'd49;
    mem[50] : rotorB_inv_dout = 6'd50;
    mem[51] : rotorB_inv_dout = 6'd51;
    mem[52] : rotorB_inv_dout = 6'd52;
    mem[53] : rotorB_inv_dout = 6'd53;
    mem[54] : rotorB_inv_dout = 6'd54;
    mem[55] : rotorB_inv_dout = 6'd55;
    mem[56] : rotorB_inv_dout = 6'd56;
    mem[57] : rotorB_inv_dout = 6'd57;
    mem[58] : rotorB_inv_dout = 6'd58;
    mem[59] : rotorB_inv_dout = 6'd59;
    mem[60] : rotorB_inv_dout = 6'd60;
    mem[61] : rotorB_inv_dout = 6'd61;
    mem[62] : rotorB_inv_dout = 6'd62;
    mem[63] : rotorB_inv_dout = 6'd63;
    default : rotorB_inv_dout = 6'd0;
  endcase
end

endmodule


module bit_sw_sel(
  input clk,
  // data inputs
  input  [5:0] din_mode00_ls0, din_mode01_ls0, din_mode10_ls0, din_mode11_ls0, // ls0
  input  [5:0] din_mode00_ls1, din_mode01_ls1, din_mode10_ls1, din_mode11_ls1, // ls1
  input  [5:0] din_mode00_ls2, din_mode01_ls2, din_mode10_ls2, din_mode11_ls2, // ls2
  input  [5:0] din_mode00_ls3, din_mode01_ls3, din_mode10_ls3, din_mode11_ls3, // ls3

  // selected outputs
  output reg [5:0] dout_ls0,
  output reg [5:0] dout_ls1,
  output reg [5:0] dout_ls2,
  output reg [5:0] dout_ls3,

  // control signal
  input        enable,
  input        crypt_mode_ff,    
  input  [1:0] encrypt_mode,      
  input  [1:0] decrypt_mode  
);

reg enable_ff; // since the first cycle the mode must be 2'b00

wire [1:0] mode;
assign mode = enable_ff ? (crypt_mode_ff ? decrypt_mode : encrypt_mode) : 2'b00;

always @* begin
  dout_ls0 = din_mode00_ls0;
  dout_ls1 = din_mode00_ls1;
  dout_ls2 = din_mode00_ls2;
  dout_ls3 = din_mode00_ls3;

  case (mode) // synopsys parallel_case
    2'b00: begin
      dout_ls0 = din_mode00_ls0;
      dout_ls1 = din_mode00_ls1;
      dout_ls2 = din_mode00_ls2;
      dout_ls3 = din_mode00_ls3;
    end
    2'b01: begin
      dout_ls0 = din_mode01_ls0;
      dout_ls1 = din_mode01_ls1;
      dout_ls2 = din_mode01_ls2;
      dout_ls3 = din_mode01_ls3;
    end
    2'b10: begin
      dout_ls0 = din_mode10_ls0;
      dout_ls1 = din_mode10_ls1;
      dout_ls2 = din_mode10_ls2;
      dout_ls3 = din_mode10_ls3;
    end
    2'b11: begin
      dout_ls0 = din_mode11_ls0;
      dout_ls1 = din_mode11_ls1;
      dout_ls2 = din_mode11_ls2;
      dout_ls3 = din_mode11_ls3;
    end
  endcase
end

always @ (posedge clk)
  enable_ff <= enable;

endmodule


module xor_whitening_4merged_sel(
	input clk,
	input srst_n,
  input [5:0] XOR_whitening_in_ls0,
  input [5:0] XOR_whitening_in_ls1,
  input [5:0] XOR_whitening_in_ls2,
  input [5:0] XOR_whitening_in_ls3,
  output reg [5:0] dout,

  // control signals
  input        crypt_mode_ff,       
  input  [5:0] bit_sw_inv_dout,
  input XOR_whitening_en,
  output XOR_whitening_out_valid
);

wire [1:0] sel_signal;
wire [5:0] XOR_whitening_out_ls0, XOR_whitening_out_ls1, XOR_whitening_out_ls2, XOR_whitening_out_ls3;
reg [5:0] state, state_next;

always @* begin
  if (XOR_whitening_en) state_next = {state[4:0], state[4]^state[5]};
  else state_next = state;
end

always @ (posedge clk) begin
  if (~srst_n) state <= 6'b000001;
  else state <= state_next;
end

assign  XOR_whitening_out_ls0 = XOR_whitening_in_ls0 ^ state;
assign  XOR_whitening_out_ls1 = XOR_whitening_in_ls1 ^ state;
assign  XOR_whitening_out_ls2 = XOR_whitening_in_ls2 ^ state;
assign  XOR_whitening_out_ls3 = XOR_whitening_in_ls3 ^ state;
assign  XOR_whitening_out_valid = XOR_whitening_en;

assign sel_signal = bit_sw_inv_dout[1:0];

always @* begin
  casez({crypt_mode_ff, sel_signal}) // synopsys parallel_case
    3'b100: dout = XOR_whitening_out_ls0;
    3'b101: dout = XOR_whitening_out_ls1;
    3'b110: dout = XOR_whitening_out_ls2;
    3'b111: dout = XOR_whitening_out_ls3;
    default : dout = XOR_whitening_out_ls0; // encrypt
  endcase
end

endmodule
