

module ipdc (                       //Don't modify interface
	input         i_clk,
	input         i_rst_n,
	input         i_op_valid,
	input  [ 2:0] i_op_mode,
	input         i_in_valid,
	input  [23:0] i_in_data,
	output        o_in_ready,
	output        o_out_valid,
	output [23:0] o_out_data
);

// IO
reg o_in_ready_r;
reg o_out_valid_r;
reg [23:0] o_out_data_r;

assign o_in_ready = o_in_ready_r;
assign o_out_valid = o_out_valid_r;
assign o_out_data = o_out_data_r;

// other reg wire
reg [4:0] state, state_n;
reg [7:0] sramAddress_r; 
reg [4:0] coeff;

reg sramWen_r;


reg [7:0] origin;
wire origin_R_limit;

assign origin_R_limit = (origin >= 15 && origin <= 18) || (origin >= 25 && origin <= 28) || (origin >= 35 && origin <= 38) || (origin >= 45 && origin <= 48) ||
								(origin >= 55 && origin <= 58) || (origin >= 65 && origin <= 68) || (origin >= 75 && origin <= 78) || (origin >= 85 && origin <= 88) || 
								(origin >= 95 && origin <= 98);
	
integer cnt;

wire [7:0] sramAddress;
wire sramWen;
wire zeropad;
assign sramAddress = sramAddress_r ;
assign sramWen = sramWen_r;
assign zeropad = (cnt < 10 || (cnt >= 88 && cnt < 99) || cnt == 18 || cnt == 28 || cnt == 38 || cnt == 48 || cnt == 58 || cnt == 68 || cnt == 78 || 
					cnt == 19 || cnt == 29 || cnt == 39 || cnt == 49 || cnt == 59 || cnt == 69 ||cnt == 79 || cnt == 89);  

reg [7:0] R_data_r, G_data_r, B_data_r;
wire [7:0] R_data, G_data, B_data, R_data_out, G_data_out, B_data_out;
assign R_data = R_data_r;
assign G_data = G_data_r;
assign B_data = B_data_r;

// compute YCbCr
reg isRGB_r, isMed_r;
wire [15:0] Y_tmp, Cb_tmp, Cr_tmp;
wire [7:0] Y, Cb, Cr;

assign Y_tmp = 2 * R_data_out + 5 * G_data_out;
assign Cb_tmp =  - R_data_out - 2 * G_data_out + 4 * B_data_out;
assign Cr_tmp =  4 * R_data_out - 3 * G_data_out - B_data_out;


assign Y = Y_tmp[2] == 1'b1 ? Y_tmp[10:3] + 1'b1 : Y_tmp[10:3];
assign Cb = Cb_tmp[2] == 1'b1 ? Cb_tmp[10:3] + 32'd128 + 1'b1 : Cb_tmp[10:3] + 32'd128;
assign Cr = Cr_tmp[2] == 1'b1 ? Cr_tmp[10:3] + 32'd128 + 1'b1 : Cr_tmp[10:3] + 32'd128;

// origin shift
reg [2:0] i_op_mode_temp;
parameter originRight = 3'h1, originDown = 3'h2, originDefault = 3'h3, originZoom = 3'h4;
parameter READ = 5'h0, Display1 = 5'h1, Display2 = 5'h2, Display3 = 5'h4, op_hold = 5'h5, DisplayNOP = 5'h3, isRGB = 5'h6, isYCbCr = 5'h7;
parameter MED = 5'h8, medRead = 5'h9, medCom1 = 5'ha, medComp2 = 5'hb,  medComp3 = 5'hf, NOP = 5'he, medComp3_0 = 5'hc, medComp3_1 = 5'hd;
parameter RD = 5'd16, Wait = 5'd17, WR = 5'd18, Wait2 = 5'd19;


// median filter
reg [23:0] tmp_1 [8:0];
reg [23:0] tmp [8:0];
reg [23:0] tmp_2;

reg [7:0] sramAddress_r_tmp;
integer idx_core, pixel_cnt;

wire origin_R_limit_med; 
assign origin_R_limit_med = (origin >= 132 && origin <= 135) || (origin >= 140 && origin <= 143) || (origin >= 148 && origin <= 151) || (origin >= 156 && origin <= 159) ||
								(origin >= 164 && origin <= 167) || (origin >= 172 && origin <= 175) || (origin >= 180 && origin <= 183) || (origin >= 188 && origin <= 191) || 
								(origin >= 195 && origin <= 198);

wire [23:0] max_r1, min_r1, med_r1;
wire [23:0] max_r2, min_r2, med_r2;
wire [23:0] max_r3, min_r3, med_r3;

wire [23:0] max_c1, min_c1, med_c1;
wire [23:0] max_c2, min_c2, med_c2;
wire [23:0] max_c3, min_c3, med_c3;

wire [23:0] max_d, min_d, med_d;
wire [23:0] max_od, min_od, med_od;

// sort in row
median sort_row1(.a(tmp_1[0]), .b(tmp_1[1]), .c(tmp_1[2]), .max(max_r1), .min(min_r1), .median(med_r1));
median sort_row2(.a(tmp_1[3]), .b(tmp_1[4]), .c(tmp_1[5]), .max(max_r2), .min(min_r2), .median(med_r2));
median sort_row3(.a(tmp_1[6]), .b(tmp_1[7]), .c(tmp_1[8]), .max(max_r3), .min(min_r3), .median(med_r3));
// sort in col
median sort_col1(.a(tmp_1[0]), .b(tmp_1[3]), .c(tmp_1[6]), .max(max_c1), .min(min_c1), .median(med_c1));
median sort_col2(.a(tmp_1[1]), .b(tmp_1[4]), .c(tmp_1[7]), .max(max_c2), .min(min_c2), .median(med_c2));
median sort_col3(.a(tmp_1[2]), .b(tmp_1[5]), .c(tmp_1[8]), .max(max_c3), .min(min_c3), .median(med_c3));
// sort in diagonal and other diagonal
median sort_d(.a(tmp_1[0]), .b(tmp_1[4]), .c(tmp_1[8]), .max(max_d), .min(min_d), .median(med_d));
median sort_od(.a(tmp_1[2]), .b(tmp_1[4]), .c(tmp_1[6]), .max(max_od), .min(min_od), .median(med_od));


// sram for RGB
sram_256x8 mySramR(.A(sramAddress),
					.D(R_data),
					.CLK(i_clk),
					.CEN(1'b0),
					.WEN(sramWen),
					.Q(R_data_out));

sram_256x8 mySramG(.A(sramAddress),
					.D(G_data),
					.CLK(i_clk),
					.CEN(1'b0),
					.WEN(sramWen),
					.Q(G_data_out));

sram_256x8 mySramB(.A(sramAddress),
					.D(B_data),
					.CLK(i_clk),
					.CEN(1'b0),
					.WEN(sramWen),
					.Q(B_data_out));
 

// state transition
always@(*) begin
	case (state)
	 READ: begin
			if (cnt == 32'd99)
				state_n = op_hold;
			else
				state_n = READ;
		end
		Display1:
			state_n = Display2;
		Display2:
			state_n = DisplayNOP;
		DisplayNOP:
			state_n = Display3;
		Display3: begin 
			if (cnt == 32'd16)
				state_n = op_hold;
			else 
				state_n = Display3;
		end
		op_hold: begin
			if (i_op_valid && i_op_mode == 0 )
				state_n = READ;			
			else if (i_op_valid && (i_op_mode == 3'h1 || i_op_mode == 3'h2 || i_op_mode == 3'h3 || i_op_mode == 3'h4))
				state_n = Display1;
			else if (i_op_valid && i_op_mode == 3'h5)
				state_n = MED;
			else if (i_op_valid && i_op_mode == 3'h6)
				state_n = isYCbCr;
			else if (i_op_valid && i_op_mode == 3'h7)
				state_n = isRGB;
			else 
				state_n = op_hold;
		end
		MED: 
			state_n = NOP;
		NOP:
			state_n = medRead;
		medRead: begin
			if (idx_core == 32'd8)
				state_n = medCom1;
			else
				state_n = NOP;
			
		end
		medCom1:
			state_n = medComp2;
		medComp2:
			state_n = medComp3_0;
		medComp3_0:
			state_n = medComp3;
		medComp3_1:
			state_n = medComp3;
		medComp3: begin  
			if (pixel_cnt == 32'd63)
				state_n = RD;
			else
				state_n = MED; 
		end
		RD:
			state_n = Wait;
		Wait:
			state_n = Wait2;
		Wait2:
			state_n = WR;
		WR:begin
			if(pixel_cnt==63)
				state_n = op_hold;
			else
				state_n = RD;
		end
		isRGB:
			state_n = op_hold;
		isYCbCr:
			state_n = op_hold;
		default: begin
			state_n = op_hold;
		end
		
	endcase
end 

always @(posedge i_clk or negedge i_rst_n) begin
	if (~i_rst_n) begin
		state <= op_hold;
		o_out_data_r <= 0;
		o_out_valid_r <= 0;
		o_in_ready_r <= 1'b1;
		sramAddress_r <= 0; // 11
		sramWen_r <= 0;
		cnt <= 0;
		origin <= 7'b000_1011; // set to 11
		R_data_r <= 0;
		G_data_r <= 0;
		B_data_r <= 0;
		idx_core <= 0;
		idx_core <= 0;
		pixel_cnt <= 0;
		isRGB_r <= 1'b1;
		isMed_r <= 0;
		i_op_mode_temp <= 0;
		sramAddress_r_tmp <= 0;
	end
	else begin 
		state <= state_n;
		case (state)
			op_hold : begin
				o_out_valid_r <= 0;
				i_op_mode_temp <= i_op_mode;
			end
		 READ: begin
				o_out_valid_r <= 0;
				cnt <= cnt + 1;
				sramAddress_r <= sramAddress_r + 1;
				o_in_ready_r <= 1'b1;

				if (zeropad) begin 
					o_in_ready_r <= 0;
					R_data_r <= 0;
					G_data_r <= 0;
					B_data_r <= 0;
				end
				else begin  
					R_data_r <= i_in_data[7:0];
					G_data_r <= i_in_data[15:8];
					B_data_r <= i_in_data[23:16];
				end

				if (cnt == 32'd99) begin 
					cnt <= 0;
					o_out_valid_r <= 1'b1;
					// sramAddress_r <= 0;
				end
			end
			Display1: begin
				o_out_valid_r <= 1'b0;
				case (i_op_mode_temp)
					originRight: begin

							if (origin_R_limit)
								origin <= origin;
							else
								origin <= origin + 1'b1;
					end
					originDown: begin

	
							if (origin >= 8'd51)
								origin <= origin;
							else
								origin <= origin + 8'd10;
						
					end
					originDefault: begin

							origin <= 8'd11;
					end
					originZoom: begin
							origin <= 8'd33;
					end
					default:
						origin <= 0;
				endcase
			end
			Display2: begin
				sramAddress_r <= origin;
				sramWen_r <= 1'b1;
				cnt <= 0;
			end
			DisplayNOP: begin
				cnt <= cnt + 1'b1;
				// Address compute: YCbCr and RGB

 
					if (cnt[1] && cnt[0])
						sramAddress_r <= sramAddress_r + 7;
					else 
						sramAddress_r <= sramAddress_r + 1;
			end
			Display3: begin
				o_out_valid_r <= 1'b1;
				cnt <= cnt + 1'b1;

				// display RGB or YCbCr
				if (isRGB_r)
					o_out_data_r <= {B_data_out, G_data_out, R_data_out};
				else
					o_out_data_r <= {Cr[7:0], Cb[7:0], Y[7:0]};



					if (cnt[1] && cnt[0])
						sramAddress_r <= sramAddress_r + 7;
					else 
						sramAddress_r <= sramAddress_r + 1;

				if (cnt == 32'd16) begin 
					cnt <= 0;
					sramWen_r <= 0;
					sramAddress_r <= 8'd100;
				end
			end
			MED: begin
				sramAddress_r <= (~pixel_cnt[2] && ~pixel_cnt[1] && ~pixel_cnt[0] && pixel_cnt != 0) ? sramAddress_r_tmp + 2 : sramAddress_r_tmp;
				sramWen_r <= 1'b1;
			end
			NOP : begin
				
			end
			medRead: begin
				tmp[idx_core] <= {B_data_out, G_data_out, R_data_out};
				tmp_1[idx_core] <= {B_data_out, G_data_out, R_data_out};

				idx_core <= idx_core == 32'd8 ? 0 : idx_core + 1;
				if (idx_core == 32'd2 || idx_core == 32'd5)
					sramAddress_r <= sramAddress_r + 32'd8;
				else if (idx_core == 32'd8) begin 
					sramAddress_r_tmp <= sramAddress_r - 32'd21;
				end
				else 
					sramAddress_r <= sramAddress_r+ 1;		 
			end
			medCom1: begin // sort in row
				tmp_1[0] <= min_r1;
				tmp_1[1] <= med_r1;
				tmp_1[2] <= max_r1;

				tmp_1[3] <= min_r2;
				tmp_1[4] <= med_r2;
				tmp_1[5] <= max_r2;

				tmp_1[6] <= min_r3;
				tmp_1[7] <= med_r3;
				tmp_1[8] <= max_r3;

			end
			medComp2: begin // sort in col
				tmp_1[0] <= min_c1;
				tmp_1[3] <= med_c1;
				tmp_1[6] <= max_c1;

				tmp_1[1] <= min_c2;
				tmp_1[4] <= med_c2;
				tmp_1[7] <= max_c2;

				tmp_1[2] <= min_c3;
				tmp_1[5] <= med_c3;
				tmp_1[8] <= max_c3;

			end
			medComp3_0: begin // sort in diagonal
				tmp_1[0] <= min_d;
				tmp_1[4] <= med_d;
				tmp_1[8] <= max_d;
			end
			medComp3_1: begin// sort in other diagonal
				tmp_1[2] <= min_od;
				tmp_1[4] <= med_od;
				tmp_1[6] <= max_od;
			end
			medComp3: begin 
				sramAddress_r <= pixel_cnt + 32'd128;
				o_out_valid_r <= 1'b0;
				sramWen_r <= 1'b0;
				{B_data_r, G_data_r, R_data_r} <= med_od;
				if(pixel_cnt==63)begin
					pixel_cnt <= 0;
					cnt <= 0;
					coeff <= 0;
					sramAddress_r_tmp <= 0;
				end
				else 
					pixel_cnt <= pixel_cnt + 1;
			end
			RD :begin
				sramAddress_r <= 128 + cnt;
				sramWen_r <= 1;
				cnt <= cnt == 63 ? 0 : cnt + 1;
			end
			Wait:begin
				
			end
			Wait2:
				tmp_2 <= {B_data_out, G_data_out, R_data_out};
			WR:begin
				sramAddress_r <= 11 + pixel_cnt + coeff;
				coeff <= pixel_cnt[2:0]==3'b111 ? coeff + 2 : coeff;
				pixel_cnt <= pixel_cnt == 63 ? 0 : pixel_cnt + 1;
				{B_data_r, G_data_r, R_data_r} <= tmp_2;
				sramWen_r <= 0;
				o_out_valid_r <= pixel_cnt==63 ? 1 : 0;
			end
			isRGB : begin
				isRGB_r <= 1'b1;
				o_out_valid_r <= 1'b1;
			end
			isYCbCr: begin
				isRGB_r <= 1'b0;
				o_out_valid_r <= 1'b1;
			end
			default: 
				;
		endcase
	end
end

endmodule


module median(a, b, c, max, min, median);

input [23:0] a, b, c;
output [23:0] max, min, median;

assign max[7:0] = a[7:0] >= b[7:0] && a[7:0] >= c[7:0] ? a[7:0] : 
			b[7:0] >= a[7:0] && b[7:0] >= c[7:0] ?  b[7:0] :
			c[7:0] >= a[7:0] && c[7:0] >= b[7:0] ? c[7:0] : 0;

assign max[15:8] = a[15:8] >= b[15:8] && a[15:8] >= c[15:8] ? a[15:8] : 
			b[15:8] >= a[15:8] && b[15:8] >= c[15:8] ?  b[15:8] :
			c[15:8] >= a[15:8] && c[15:8] >= b[15:8] ? c[15:8] : 0;

assign max[23:16] = a[23:16] >= b[23:16] && a[23:16] >= c[23:16] ? a[23:16] : 
			b[23:16] >= a[23:16] && b[23:16] >= c[23:16] ?  b[23:16] :
			c[23:16] >= a[23:16] && c[23:16] >= b[23:16] ? c[23:16] : 0;

assign min[7:0] = a[7:0] <= b[7:0] && a[7:0] <= c[7:0] ? a[7:0] :
			b[7:0] <= a[7:0] && b[7:0] <= c[7:0] ? b[7:0] : 
			c[7:0] <= a[7:0] && c[7:0] <= b[7:0] ? c[7:0] : 0;

assign min[15:8] = a[15:8] <= b[15:8] && a[15:8] <= c[15:8] ? a[15:8] :
			b[15:8] <= a[15:8] && b[15:8] <= c[15:8] ? b[15:8] : 
			c[15:8] <= a[15:8] && c[15:8] <= b[15:8] ? c[15:8] : 0;

assign min[23:16] = a[23:16] <= b[23:16] && a[23:16] <= c[23:16] ? a[23:16] :
			b[23:16] <= a[23:16] && b[23:16] <= c[23:16] ? b[23:16] : 
			c[23:16] <= a[23:16] && c[23:16] <= b[23:16] ? c[23:16] : 0;

assign median[7:0] = (a[7:0] >= b[7:0] && a[7:0] <= c[7:0]) || (a[7:0] >= c[7:0] && a[7:0] <= b[7:0]) ? a[7:0] : 
				(b[7:0] >= a[7:0] && b[7:0] <= c[7:0]) || (b[7:0] >= c[7:0] && b[7:0] <= a[7:0]) ? b[7:0] : 
				(c[7:0] >= a[7:0] && c[7:0] <= b[7:0]) || (c[7:0] >= b[7:0] && c[7:0] <= a[7:0]) ? c[7:0] : 0;

assign median[15:8] = (a[15:8] >= b[15:8] && a[15:8] <= c[15:8]) || (a[15:8] >= c[15:8] && a[15:8] <= b[15:8]) ? a[15:8] : 
				(b[15:8] >= a[15:8] && b[15:8] <= c[15:8]) || (b[15:8] >= c[15:8] && b[15:8] <= a[15:8]) ? b[15:8] : 
				(c[15:8] >= a[15:8] && c[15:8] <= b[15:8]) || (c[15:8] >= b[15:8] && c[15:8] <= a[15:8]) ? c[15:8] : 0;

assign median[23:16] = (a[23:16] >= b[23:16] && a[23:16] <= c[23:16]) || (a[23:16] >= c[23:16] && a[23:16] <= b[23:16]) ? a[23:16] : 
				(b[23:16] >= a[23:16] && b[23:16] <= c[23:16]) || (b[23:16] >= c[23:16] && b[23:16] <= a[23:16]) ? b[23:16] : 
				(c[23:16] >= a[23:16] && c[23:16] <= b[23:16]) || (c[23:16] >= b[23:16] && c[23:16] <= a[23:16]) ? c[23:16] : 0;

endmodule