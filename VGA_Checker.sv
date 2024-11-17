`define HS_F_DIV		781
`define VS_F_DIV		831480
`define HS_DUR			156		//total length of each horizontal signal
`define HS_P_DUR		10			//hs pulse duration, 100s of ns
`define HS_B_DUR		23			//back porch duration, 100s of ns
`define HS_D_DUR		119		//display duration, 100s of ns
`define HS_F_DUR		4			//front porch duration, 100s of ns
`define VS_DUR			1066		//total length of the frame in lines
`define VS_P_DUR		3			//vs pulse duration, lines
`define VS_B_DUR		38			//back porch duration, lines
`define VS_D_DUR		1024		//display duration, lines
`define VS_F_DUR		1			//front porch duration, lines
`define N				15


module VGA_Checker(
		input		[3:0]		KEY,
		input					CLOCK_50,
		input		[35:0]	GPIO,
		
		output	[7:0]		VGA_R,
		output	[7:0]		VGA_G,
		output	[7:0]		VGA_B,
		output				VGA_CLK,
		output				VGA_BLANK_N,
		output				VGA_SYNC_N,
		
		output				VGA_HS,
		output				VGA_VS
);
		logic		[$clog2(`HS_F_DIV)-1:0]	HS_counter, next_HS_counter;
		logic		[$clog2(`VS_F_DIV)-1:0]	VS_counter, next_VS_counter;
		
		logic		[7:0]		red,		next_red;
		logic		[7:0]		green,	next_green;
		logic		[7:0]		blue,		next_blue;
		logic					hs,		next_hs;
		logic					vs,		next_vs;
		logic					blank,	next_blank;
		logic					sync,		next_sync;
		
		logic					spi_clk, csel, mosi, miso;
		
		logic	[`N-1:0][7:0]	regs, next_regs;
		logic [15:0]		 	packet, next_packet;
		logic [4:0]				spi_cnt, next_spi_cnt;
		
		logic	[`N-1:0][$clog2(`HS_F_DIV)-1:0]	cmp_regs;
		
		assign spi_clk			= GPIO[0];
		assign csel				= GPIO[2];
		assign mosi				= GPIO[4];
		assign miso				= GPIO[6];
		
		assign VGA_R			= red;
		assign VGA_G			= green;
		assign VGA_B			= blue;
		assign VGA_BLANK_N	= blank;
		assign VGA_SYNC_N		= sync;
		assign VGA_HS			= hs;
		assign VGA_VS			= vs;
		assign VGA_CLK			= CLOCK_50;
		//assign GPIO[1]			= VS_counter[$clog2(`VS_F_DIV)-1];
		generate
			genvar j;
			for(j = 0; j < `N; j++) begin : compCreation
				assign cmp_regs[j] = {regs[j],'0};
			end
		endgenerate
		
		always_comb begin
			next_HS_counter	= HS_counter;
			next_red				= red;
			next_green			= green;
			next_blue			= blue;
			if(HS_counter >= `HS_F_DIV) begin //clock division for VS
				next_HS_counter = '0;
			end else begin
				next_HS_counter = HS_counter + 1;
			end
			
			next_VS_counter = VS_counter;
			if(VS_counter >= `VS_F_DIV) begin //clock division for VS
				next_VS_counter = '0;
			end else begin
				next_VS_counter = VS_counter + 1;
			end
			
			//next_red			= VS_counter[$clog2(`VS_F_DIV)-3] ^ HS_counter[$clog2(`HS_F_DIV)-4] ? '1 : '0;
			//next_green		= VS_counter[$clog2(`VS_F_DIV)-3] ^ HS_counter[$clog2(`HS_F_DIV)-4] ? '1 : '0;
			//next_blue		= VS_counter[$clog2(`VS_F_DIV)-3] ^ HS_counter[$clog2(`HS_F_DIV)-4] ? '1 : '0;
			
			for(int i = 0; i < `N; i++) begin
				if(VS_counter <= (((i+1) * `VS_F_DIV) / (`N)) && VS_counter >= (((i) * `VS_F_DIV) / (`N))) begin
					if((HS_counter-200) <= cmp_regs[i]) begin
						next_red		= '1;
						next_green	= '1;
						next_blue	= '1;
					end else begin
						next_red		= '0;
						next_green	= '0;
						next_blue	= '0;
					end
				end
			end
			
			
			if(HS_counter <= ((`HS_F_DIV *`HS_P_DUR)/`HS_DUR)) begin //creates the HS pulse
				next_hs = 0;
			end else begin
				next_hs = 1;
			end
			
			if(HS_counter <= 200 || HS_counter >= 780) begin
				next_blank = 0;
			end else begin
				next_blank = 1;
			end
			
			
			if(VS_counter <= 2340) begin//((`VS_F_DIV * `VS_P_DUR)/`VS_DUR)) begin //creates the VS pulse
				next_vs = 0;
			end else begin
				next_vs = 1;
			end
			
			if(VS_counter <= 31980 || VS_counter >= 830700) begin
				next_sync = 0;
			end else begin
				next_sync = 1;
			end
		end
		
		always_ff @(posedge CLOCK_50) begin
			if(~KEY[0]) begin
				HS_counter <= '0;
				VS_counter <= '0;
			
				red	<=	'0;
				green <=	'0;
				blue	<=	'0;
				
				blank	<=	'0;
				sync	<=	'0;
				hs		<=	'0;
				vs		<= '0;
				regs	<=	'0;
			end else begin
				HS_counter <= next_HS_counter;
				VS_counter <= next_VS_counter;
				
				red	<=	next_red;
				green <=	next_green;
				blue	<=	next_blue;
				
				blank	<=	next_blank;
				sync	<=	next_sync;
				hs		<=	next_hs;
				vs		<= next_vs;
				regs	<=	next_regs;
			end
		end
		
		

		always_comb begin
			next_packet 	= packet;
			next_regs		= regs;
			
			if(csel) begin
				next_packet = {mosi, packet[15:1]};
			end
			
			if(~csel) begin
				next_regs[packet[7:0]] = packet[15:8];
			end
		end
		
		always_ff @(posedge spi_clk) begin
			 packet	<= next_packet;
		end
endmodule