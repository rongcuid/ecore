module ecore #(
	parameter ROM_WORDS = 1024,
	parameter ROM_WORDS_LOG = 10
)(
	input wire i_clk,
	input wire i_rst,

	inout wire [31:0] io_gpio_bank,

	output wire [ROM_WORDS_LOG-1:0] o_rom_addr,
	input wire [31:0] i_rom_data,

	output wire [29:0] o_ram_addr,
	output wire [31:0] o_ram_wdata,
	input wire [31:0] i_ram_rdata
);


localparam CORE_FETCH = 0,
	CORE_DECODE = 1,
	CORE_ALU = 2,
	CORE_JUMP = 3,
	CORE_RS2 = 4;

/* ========== RegFile ========== */

reg [3:0] rf_addr;
reg [31:0] regfile [0:14];
reg [31:0] rf_wdata;
reg [31:0] rf_rdata; 
reg rf_we;
always @ (posedge i_clk) begin : REGFILE
	if (rf_addr != 4'b0) 
		rf_rdata <= regfile[~rf_addr];
	else
		rf_rdata <= 32'b0;
	if (rf_addr != 4'b0 && rf_we) begin
		regfile[~rf_addr] <= rf_wdata;
	end
end

/* ========== Decode ========== */

localparam OP_LOAD = 7'b0000011,
	OP_STORE = 7'b0100011,
	OP_BRANCH = 7'b1100011,
	OP_JALR = 7'b1100111,
	OP_MISCMEM = 7'b0001111,
	OP_JAL = 7'b1101111,
	OP_OPIMM = 7'b0010011,
	OP_OP = 7'b0110011,
	OP_SYSTEM = 7'b1110011,
	OP_AUIPC = 7'b0010111,
	OP_LUI = 7'b0110111;

localparam ALU_ADD = 4'd0,
	ALU_SUB = 4'd1,
	ALU_SLL = 4'd2,
	ALU_SLT = 4'd3,
	ALU_SLTU = 4'd4,
	ALU_XOR = 4'd5,
	ALU_SRL = 4'd6,
	ALU_SRA = 4'd7,
	ALU_OR = 4'd8,
	ALU_AND = 4'd9,
	ALU_EQ = 4'd10,
	ALU_NE = 4'd11,
	ALU_LT = 4'd12,
	ALU_GE = 4'd13,
	ALU_LTU = 4'd14,
	ALU_GEU = 4'd15;

task decode;
	input [31:0] inst;
	output [3:0] alu;
	output op1_pc;
	output [31:0] op2;
	output illegal;
	output [31:0] next_state;
	output memory;
	output regwrite;
	output branch;
	output jump;
	output [4:0] rd, rs1, rs2;
	reg [6:0] opcode;
	reg [2:0] funct3;
	reg [6:0] funct7;
	reg [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
	begin
		// Defaults
		op1_pc = 1'b0;
		illegal = 1'b0;
		memory = 1'b0;
		regwrite = 1'b0;
		branch = 1'b0;
		jump = 1'b0;
		next_state = CORE_FETCH; // NOP
		// Fields
		opcode = inst[6:0];
		rd = inst[11:7];
		funct3 = inst[14:12];
		rs1 = inst[19:15];
		rs2 = inst[24:20];
		funct7 = inst[31:25];
		// Immediates
		imm_i = { {20{inst[31]}}, inst[31:20] };
		imm_s = { {20{inst[31]}}, funct7, rd };
		imm_b = { {20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0 };
		imm_u = { inst[31:12], 12'b0 };
		imm_j = { {12{inst[31]}}, inst[19:12], inst[20], inst[30:25], inst[24:21], 1'b0 };
		case (opcode)
			OP_LOAD: begin
				next_state = CORE_ALU;
				memory = 1'b1;
				regwrite = 1'b1;
				op2 = imm_i;
				alu = ALU_ADD;
			end
			OP_STORE: begin
				next_state = CORE_RS2;
				memory = 1'b1;
				op2 = imm_s;
				alu = ALU_ADD;
			end
			OP_BRANCH: begin
				next_state = CORE_RS2;
				branch = 1'b1;
				case (funct3)
					3'b000: alu = ALU_EQ;
					3'b001: alu = ALU_NE;
					3'b100: alu = ALU_LT;
					3'b101: alu = ALU_GE;
					3'b110: alu = ALU_LTU;
					3'b111: alu = ALU_GEU;
					default: illegal = 1'b1;
				endcase
			end
			OP_JALR: begin
				next_state = CORE_ALU;
				regwrite = 1'b1;
				jump = 1'b1;
				op2 = imm_i;
				alu = ALU_ADD;
			end
			OP_MISCMEM: begin
			end
			OP_JAL: begin
				next_state = CORE_ALU;
				regwrite = 1'b1;
				jump = 1'b1;
				op1_pc = 1'b1; // Add with PC
				op2 = imm_j + 32'h4;
				alu = ALU_ADD;
			end
			OP_OPIMM: begin
				next_state = CORE_ALU;
				regwrite = 1'b1;
				op2 = imm_i;
				case (funct3)
					3'b000: alu = ALU_ADD;
					3'b010: alu = ALU_SLT;
					3'b011: alu = ALU_SLTU;
					3'b100: alu = ALU_XOR;
					3'b110: alu = ALU_OR;
					3'b111: alu = ALU_AND;
					3'b001: alu = ALU_SLL;
					3'b101: begin
						if (funct7 == 7'b0000000)
							alu = ALU_SRL;
						else if (funct7 == 7'b0100000)
							alu = ALU_SRA;
						else
							illegal = 1'b1;
					end
				endcase
			end
			OP_OP: begin
				next_state = CORE_RS2;
				regwrite = 1'b1;
			end
			OP_SYSTEM: begin
			end
			OP_AUIPC: begin
				next_state = CORE_ALU;
				regwrite = 1'b1;
			end
			OP_LUI: begin
				next_state = CORE_ALU;
				regwrite = 1'b1;
			end
			default: begin
				illegal = 1'b1;
			end
		endcase
	end
endtask

/* ========== CPU Core ========== */

integer core_state;

/* PC is always aligned. */
reg [31:2] pc;

reg [ROM_WORDS-1:0] rom_addr;
assign o_rom_addr = rom_addr;

always @ (posedge i_clk) begin : CORE_STATE_MACHINE
	if (i_rst) begin
		core_state <= CORE_FETCH;
		pc <= 32'b0;
	end
	else begin
		case (core_state)
			CORE_FETCH: begin
				// Combinational
				rom_addr = pc[2+:ROM_WORDS_LOG];
				// Sequential
				core_state <= CORE_DECODE;
			end
			CORE_DECODE: begin
			end
		endcase
	end
end

endmodule
