// Generator : SpinalHDL v1.2.2    git head : ec5e7e191d669ded826c34b7aaa74f2563574157
// Date      : 16/11/2018, 14:25:32
// Component : Up5kAreaEvn


`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10
`define AluBitwiseCtrlEnum_defaultEncoding_SRC1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define EnvCtrlEnum_defaultEncoding_type [1:0]
`define EnvCtrlEnum_defaultEncoding_NONE 2'b00
`define EnvCtrlEnum_defaultEncoding_XRET 2'b01
`define EnvCtrlEnum_defaultEncoding_ECALL 2'b10
`define EnvCtrlEnum_defaultEncoding_EBREAK 2'b11

`define fsm_enumDefinition_defaultEncoding_type [2:0]
`define fsm_enumDefinition_defaultEncoding_boot 3'b000
`define fsm_enumDefinition_defaultEncoding_fsm_SETUP 3'b001
`define fsm_enumDefinition_defaultEncoding_fsm_IDLE 3'b010
`define fsm_enumDefinition_defaultEncoding_fsm_CMD 3'b011
`define fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD 3'b100

module StreamFifoLowLatency (
      input   io_push_valid,
      output  io_push_ready,
      input   io_push_payload_error,
      input  [31:0] io_push_payload_inst,
      output reg  io_pop_valid,
      input   io_pop_ready,
      output reg  io_pop_payload_error,
      output reg [31:0] io_pop_payload_inst,
      input   io_flush,
      output [0:0] io_occupancy,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  wire [0:0] _zz_5_;
  reg  _zz_1_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [32:0] _zz_2_;
  wire [32:0] _zz_3_;
  reg [32:0] _zz_4_;
  assign _zz_5_ = _zz_2_[0 : 0];
  always @ (*) begin
    _zz_1_ = 1'b0;
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      _zz_1_ = 1'b1;
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = 1'b1;
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = 1'b1;
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  assign ptrMatch = 1'b1;
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
      io_pop_payload_error = _zz_5_[0];
      io_pop_payload_inst = _zz_2_[32 : 1];
    end else begin
      io_pop_valid = io_push_valid;
      io_pop_payload_error = io_push_payload_error;
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign _zz_2_ = _zz_3_;
  assign io_occupancy = (risingOccupancy && ptrMatch);
  assign _zz_3_ = _zz_4_;
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      risingOccupancy <= 1'b0;
    end else begin
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_1_)begin
      _zz_4_ <= {io_push_payload_inst,io_push_payload_error};
    end
  end

endmodule

module StreamArbiter (
      input   io_inputs_0_0,
      output  io_inputs_0_ready,
      input   io_inputs_0_0_wr,
      input  [19:0] io_inputs_0_0_address,
      input  [31:0] io_inputs_0_0_data,
      input  [3:0] io_inputs_0_0_mask,
      input   io_inputs_1_1,
      output  io_inputs_1_ready,
      input   io_inputs_1_1_wr,
      input  [19:0] io_inputs_1_1_address,
      input  [31:0] io_inputs_1_1_data,
      input  [3:0] io_inputs_1_1_mask,
      output  io_output_valid,
      input   io_output_ready,
      output  io_output_payload_wr,
      output [19:0] io_output_payload_address,
      output [31:0] io_output_payload_data,
      output [3:0] io_output_payload_mask,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  wire [1:0] _zz_3_;
  wire [1:0] _zz_4_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_1_;
  wire  _zz_2_;
  assign _zz_3_ = (_zz_1_ & (~ _zz_4_));
  assign _zz_4_ = (_zz_1_ - (2'b01));
  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_1_ = {io_inputs_1_1,io_inputs_0_0};
  assign maskProposal_0 = io_inputs_0_0;
  assign maskProposal_1 = _zz_3_[1];
  assign io_output_valid = ((io_inputs_0_0 && maskRouted_0) || (io_inputs_1_1 && maskRouted_1));
  assign io_output_payload_wr = (maskRouted_0 ? io_inputs_0_0_wr : io_inputs_1_1_wr);
  assign io_output_payload_address = (maskRouted_0 ? io_inputs_0_0_address : io_inputs_1_1_address);
  assign io_output_payload_data = (maskRouted_0 ? io_inputs_0_0_data : io_inputs_1_1_data);
  assign io_output_payload_mask = (maskRouted_0 ? io_inputs_0_0_mask : io_inputs_1_1_mask);
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_2_ = io_chosenOH[1];
  assign io_chosen = _zz_2_;
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      locked <= 1'b0;
    end else begin
      if(io_output_valid)begin
        locked <= 1'b1;
      end
      if((io_output_valid && io_output_ready))begin
        locked <= 1'b0;
      end
    end
  end

  always @ (posedge io_clk) begin
    if(io_output_valid)begin
      maskLocked_0 <= maskRouted_0;
      maskLocked_1 <= maskRouted_1;
    end
  end

endmodule

module BufferCC (
      input   io_dataIn,
      output  io_dataOut,
      input   io_clk);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge io_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module Spram (
      input   io_bus_cmd_valid,
      output  io_bus_cmd_ready,
      input   io_bus_cmd_payload_wr,
      input  [15:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_payload_data,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  wire [15:0] _zz_1_;
  wire [13:0] _zz_2_;
  wire [3:0] _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire [15:0] _zz_7_;
  wire [13:0] _zz_8_;
  wire [3:0] _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire  _zz_12_;
  wire [15:0] _zz_13_;
  wire [15:0] _zz_14_;
  wire  cmd_valid;
  wire  cmd_payload_wr;
  wire [15:0] cmd_payload_address;
  wire [31:0] cmd_payload_data;
  wire [3:0] cmd_payload_mask;
  reg  rspPending;
  reg  rspTarget;
  wire [31:0] readData;
  SB_SPRAM256KA mems_0 ( 
    .DATAIN(_zz_1_),
    .ADDRESS(_zz_2_),
    .MASKWREN(_zz_3_),
    .WREN(cmd_payload_wr),
    .CHIPSELECT(cmd_valid),
    .CLOCK(io_clk),
    .DATAOUT(_zz_13_),
    .STANDBY(_zz_4_),
    .SLEEP(_zz_5_),
    .POWEROFF(_zz_6_) 
  );
  SB_SPRAM256KA mems_1 ( 
    .DATAIN(_zz_7_),
    .ADDRESS(_zz_8_),
    .MASKWREN(_zz_9_),
    .WREN(cmd_payload_wr),
    .CHIPSELECT(cmd_valid),
    .CLOCK(io_clk),
    .DATAOUT(_zz_14_),
    .STANDBY(_zz_10_),
    .SLEEP(_zz_11_),
    .POWEROFF(_zz_12_) 
  );
  assign io_bus_cmd_ready = 1'b1;
  assign cmd_valid = io_bus_cmd_valid;
  assign cmd_payload_wr = io_bus_cmd_payload_wr;
  assign cmd_payload_address = io_bus_cmd_payload_address;
  assign cmd_payload_data = io_bus_cmd_payload_data;
  assign cmd_payload_mask = io_bus_cmd_payload_mask;
  assign _zz_1_ = cmd_payload_data[15 : 0];
  assign _zz_3_ = {{{cmd_payload_mask[1],cmd_payload_mask[1]},cmd_payload_mask[0]},cmd_payload_mask[0]};
  assign _zz_7_ = cmd_payload_data[31 : 16];
  assign _zz_9_ = {{{cmd_payload_mask[3],cmd_payload_mask[3]},cmd_payload_mask[2]},cmd_payload_mask[2]};
  assign _zz_2_ = (cmd_payload_address >>> 2);
  assign _zz_4_ = 1'b0;
  assign _zz_5_ = 1'b0;
  assign _zz_6_ = 1'b1;
  assign _zz_8_ = (cmd_payload_address >>> 2);
  assign _zz_10_ = 1'b0;
  assign _zz_11_ = 1'b0;
  assign _zz_12_ = 1'b1;
  assign readData = {_zz_14_,_zz_13_};
  assign io_bus_rsp_valid = (rspPending && rspTarget);
  assign io_bus_rsp_payload_data = readData;
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      rspPending <= 1'b0;
    end else begin
      rspPending <= (cmd_valid && (! cmd_payload_wr));
    end
  end

  always @ (posedge io_clk) begin
    rspTarget <= io_bus_cmd_valid;
  end

endmodule

module Peripherals (
      input   io_bus_cmd_valid,
      output  io_bus_cmd_ready,
      input   io_bus_cmd_payload_wr,
      input  [5:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_payload_data,
      output  io_mTimeInterrupt,
      output [2:0] io_leds,
      output  io_serialTx,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  wire [0:0] _zz_6_;
  wire [3:0] _zz_7_;
  wire [0:0] _zz_8_;
  wire [6:0] _zz_9_;
  wire  mapper_readAtCmd_valid;
  reg [31:0] mapper_readAtCmd_payload;
  reg  mapper_readAtRsp_valid;
  reg [31:0] mapper_readAtRsp_payload;
  wire  mapper_askWrite;
  wire  mapper_askRead;
  wire  mapper_doWrite;
  wire  mapper_doRead;
  reg [2:0] _zz_1_;
  reg  serialTx_counter_willIncrement;
  wire  serialTx_counter_willClear;
  reg [3:0] serialTx_counter_valueNext;
  reg [3:0] serialTx_counter_value;
  wire  serialTx_counter_willOverflowIfInc;
  wire  serialTx_counter_willOverflow;
  reg [7:0] serialTx_buffer;
  wire [11:0] serialTx_bitstream;
  wire  serialTx_busy;
  reg  _zz_2_;
  wire  serialTx_timer_willIncrement;
  wire  serialTx_timer_willClear;
  reg [6:0] serialTx_timer_valueNext;
  reg [6:0] serialTx_timer_value;
  wire  serialTx_timer_willOverflowIfInc;
  wire  serialTx_timer_willOverflow;
  reg  _zz_3_;
  reg [19:0] timer_counter;
  reg [19:0] timer_cmp;
  wire  timer_hit;
  reg  timer_interrupt;
  reg  _zz_4_;
  reg  _zz_5_;
  assign _zz_6_ = serialTx_counter_willIncrement;
  assign _zz_7_ = {3'd0, _zz_6_};
  assign _zz_8_ = serialTx_timer_willIncrement;
  assign _zz_9_ = {6'd0, _zz_8_};
  assign io_bus_cmd_ready = 1'b1;
  assign mapper_askWrite = (io_bus_cmd_valid && io_bus_cmd_payload_wr);
  assign mapper_askRead = (io_bus_cmd_valid && (! io_bus_cmd_payload_wr));
  assign mapper_doWrite = (mapper_askWrite && io_bus_cmd_ready);
  assign mapper_doRead = (mapper_askRead && io_bus_cmd_ready);
  assign io_bus_rsp_valid = mapper_readAtRsp_valid;
  assign io_bus_rsp_payload_data = mapper_readAtRsp_payload;
  assign mapper_readAtCmd_valid = mapper_doRead;
  always @ (*) begin
    mapper_readAtCmd_payload = (32'b00000000000000000000000000000000);
    _zz_3_ = 1'b0;
    _zz_4_ = 1'b0;
    _zz_5_ = 1'b0;
    case(io_bus_cmd_payload_address)
      6'b000100 : begin
        mapper_readAtCmd_payload[2 : 0] = _zz_1_;
      end
      6'b000000 : begin
        if(mapper_doWrite)begin
          _zz_3_ = 1'b1;
        end
        mapper_readAtCmd_payload[0 : 0] = serialTx_busy;
      end
      6'b010000 : begin
        if(mapper_doWrite)begin
          _zz_4_ = 1'b1;
        end
        mapper_readAtCmd_payload[19 : 0] = timer_counter;
      end
      6'b011000 : begin
        if(mapper_doWrite)begin
          _zz_5_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign io_leds = _zz_1_;
  always @ (*) begin
    serialTx_counter_willIncrement = 1'b0;
    if(((serialTx_counter_value != (4'b0000)) && serialTx_timer_willOverflowIfInc))begin
      serialTx_counter_willIncrement = 1'b1;
    end
    if(_zz_3_)begin
      serialTx_counter_willIncrement = 1'b1;
    end
  end

  assign serialTx_counter_willClear = 1'b0;
  assign serialTx_counter_willOverflowIfInc = (serialTx_counter_value == (4'b1011));
  assign serialTx_counter_willOverflow = (serialTx_counter_willOverflowIfInc && serialTx_counter_willIncrement);
  always @ (*) begin
    if(serialTx_counter_willOverflow)begin
      serialTx_counter_valueNext = (4'b0000);
    end else begin
      serialTx_counter_valueNext = (serialTx_counter_value + _zz_7_);
    end
    if(serialTx_counter_willClear)begin
      serialTx_counter_valueNext = (4'b0000);
    end
  end

  assign serialTx_bitstream = {serialTx_buffer,(4'b0111)};
  assign serialTx_busy = (serialTx_counter_value != (4'b0000));
  assign io_serialTx = _zz_2_;
  assign serialTx_timer_willClear = 1'b0;
  assign serialTx_timer_willOverflowIfInc = (serialTx_timer_value == (7'b1100111));
  assign serialTx_timer_willOverflow = (serialTx_timer_willOverflowIfInc && serialTx_timer_willIncrement);
  always @ (*) begin
    if(serialTx_timer_willOverflow)begin
      serialTx_timer_valueNext = (7'b0000000);
    end else begin
      serialTx_timer_valueNext = (serialTx_timer_value + _zz_9_);
    end
    if(serialTx_timer_willClear)begin
      serialTx_timer_valueNext = (7'b0000000);
    end
  end

  assign serialTx_timer_willIncrement = 1'b1;
  assign timer_hit = (timer_counter == timer_cmp);
  assign io_mTimeInterrupt = timer_interrupt;
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      mapper_readAtRsp_valid <= 1'b0;
      _zz_1_ <= (3'b000);
      serialTx_counter_value <= (4'b0000);
      _zz_2_ <= 1'b1;
      serialTx_timer_value <= (7'b0000000);
      timer_interrupt <= 1'b0;
    end else begin
      mapper_readAtRsp_valid <= mapper_readAtCmd_valid;
      serialTx_counter_value <= serialTx_counter_valueNext;
      _zz_2_ <= serialTx_bitstream[serialTx_counter_value];
      serialTx_timer_value <= serialTx_timer_valueNext;
      if(timer_hit)begin
        timer_interrupt <= 1'b1;
      end
      if(_zz_4_)begin
        timer_interrupt <= 1'b0;
      end
      case(io_bus_cmd_payload_address)
        6'b000100 : begin
          if(mapper_doWrite)begin
            _zz_1_ <= io_bus_cmd_payload_data[2 : 0];
          end
        end
        6'b000000 : begin
        end
        6'b010000 : begin
        end
        6'b011000 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_clk) begin
    mapper_readAtRsp_payload <= mapper_readAtCmd_payload;
    timer_counter <= (timer_counter + (20'b00000000000000000001));
    if((timer_hit || _zz_5_))begin
      timer_counter <= (20'b00000000000000000000);
    end
    case(io_bus_cmd_payload_address)
      6'b000100 : begin
      end
      6'b000000 : begin
        if(mapper_doWrite)begin
          serialTx_buffer <= io_bus_cmd_payload_data[7 : 0];
        end
      end
      6'b010000 : begin
      end
      6'b011000 : begin
        if(mapper_doWrite)begin
          timer_cmp <= io_bus_cmd_payload_data[19 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module FlashXpi (
      input   io_bus_cmd_valid,
      output reg  io_bus_cmd_ready,
      input   io_bus_cmd_payload_wr,
      input  [18:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_payload_data,
      output reg [0:0] io_flash_ss,
      output reg  io_flash_sclk,
      output reg  io_flash_mosi,
      input   io_flash_miso,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  reg  _zz_3_;
  reg  _zz_4_;
  wire [0:0] _zz_5_;
  wire [4:0] _zz_6_;
  wire [5:0] _zz_7_;
  wire [3:0] _zz_8_;
  wire [23:0] _zz_9_;
  wire [5:0] _zz_10_;
  reg  buffer_fill;
  reg [31:0] buffer_buffer;
  reg  buffer_counter_willIncrement;
  wire  buffer_counter_willClear;
  reg [4:0] buffer_counter_valueNext;
  reg [4:0] buffer_counter_value;
  wire  buffer_counter_willOverflowIfInc;
  wire  buffer_counter_willOverflow;
  reg  buffer_done;
  wire  fsm_wantExit;
  reg [6:0] fsm_counter;
  reg `fsm_enumDefinition_defaultEncoding_type fsm_stateReg;
  reg `fsm_enumDefinition_defaultEncoding_type fsm_stateNext;
  wire [15:0] _zz_1_;
  wire [39:0] _zz_2_;
  assign _zz_5_ = buffer_counter_willIncrement;
  assign _zz_6_ = {4'd0, _zz_5_};
  assign _zz_7_ = (fsm_counter >>> 1);
  assign _zz_8_ = _zz_7_[3:0];
  assign _zz_9_ = {5'd0, io_bus_cmd_payload_address};
  assign _zz_10_ = (fsm_counter >>> 1);
  always @(*) begin
    case(_zz_8_)
      4'b0000 : begin
        _zz_3_ = _zz_1_[15];
      end
      4'b0001 : begin
        _zz_3_ = _zz_1_[14];
      end
      4'b0010 : begin
        _zz_3_ = _zz_1_[13];
      end
      4'b0011 : begin
        _zz_3_ = _zz_1_[12];
      end
      4'b0100 : begin
        _zz_3_ = _zz_1_[11];
      end
      4'b0101 : begin
        _zz_3_ = _zz_1_[10];
      end
      4'b0110 : begin
        _zz_3_ = _zz_1_[9];
      end
      4'b0111 : begin
        _zz_3_ = _zz_1_[8];
      end
      4'b1000 : begin
        _zz_3_ = _zz_1_[7];
      end
      4'b1001 : begin
        _zz_3_ = _zz_1_[6];
      end
      4'b1010 : begin
        _zz_3_ = _zz_1_[5];
      end
      4'b1011 : begin
        _zz_3_ = _zz_1_[4];
      end
      4'b1100 : begin
        _zz_3_ = _zz_1_[3];
      end
      4'b1101 : begin
        _zz_3_ = _zz_1_[2];
      end
      4'b1110 : begin
        _zz_3_ = _zz_1_[1];
      end
      default : begin
        _zz_3_ = _zz_1_[0];
      end
    endcase
  end

  always @(*) begin
    case(_zz_10_)
      6'b000000 : begin
        _zz_4_ = _zz_2_[39];
      end
      6'b000001 : begin
        _zz_4_ = _zz_2_[38];
      end
      6'b000010 : begin
        _zz_4_ = _zz_2_[37];
      end
      6'b000011 : begin
        _zz_4_ = _zz_2_[36];
      end
      6'b000100 : begin
        _zz_4_ = _zz_2_[35];
      end
      6'b000101 : begin
        _zz_4_ = _zz_2_[34];
      end
      6'b000110 : begin
        _zz_4_ = _zz_2_[33];
      end
      6'b000111 : begin
        _zz_4_ = _zz_2_[32];
      end
      6'b001000 : begin
        _zz_4_ = _zz_2_[31];
      end
      6'b001001 : begin
        _zz_4_ = _zz_2_[30];
      end
      6'b001010 : begin
        _zz_4_ = _zz_2_[29];
      end
      6'b001011 : begin
        _zz_4_ = _zz_2_[28];
      end
      6'b001100 : begin
        _zz_4_ = _zz_2_[27];
      end
      6'b001101 : begin
        _zz_4_ = _zz_2_[26];
      end
      6'b001110 : begin
        _zz_4_ = _zz_2_[25];
      end
      6'b001111 : begin
        _zz_4_ = _zz_2_[24];
      end
      6'b010000 : begin
        _zz_4_ = _zz_2_[23];
      end
      6'b010001 : begin
        _zz_4_ = _zz_2_[22];
      end
      6'b010010 : begin
        _zz_4_ = _zz_2_[21];
      end
      6'b010011 : begin
        _zz_4_ = _zz_2_[20];
      end
      6'b010100 : begin
        _zz_4_ = _zz_2_[19];
      end
      6'b010101 : begin
        _zz_4_ = _zz_2_[18];
      end
      6'b010110 : begin
        _zz_4_ = _zz_2_[17];
      end
      6'b010111 : begin
        _zz_4_ = _zz_2_[16];
      end
      6'b011000 : begin
        _zz_4_ = _zz_2_[15];
      end
      6'b011001 : begin
        _zz_4_ = _zz_2_[14];
      end
      6'b011010 : begin
        _zz_4_ = _zz_2_[13];
      end
      6'b011011 : begin
        _zz_4_ = _zz_2_[12];
      end
      6'b011100 : begin
        _zz_4_ = _zz_2_[11];
      end
      6'b011101 : begin
        _zz_4_ = _zz_2_[10];
      end
      6'b011110 : begin
        _zz_4_ = _zz_2_[9];
      end
      6'b011111 : begin
        _zz_4_ = _zz_2_[8];
      end
      6'b100000 : begin
        _zz_4_ = _zz_2_[7];
      end
      6'b100001 : begin
        _zz_4_ = _zz_2_[6];
      end
      6'b100010 : begin
        _zz_4_ = _zz_2_[5];
      end
      6'b100011 : begin
        _zz_4_ = _zz_2_[4];
      end
      6'b100100 : begin
        _zz_4_ = _zz_2_[3];
      end
      6'b100101 : begin
        _zz_4_ = _zz_2_[2];
      end
      6'b100110 : begin
        _zz_4_ = _zz_2_[1];
      end
      default : begin
        _zz_4_ = _zz_2_[0];
      end
    endcase
  end

  always @ (*) begin
    io_bus_cmd_ready = 1'b0;
    io_flash_ss[0] = 1'b1;
    io_flash_sclk = 1'b0;
    io_flash_mosi = 1'b0;
    fsm_stateNext = fsm_stateReg;
    case(fsm_stateReg)
      `fsm_enumDefinition_defaultEncoding_fsm_SETUP : begin
        io_flash_ss[0] = 1'b0;
        io_flash_sclk = fsm_counter[0];
        io_flash_mosi = _zz_3_;
        if((fsm_counter == (7'b0011111)))begin
          fsm_stateNext = `fsm_enumDefinition_defaultEncoding_fsm_IDLE;
        end
      end
      `fsm_enumDefinition_defaultEncoding_fsm_IDLE : begin
        if(io_bus_cmd_valid)begin
          fsm_stateNext = `fsm_enumDefinition_defaultEncoding_fsm_CMD;
        end
      end
      `fsm_enumDefinition_defaultEncoding_fsm_CMD : begin
        io_flash_ss[0] = 1'b0;
        io_flash_sclk = fsm_counter[0];
        io_flash_mosi = _zz_4_;
        if((fsm_counter == (7'b1001111)))begin
          io_bus_cmd_ready = 1'b1;
          fsm_stateNext = `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD;
        end
      end
      `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD : begin
        io_flash_ss[0] = 1'b0;
        io_flash_sclk = fsm_counter[0];
        if((fsm_counter == (7'b0111111)))begin
          fsm_stateNext = `fsm_enumDefinition_defaultEncoding_fsm_IDLE;
        end
      end
      default : begin
        fsm_stateNext = `fsm_enumDefinition_defaultEncoding_fsm_SETUP;
      end
    endcase
  end

  always @ (*) begin
    buffer_counter_willIncrement = 1'b0;
    if(buffer_fill)begin
      buffer_counter_willIncrement = 1'b1;
    end
  end

  assign buffer_counter_willClear = 1'b0;
  assign buffer_counter_willOverflowIfInc = (buffer_counter_value == (5'b11111));
  assign buffer_counter_willOverflow = (buffer_counter_willOverflowIfInc && buffer_counter_willIncrement);
  always @ (*) begin
    buffer_counter_valueNext = (buffer_counter_value + _zz_6_);
    if(buffer_counter_willClear)begin
      buffer_counter_valueNext = (5'b00000);
    end
  end

  assign io_bus_rsp_valid = buffer_done;
  assign io_bus_rsp_payload_data = {buffer_buffer[7 : 0],{buffer_buffer[15 : 8],{buffer_buffer[23 : 16],buffer_buffer[31 : 24]}}};
  assign fsm_wantExit = 1'b0;
  assign _zz_1_ = (16'b1000000110000011);
  assign _zz_2_ = {{(8'b00001011),_zz_9_},(8'b00000000)};
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      buffer_fill <= 1'b0;
      buffer_counter_value <= (5'b00000);
      buffer_done <= 1'b0;
      fsm_stateReg <= `fsm_enumDefinition_defaultEncoding_boot;
    end else begin
      buffer_fill <= 1'b0;
      buffer_counter_value <= buffer_counter_valueNext;
      buffer_done <= buffer_counter_willOverflow;
      fsm_stateReg <= fsm_stateNext;
      case(fsm_stateReg)
        `fsm_enumDefinition_defaultEncoding_fsm_SETUP : begin
        end
        `fsm_enumDefinition_defaultEncoding_fsm_IDLE : begin
        end
        `fsm_enumDefinition_defaultEncoding_fsm_CMD : begin
        end
        `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD : begin
          if((fsm_counter[0 : 0] == (1'b1)))begin
            buffer_fill <= 1'b1;
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_clk) begin
    if(buffer_fill)begin
      buffer_buffer <= {buffer_buffer[30 : 0],io_flash_miso};
    end
    fsm_counter <= (fsm_counter + (7'b0000001));
    if(((! (fsm_stateReg == `fsm_enumDefinition_defaultEncoding_fsm_SETUP)) && (fsm_stateNext == `fsm_enumDefinition_defaultEncoding_fsm_SETUP)))begin
      fsm_counter <= (7'b0000000);
    end
    if(((! (fsm_stateReg == `fsm_enumDefinition_defaultEncoding_fsm_CMD)) && (fsm_stateNext == `fsm_enumDefinition_defaultEncoding_fsm_CMD)))begin
      fsm_counter <= (7'b0000000);
    end
    if(((! (fsm_stateReg == `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD)) && (fsm_stateNext == `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD)))begin
      fsm_counter <= (7'b0000000);
    end
  end

endmodule

module VexRiscv (
      output  iBus_cmd_valid,
      input   iBus_cmd_ready,
      output [31:0] iBus_cmd_payload_pc,
      input   iBus_rsp_valid,
      input   iBus_rsp_payload_error,
      input  [31:0] iBus_rsp_payload_inst,
      input   timerInterrupt,
      input   externalInterrupt,
      output  dBus_cmd_valid,
      input   dBus_cmd_ready,
      output  dBus_cmd_payload_wr,
      output [31:0] dBus_cmd_payload_address,
      output [31:0] dBus_cmd_payload_data,
      output [1:0] dBus_cmd_payload_size,
      input   dBus_rsp_ready,
      input   dBus_rsp_error,
      input  [31:0] dBus_rsp_data,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  wire  _zz_134_;
  wire  _zz_135_;
  reg [31:0] _zz_136_;
  reg [31:0] _zz_137_;
  reg [3:0] _zz_138_;
  reg [31:0] _zz_139_;
  wire  _zz_140_;
  wire  _zz_141_;
  wire  _zz_142_;
  wire [31:0] _zz_143_;
  wire [0:0] _zz_144_;
  wire  _zz_145_;
  wire  _zz_146_;
  wire  _zz_147_;
  wire  _zz_148_;
  wire  _zz_149_;
  wire  _zz_150_;
  wire [1:0] _zz_151_;
  wire [1:0] _zz_152_;
  wire  _zz_153_;
  wire [1:0] _zz_154_;
  wire [1:0] _zz_155_;
  wire [2:0] _zz_156_;
  wire [31:0] _zz_157_;
  wire [1:0] _zz_158_;
  wire [0:0] _zz_159_;
  wire [1:0] _zz_160_;
  wire [0:0] _zz_161_;
  wire [1:0] _zz_162_;
  wire [0:0] _zz_163_;
  wire [1:0] _zz_164_;
  wire [0:0] _zz_165_;
  wire [1:0] _zz_166_;
  wire [0:0] _zz_167_;
  wire [0:0] _zz_168_;
  wire [0:0] _zz_169_;
  wire [0:0] _zz_170_;
  wire [0:0] _zz_171_;
  wire [0:0] _zz_172_;
  wire [0:0] _zz_173_;
  wire [0:0] _zz_174_;
  wire [0:0] _zz_175_;
  wire [2:0] _zz_176_;
  wire [4:0] _zz_177_;
  wire [11:0] _zz_178_;
  wire [11:0] _zz_179_;
  wire [31:0] _zz_180_;
  wire [31:0] _zz_181_;
  wire [31:0] _zz_182_;
  wire [31:0] _zz_183_;
  wire [1:0] _zz_184_;
  wire [31:0] _zz_185_;
  wire [1:0] _zz_186_;
  wire [1:0] _zz_187_;
  wire [31:0] _zz_188_;
  wire [32:0] _zz_189_;
  wire [19:0] _zz_190_;
  wire [11:0] _zz_191_;
  wire [11:0] _zz_192_;
  wire [2:0] _zz_193_;
  wire [2:0] _zz_194_;
  wire [3:0] _zz_195_;
  wire [0:0] _zz_196_;
  wire [0:0] _zz_197_;
  wire [0:0] _zz_198_;
  wire [0:0] _zz_199_;
  wire [0:0] _zz_200_;
  wire [0:0] _zz_201_;
  wire  _zz_202_;
  wire  _zz_203_;
  wire [31:0] _zz_204_;
  wire [31:0] _zz_205_;
  wire [31:0] _zz_206_;
  wire [31:0] _zz_207_;
  wire  _zz_208_;
  wire  _zz_209_;
  wire [0:0] _zz_210_;
  wire [1:0] _zz_211_;
  wire [2:0] _zz_212_;
  wire [2:0] _zz_213_;
  wire  _zz_214_;
  wire [0:0] _zz_215_;
  wire [18:0] _zz_216_;
  wire [31:0] _zz_217_;
  wire [31:0] _zz_218_;
  wire [31:0] _zz_219_;
  wire [31:0] _zz_220_;
  wire  _zz_221_;
  wire  _zz_222_;
  wire  _zz_223_;
  wire [0:0] _zz_224_;
  wire [0:0] _zz_225_;
  wire [0:0] _zz_226_;
  wire [0:0] _zz_227_;
  wire [1:0] _zz_228_;
  wire [1:0] _zz_229_;
  wire  _zz_230_;
  wire [0:0] _zz_231_;
  wire [16:0] _zz_232_;
  wire [31:0] _zz_233_;
  wire [31:0] _zz_234_;
  wire [31:0] _zz_235_;
  wire [31:0] _zz_236_;
  wire [31:0] _zz_237_;
  wire [31:0] _zz_238_;
  wire [31:0] _zz_239_;
  wire [31:0] _zz_240_;
  wire [31:0] _zz_241_;
  wire  _zz_242_;
  wire [0:0] _zz_243_;
  wire [4:0] _zz_244_;
  wire [0:0] _zz_245_;
  wire [0:0] _zz_246_;
  wire  _zz_247_;
  wire [0:0] _zz_248_;
  wire [14:0] _zz_249_;
  wire [31:0] _zz_250_;
  wire [31:0] _zz_251_;
  wire [31:0] _zz_252_;
  wire  _zz_253_;
  wire [0:0] _zz_254_;
  wire [1:0] _zz_255_;
  wire [31:0] _zz_256_;
  wire  _zz_257_;
  wire  _zz_258_;
  wire [0:0] _zz_259_;
  wire [0:0] _zz_260_;
  wire [2:0] _zz_261_;
  wire [2:0] _zz_262_;
  wire  _zz_263_;
  wire [0:0] _zz_264_;
  wire [11:0] _zz_265_;
  wire [31:0] _zz_266_;
  wire [31:0] _zz_267_;
  wire [31:0] _zz_268_;
  wire  _zz_269_;
  wire [31:0] _zz_270_;
  wire [31:0] _zz_271_;
  wire [31:0] _zz_272_;
  wire [31:0] _zz_273_;
  wire [0:0] _zz_274_;
  wire [0:0] _zz_275_;
  wire [0:0] _zz_276_;
  wire [0:0] _zz_277_;
  wire  _zz_278_;
  wire [0:0] _zz_279_;
  wire [9:0] _zz_280_;
  wire [31:0] _zz_281_;
  wire [31:0] _zz_282_;
  wire [31:0] _zz_283_;
  wire [31:0] _zz_284_;
  wire [31:0] _zz_285_;
  wire [2:0] _zz_286_;
  wire [2:0] _zz_287_;
  wire  _zz_288_;
  wire [0:0] _zz_289_;
  wire [6:0] _zz_290_;
  wire [31:0] _zz_291_;
  wire [31:0] _zz_292_;
  wire [31:0] _zz_293_;
  wire [31:0] _zz_294_;
  wire [31:0] _zz_295_;
  wire [31:0] _zz_296_;
  wire [31:0] _zz_297_;
  wire [31:0] _zz_298_;
  wire [0:0] _zz_299_;
  wire [0:0] _zz_300_;
  wire [0:0] _zz_301_;
  wire [0:0] _zz_302_;
  wire  _zz_303_;
  wire [0:0] _zz_304_;
  wire [2:0] _zz_305_;
  wire [31:0] _zz_306_;
  wire [31:0] _zz_307_;
  wire [31:0] _zz_308_;
  wire [31:0] _zz_309_;
  wire [31:0] _zz_310_;
  wire [0:0] _zz_311_;
  wire [0:0] _zz_312_;
  wire [1:0] _zz_313_;
  wire [1:0] _zz_314_;
  wire  _zz_315_;
  wire  _zz_316_;
  wire [31:0] _zz_317_;
  wire [31:0] _zz_318_;
  wire [31:0] _zz_319_;
  wire [31:0] _zz_320_;
  wire [31:0] _zz_321_;
  wire  _zz_322_;
  wire  _zz_323_;
  wire `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_1_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_2_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_3_;
  wire `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_4_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_5_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_6_;
  wire `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_7_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_8_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_9_;
  wire  decode_RS2_USE;
  wire `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_10_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_11_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_12_;
  wire  decode_IS_CSR;
  wire  decode_SRC_LESS_UNSIGNED;
  wire [31:0] execute_FORMAL_PC_NEXT;
  wire [31:0] decode_FORMAL_PC_NEXT;
  wire  decode_RS1_USE;
  wire  execute_REGFILE_WRITE_VALID;
  wire `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_13_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_14_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_15_;
  wire  decode_CSR_WRITE_OPCODE;
  wire  decode_CSR_READ_OPCODE;
  wire `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_16_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_17_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_18_;
  wire  decode_SRC_USE_SUB_LESS;
  wire  decode_MEMORY_ENABLE;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_19_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_20_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_21_;
  wire  execute_CSR_READ_OPCODE;
  wire  execute_CSR_WRITE_OPCODE;
  wire  execute_IS_CSR;
  wire  _zz_22_;
  wire  _zz_23_;
  wire `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_24_;
  wire  execute_RS2_USE;
  wire  execute_RS1_USE;
  wire  execute_IS_FENCEI;
  reg [31:0] _zz_25_;
  wire  decode_IS_FENCEI;
  wire [31:0] execute_BRANCH_CALC;
  wire  execute_BRANCH_DO;
  wire [31:0] _zz_26_;
  wire [31:0] execute_PC;
  wire [31:0] execute_RS1;
  wire `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_27_;
  wire  _zz_28_;
  wire `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_29_;
  wire  _zz_30_;
  wire [31:0] _zz_31_;
  wire [31:0] _zz_32_;
  wire  execute_SRC_LESS_UNSIGNED;
  wire  execute_SRC_USE_SUB_LESS;
  wire `Src2CtrlEnum_defaultEncoding_type execute_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_33_;
  wire [31:0] _zz_34_;
  wire `Src1CtrlEnum_defaultEncoding_type execute_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_35_;
  wire [31:0] _zz_36_;
  wire [31:0] execute_SRC_ADD_SUB;
  wire  execute_SRC_LESS;
  wire `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_37_;
  wire [31:0] _zz_38_;
  wire [31:0] execute_SRC2;
  wire [31:0] execute_SRC1;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_39_;
  wire [31:0] _zz_40_;
  wire  _zz_41_;
  reg  _zz_42_;
  wire [31:0] _zz_43_;
  wire [31:0] _zz_44_;
  reg  decode_REGFILE_WRITE_VALID;
  wire  _zz_45_;
  wire  _zz_46_;
  wire  _zz_47_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_48_;
  wire  _zz_49_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_50_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_51_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_52_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_53_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_54_;
  wire  _zz_55_;
  wire  _zz_56_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_57_;
  wire  _zz_58_;
  wire  _zz_59_;
  reg [31:0] _zz_60_;
  wire [1:0] execute_MEMORY_ADDRESS_LOW;
  wire [31:0] execute_MEMORY_READ_DATA;
  wire [31:0] execute_REGFILE_WRITE_DATA;
  wire [31:0] _zz_61_;
  wire [1:0] _zz_62_;
  wire [31:0] execute_RS2;
  wire [31:0] execute_SRC_ADD;
  wire [31:0] execute_INSTRUCTION;
  wire  execute_ALIGNEMENT_FAULT;
  wire  execute_MEMORY_ENABLE;
  wire  _zz_63_;
  wire [31:0] _zz_64_;
  wire [31:0] _zz_65_;
  wire [31:0] _zz_66_;
  wire [31:0] decode_PC /* verilator public */ ;
  wire [31:0] decode_INSTRUCTION /* verilator public */ ;
  wire  decode_arbitration_haltItself /* verilator public */ ;
  reg  decode_arbitration_haltByOther;
  reg  decode_arbitration_removeIt;
  reg  decode_arbitration_flushAll /* verilator public */ ;
  wire  decode_arbitration_redoIt;
  wire  decode_arbitration_isValid /* verilator public */ ;
  wire  decode_arbitration_isStuck;
  wire  decode_arbitration_isStuckByOthers;
  wire  decode_arbitration_isFlushed;
  wire  decode_arbitration_isMoving;
  wire  decode_arbitration_isFiring;
  reg  execute_arbitration_haltItself;
  reg  execute_arbitration_haltByOther;
  reg  execute_arbitration_removeIt;
  wire  execute_arbitration_flushAll;
  wire  execute_arbitration_redoIt;
  reg  execute_arbitration_isValid;
  wire  execute_arbitration_isStuck;
  wire  execute_arbitration_isStuckByOthers;
  wire  execute_arbitration_isFlushed;
  wire  execute_arbitration_isMoving;
  wire  execute_arbitration_isFiring;
  reg  _zz_67_;
  wire  _zz_68_;
  reg  _zz_69_;
  wire  _zz_70_;
  wire [31:0] _zz_71_;
  wire  _zz_72_;
  reg  _zz_73_;
  reg [31:0] _zz_74_;
  wire  contextSwitching;
  reg [1:0] CsrPlugin_privilege;
  reg  _zz_75_;
  reg [3:0] _zz_76_;
  wire  IBusSimplePlugin_jump_pcLoad_valid;
  wire [31:0] IBusSimplePlugin_jump_pcLoad_payload;
  wire [1:0] _zz_77_;
  wire  IBusSimplePlugin_fetchPc_preOutput_valid;
  wire  IBusSimplePlugin_fetchPc_preOutput_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_preOutput_payload;
  wire  _zz_78_;
  wire  IBusSimplePlugin_fetchPc_output_valid;
  wire  IBusSimplePlugin_fetchPc_output_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_output_payload;
  reg [31:0] IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg  IBusSimplePlugin_fetchPc_inc;
  reg  IBusSimplePlugin_fetchPc_propagatePc;
  reg [31:0] IBusSimplePlugin_fetchPc_pc;
  reg  IBusSimplePlugin_fetchPc_samplePcNext;
  reg  _zz_79_;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  reg  IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_0_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_1_inputSample;
  wire  _zz_80_;
  wire  _zz_81_;
  wire  _zz_82_;
  wire  _zz_83_;
  reg  _zz_84_;
  wire  IBusSimplePlugin_iBusRsp_readyForError;
  wire  IBusSimplePlugin_iBusRsp_decodeInput_valid;
  wire  IBusSimplePlugin_iBusRsp_decodeInput_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_decodeInput_payload_pc;
  wire  IBusSimplePlugin_iBusRsp_decodeInput_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_iBusRsp_decodeInput_payload_rsp_rawInDecode;
  wire  IBusSimplePlugin_iBusRsp_decodeInput_payload_isRvc;
  reg  IBusSimplePlugin_injector_nextPcCalc_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_1;
  reg  IBusSimplePlugin_injector_decodeRemoved;
  wire  IBusSimplePlugin_cmd_valid;
  wire  IBusSimplePlugin_cmd_ready;
  wire [31:0] IBusSimplePlugin_cmd_payload_pc;
  reg [1:0] IBusSimplePlugin_pendingCmd;
  wire [1:0] IBusSimplePlugin_pendingCmdNext;
  reg [1:0] IBusSimplePlugin_rspJoin_discardCounter;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_valid;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_ready;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_pc;
  reg  IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  wire  IBusSimplePlugin_rspJoin_issueDetected;
  wire  IBusSimplePlugin_rspJoin_join_valid;
  wire  IBusSimplePlugin_rspJoin_join_ready;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_pc;
  wire  IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_join_payload_isRvc;
  wire  _zz_85_;
  reg  execute_DBusSimplePlugin_cmdSent;
  reg [31:0] _zz_86_;
  reg [3:0] _zz_87_;
  wire [3:0] execute_DBusSimplePlugin_formalMask;
  reg [31:0] execute_DBusSimplePlugin_rspShifted;
  wire  _zz_88_;
  reg [31:0] _zz_89_;
  wire  _zz_90_;
  reg [31:0] _zz_91_;
  reg [31:0] execute_DBusSimplePlugin_rspFormated;
  wire [24:0] _zz_92_;
  wire  _zz_93_;
  wire  _zz_94_;
  wire  _zz_95_;
  wire  _zz_96_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_97_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_98_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_99_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_100_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_101_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_102_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_103_;
  wire [31:0] execute_RegFilePlugin_srcInstruction;
  wire [4:0] execute_RegFilePlugin_regFileReadAddress1;
  wire [4:0] execute_RegFilePlugin_regFileReadAddress2;
  wire [31:0] execute_RegFilePlugin_rs1Data;
  wire [31:0] execute_RegFilePlugin_rs2Data;
  wire  execute_RegFilePlugin_regFileWrite_valid /* verilator public */ ;
  wire [4:0] execute_RegFilePlugin_regFileWrite_payload_address /* verilator public */ ;
  wire [31:0] execute_RegFilePlugin_regFileWrite_payload_data /* verilator public */ ;
  reg [31:0] execute_IntAluPlugin_bitwise;
  reg [31:0] _zz_104_;
  reg [31:0] _zz_105_;
  wire  _zz_106_;
  reg [19:0] _zz_107_;
  wire  _zz_108_;
  reg [19:0] _zz_109_;
  reg [31:0] _zz_110_;
  wire [31:0] execute_SrcPlugin_addSub;
  wire  execute_SrcPlugin_less;
  reg  execute_LightShifterPlugin_isActive;
  wire  execute_LightShifterPlugin_isShift;
  reg [4:0] execute_LightShifterPlugin_amplitudeReg;
  wire [4:0] execute_LightShifterPlugin_amplitude;
  reg [31:0] execute_LightShifterPlugin_shiftReg;
  wire [31:0] execute_LightShifterPlugin_shiftInput;
  wire  execute_LightShifterPlugin_done;
  reg [31:0] _zz_111_;
  wire  execute_BranchPlugin_eq;
  wire [2:0] _zz_112_;
  reg  _zz_113_;
  reg  _zz_114_;
  wire [31:0] execute_BranchPlugin_branch_src1;
  wire  _zz_115_;
  reg [10:0] _zz_116_;
  wire  _zz_117_;
  reg [19:0] _zz_118_;
  wire  _zz_119_;
  reg [18:0] _zz_120_;
  reg [31:0] _zz_121_;
  wire [31:0] execute_BranchPlugin_branch_src2;
  wire [31:0] execute_BranchPlugin_branchAdder;
  reg  _zz_122_;
  reg  _zz_123_;
  reg  _zz_124_;
  reg [4:0] _zz_125_;
  wire [1:0] CsrPlugin_misa_base;
  wire [25:0] CsrPlugin_misa_extensions;
  reg [1:0] CsrPlugin_mtvec_mode;
  reg [29:0] CsrPlugin_mtvec_base;
  reg [31:0] CsrPlugin_mepc;
  reg  CsrPlugin_mstatus_MIE;
  reg  CsrPlugin_mstatus_MPIE;
  reg [1:0] CsrPlugin_mstatus_MPP;
  reg  CsrPlugin_mip_MEIP;
  reg  CsrPlugin_mip_MTIP;
  reg  CsrPlugin_mip_MSIP;
  reg  CsrPlugin_mie_MEIE;
  reg  CsrPlugin_mie_MTIE;
  reg  CsrPlugin_mie_MSIE;
  reg [31:0] CsrPlugin_mscratch;
  reg  CsrPlugin_mcause_interrupt;
  reg [3:0] CsrPlugin_mcause_exceptionCode;
  reg [31:0] CsrPlugin_mtval;
  reg [63:0] CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg [63:0] CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  wire [31:0] CsrPlugin_medeleg;
  wire [31:0] CsrPlugin_mideleg;
  wire  _zz_126_;
  wire  _zz_127_;
  wire  _zz_128_;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  reg [3:0] CsrPlugin_exceptionPortCtrl_exceptionContext_code;
  reg [31:0] CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
  wire [1:0] CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
  reg  CsrPlugin_exceptionPortCtrl_emulationRequiredReg;
  reg  CsrPlugin_exceptionPortCtrl_emulationRequired;
  wire  execute_exception_agregat_valid;
  wire [3:0] execute_exception_agregat_payload_code;
  wire [31:0] execute_exception_agregat_payload_badAddr;
  wire [2:0] _zz_129_;
  wire [2:0] _zz_130_;
  wire  _zz_131_;
  wire  _zz_132_;
  wire [1:0] _zz_133_;
  reg  CsrPlugin_interrupt;
  reg [3:0] CsrPlugin_interruptCode /* verilator public */ ;
  reg [1:0] CsrPlugin_interruptTargetPrivilege;
  wire  CsrPlugin_exception;
  wire  CsrPlugin_lastStageWasWfi;
  reg  CsrPlugin_pipelineLiberator_done;
  wire  CsrPlugin_interruptJump /* verilator public */ ;
  reg  CsrPlugin_hadException;
  reg [1:0] CsrPlugin_targetPrivilege;
  reg [3:0] CsrPlugin_trapCause;
  reg [31:0] CsrPlugin_emulationEpc;
  wire  execute_CsrPlugin_blockedBySideEffects;
  reg  execute_CsrPlugin_illegalAccess;
  reg  execute_CsrPlugin_illegalInstruction;
  reg [31:0] execute_CsrPlugin_readData;
  wire  execute_CsrPlugin_writeInstruction;
  wire  execute_CsrPlugin_readInstruction;
  wire  execute_CsrPlugin_writeEnable;
  wire  execute_CsrPlugin_readEnable;
  reg [31:0] execute_CsrPlugin_writeData;
  wire [11:0] execute_CsrPlugin_csrAddress;
  reg  decode_to_execute_IS_FENCEI;
  reg `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg  decode_to_execute_MEMORY_ENABLE;
  reg [31:0] decode_to_execute_PC;
  reg  decode_to_execute_SRC_USE_SUB_LESS;
  reg `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg  decode_to_execute_CSR_READ_OPCODE;
  reg  decode_to_execute_CSR_WRITE_OPCODE;
  reg `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg  decode_to_execute_REGFILE_WRITE_VALID;
  reg [31:0] decode_to_execute_INSTRUCTION;
  reg  decode_to_execute_RS1_USE;
  reg [31:0] decode_to_execute_FORMAL_PC_NEXT;
  reg  decode_to_execute_SRC_LESS_UNSIGNED;
  reg  decode_to_execute_IS_CSR;
  reg `Src2CtrlEnum_defaultEncoding_type decode_to_execute_SRC2_CTRL;
  reg  decode_to_execute_RS2_USE;
  reg `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg `Src1CtrlEnum_defaultEncoding_type decode_to_execute_SRC1_CTRL;
  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;
  assign _zz_145_ = ((execute_arbitration_isValid && execute_LightShifterPlugin_isShift) && (execute_SRC2[4 : 0] != (5'b00000)));
  assign _zz_146_ = (! execute_arbitration_isStuckByOthers);
  assign _zz_147_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_148_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_149_ = (IBusSimplePlugin_fetchPc_preOutput_valid && IBusSimplePlugin_fetchPc_preOutput_ready);
  assign _zz_150_ = 1'b1;
  assign _zz_151_ = execute_INSTRUCTION[13 : 12];
  assign _zz_152_ = execute_INSTRUCTION[29 : 28];
  assign _zz_153_ = execute_INSTRUCTION[13];
  assign _zz_154_ = (_zz_77_ & (~ _zz_155_));
  assign _zz_155_ = (_zz_77_ - (2'b01));
  assign _zz_156_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_157_ = {29'd0, _zz_156_};
  assign _zz_158_ = (IBusSimplePlugin_pendingCmd + _zz_160_);
  assign _zz_159_ = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign _zz_160_ = {1'd0, _zz_159_};
  assign _zz_161_ = iBus_rsp_valid;
  assign _zz_162_ = {1'd0, _zz_161_};
  assign _zz_163_ = (iBus_rsp_valid && (IBusSimplePlugin_rspJoin_discardCounter != (2'b00)));
  assign _zz_164_ = {1'd0, _zz_163_};
  assign _zz_165_ = iBus_rsp_valid;
  assign _zz_166_ = {1'd0, _zz_165_};
  assign _zz_167_ = _zz_92_[0 : 0];
  assign _zz_168_ = _zz_92_[1 : 1];
  assign _zz_169_ = _zz_92_[4 : 4];
  assign _zz_170_ = _zz_92_[5 : 5];
  assign _zz_171_ = _zz_92_[18 : 18];
  assign _zz_172_ = _zz_92_[22 : 22];
  assign _zz_173_ = _zz_92_[23 : 23];
  assign _zz_174_ = _zz_92_[24 : 24];
  assign _zz_175_ = execute_SRC_LESS;
  assign _zz_176_ = (3'b100);
  assign _zz_177_ = execute_INSTRUCTION[19 : 15];
  assign _zz_178_ = execute_INSTRUCTION[31 : 20];
  assign _zz_179_ = {execute_INSTRUCTION[31 : 25],execute_INSTRUCTION[11 : 7]};
  assign _zz_180_ = ($signed(_zz_181_) + $signed(_zz_185_));
  assign _zz_181_ = ($signed(_zz_182_) + $signed(_zz_183_));
  assign _zz_182_ = execute_SRC1;
  assign _zz_183_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_184_ = (execute_SRC_USE_SUB_LESS ? _zz_186_ : _zz_187_);
  assign _zz_185_ = {{30{_zz_184_[1]}}, _zz_184_};
  assign _zz_186_ = (2'b01);
  assign _zz_187_ = (2'b00);
  assign _zz_188_ = (_zz_189_ >>> 1);
  assign _zz_189_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_LightShifterPlugin_shiftInput[31]),execute_LightShifterPlugin_shiftInput};
  assign _zz_190_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_191_ = execute_INSTRUCTION[31 : 20];
  assign _zz_192_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_193_ = (_zz_129_ - (3'b001));
  assign _zz_194_ = (execute_INSTRUCTION[5] ? (3'b110) : (3'b100));
  assign _zz_195_ = {1'd0, _zz_194_};
  assign _zz_196_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_197_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_198_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_199_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_200_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_201_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_202_ = 1'b1;
  assign _zz_203_ = 1'b1;
  assign _zz_204_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010000));
  assign _zz_205_ = (32'b00000000000000000010000000000000);
  assign _zz_206_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000000));
  assign _zz_207_ = (32'b00000000000000000001000000000000);
  assign _zz_208_ = ((decode_INSTRUCTION & _zz_217_) == (32'b00000000000000000001000001010000));
  assign _zz_209_ = ((decode_INSTRUCTION & _zz_218_) == (32'b00000000000000000010000001010000));
  assign _zz_210_ = (_zz_219_ == _zz_220_);
  assign _zz_211_ = {_zz_221_,_zz_222_};
  assign _zz_212_ = {_zz_223_,{_zz_224_,_zz_225_}};
  assign _zz_213_ = (3'b000);
  assign _zz_214_ = ({_zz_226_,_zz_227_} != (2'b00));
  assign _zz_215_ = (_zz_228_ != _zz_229_);
  assign _zz_216_ = {_zz_230_,{_zz_231_,_zz_232_}};
  assign _zz_217_ = (32'b00000000000000000001000001010000);
  assign _zz_218_ = (32'b00000000000000000010000001010000);
  assign _zz_219_ = (decode_INSTRUCTION & (32'b00000000000000000000000001000100));
  assign _zz_220_ = (32'b00000000000000000000000001000000);
  assign _zz_221_ = ((decode_INSTRUCTION & _zz_233_) == (32'b01000000000000000000000000110000));
  assign _zz_222_ = ((decode_INSTRUCTION & _zz_234_) == (32'b00000000000000000010000000010000));
  assign _zz_223_ = ((decode_INSTRUCTION & _zz_235_) == (32'b00000000000000000000000001000000));
  assign _zz_224_ = (_zz_236_ == _zz_237_);
  assign _zz_225_ = (_zz_238_ == _zz_239_);
  assign _zz_226_ = _zz_96_;
  assign _zz_227_ = (_zz_240_ == _zz_241_);
  assign _zz_228_ = {_zz_96_,_zz_242_};
  assign _zz_229_ = (2'b00);
  assign _zz_230_ = ({_zz_243_,_zz_244_} != (6'b000000));
  assign _zz_231_ = (_zz_245_ != _zz_246_);
  assign _zz_232_ = {_zz_247_,{_zz_248_,_zz_249_}};
  assign _zz_233_ = (32'b01000000000000000000000000110000);
  assign _zz_234_ = (32'b00000000000000000010000000010100);
  assign _zz_235_ = (32'b00000000000000000000000001010000);
  assign _zz_236_ = (decode_INSTRUCTION & (32'b00000000000000000000000000110000));
  assign _zz_237_ = (32'b00000000000000000000000000000000);
  assign _zz_238_ = (decode_INSTRUCTION & (32'b00000000010000000011000001000000));
  assign _zz_239_ = (32'b00000000000000000000000001000000);
  assign _zz_240_ = (decode_INSTRUCTION & (32'b00000000000000000000000001110000));
  assign _zz_241_ = (32'b00000000000000000000000000100000);
  assign _zz_242_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000100000)) == (32'b00000000000000000000000000000000));
  assign _zz_243_ = ((decode_INSTRUCTION & _zz_250_) == (32'b00000000000000000000000001001000));
  assign _zz_244_ = {(_zz_251_ == _zz_252_),{_zz_253_,{_zz_254_,_zz_255_}}};
  assign _zz_245_ = ((decode_INSTRUCTION & _zz_256_) == (32'b00000000000000000101000000010000));
  assign _zz_246_ = (1'b0);
  assign _zz_247_ = ({_zz_257_,_zz_258_} != (2'b00));
  assign _zz_248_ = ({_zz_259_,_zz_260_} != (2'b00));
  assign _zz_249_ = {(_zz_261_ != _zz_262_),{_zz_263_,{_zz_264_,_zz_265_}}};
  assign _zz_250_ = (32'b00000000000000000000000001001000);
  assign _zz_251_ = (decode_INSTRUCTION & (32'b00000000000000000001000000010000));
  assign _zz_252_ = (32'b00000000000000000001000000010000);
  assign _zz_253_ = ((decode_INSTRUCTION & _zz_266_) == (32'b00000000000000000010000000010000));
  assign _zz_254_ = (_zz_267_ == _zz_268_);
  assign _zz_255_ = {_zz_95_,_zz_269_};
  assign _zz_256_ = (32'b00000000000000000111000001010100);
  assign _zz_257_ = ((decode_INSTRUCTION & _zz_270_) == (32'b01000000000000000001000000010000));
  assign _zz_258_ = ((decode_INSTRUCTION & _zz_271_) == (32'b00000000000000000001000000010000));
  assign _zz_259_ = (_zz_272_ == _zz_273_);
  assign _zz_260_ = _zz_96_;
  assign _zz_261_ = {_zz_96_,{_zz_274_,_zz_275_}};
  assign _zz_262_ = (3'b000);
  assign _zz_263_ = (_zz_94_ != (1'b0));
  assign _zz_264_ = (_zz_276_ != _zz_277_);
  assign _zz_265_ = {_zz_278_,{_zz_279_,_zz_280_}};
  assign _zz_266_ = (32'b00000000000000000010000000010000);
  assign _zz_267_ = (decode_INSTRUCTION & (32'b00000000000000000001000000000100));
  assign _zz_268_ = (32'b00000000000000000000000000000100);
  assign _zz_269_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000101000)) == (32'b00000000000000000000000000000000));
  assign _zz_270_ = (32'b01000000000000000011000001010100);
  assign _zz_271_ = (32'b00000000000000000111000001010100);
  assign _zz_272_ = (decode_INSTRUCTION & (32'b00000000000000000001000000000000));
  assign _zz_273_ = (32'b00000000000000000001000000000000);
  assign _zz_274_ = ((decode_INSTRUCTION & _zz_281_) == (32'b00000000000000000001000000000000));
  assign _zz_275_ = ((decode_INSTRUCTION & _zz_282_) == (32'b00000000000000000010000000000000));
  assign _zz_276_ = ((decode_INSTRUCTION & _zz_283_) == (32'b00000000000000000000000001000000));
  assign _zz_277_ = (1'b0);
  assign _zz_278_ = ((_zz_284_ == _zz_285_) != (1'b0));
  assign _zz_279_ = (_zz_95_ != (1'b0));
  assign _zz_280_ = {(_zz_286_ != _zz_287_),{_zz_288_,{_zz_289_,_zz_290_}}};
  assign _zz_281_ = (32'b00000000000000000011000000000000);
  assign _zz_282_ = (32'b00000000000000000011000000000000);
  assign _zz_283_ = (32'b00000000000000000000000001011000);
  assign _zz_284_ = (decode_INSTRUCTION & (32'b00000000000000000000000000010000));
  assign _zz_285_ = (32'b00000000000000000000000000010000);
  assign _zz_286_ = {((decode_INSTRUCTION & _zz_291_) == (32'b00000000000000000000000000100100)),{(_zz_292_ == _zz_293_),(_zz_294_ == _zz_295_)}};
  assign _zz_287_ = (3'b000);
  assign _zz_288_ = (((decode_INSTRUCTION & _zz_296_) == (32'b00000000000000000010000000010000)) != (1'b0));
  assign _zz_289_ = ((_zz_297_ == _zz_298_) != (1'b0));
  assign _zz_290_ = {({_zz_299_,_zz_300_} != (2'b00)),{(_zz_301_ != _zz_302_),{_zz_303_,{_zz_304_,_zz_305_}}}};
  assign _zz_291_ = (32'b00000000000000000000000001100100);
  assign _zz_292_ = (decode_INSTRUCTION & (32'b00000000000000000100000000010100));
  assign _zz_293_ = (32'b00000000000000000100000000010000);
  assign _zz_294_ = (decode_INSTRUCTION & (32'b00000000000000000011000000010100));
  assign _zz_295_ = (32'b00000000000000000001000000010000);
  assign _zz_296_ = (32'b00000000000000000110000000010100);
  assign _zz_297_ = (decode_INSTRUCTION & (32'b00010000000000000011000001010000));
  assign _zz_298_ = (32'b00000000000000000000000001010000);
  assign _zz_299_ = ((decode_INSTRUCTION & _zz_306_) == (32'b00000000000100000000000001010000));
  assign _zz_300_ = ((decode_INSTRUCTION & _zz_307_) == (32'b00010000000000000000000001010000));
  assign _zz_301_ = ((decode_INSTRUCTION & _zz_308_) == (32'b00000000000000000000000000001000));
  assign _zz_302_ = (1'b0);
  assign _zz_303_ = ((_zz_309_ == _zz_310_) != (1'b0));
  assign _zz_304_ = ({_zz_311_,_zz_312_} != (2'b00));
  assign _zz_305_ = {(_zz_313_ != _zz_314_),{_zz_315_,_zz_316_}};
  assign _zz_306_ = (32'b00010000000100000011000001010000);
  assign _zz_307_ = (32'b00010000010000000011000001010000);
  assign _zz_308_ = (32'b00000000000000000000000001001000);
  assign _zz_309_ = (decode_INSTRUCTION & (32'b00000000000000000000000001011000));
  assign _zz_310_ = (32'b00000000000000000000000000000000);
  assign _zz_311_ = _zz_94_;
  assign _zz_312_ = _zz_93_;
  assign _zz_313_ = {((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000100)),_zz_93_};
  assign _zz_314_ = (2'b00);
  assign _zz_315_ = ({((decode_INSTRUCTION & _zz_317_) == (32'b00000000000000000000000000100000)),((decode_INSTRUCTION & _zz_318_) == (32'b00000000000000000000000000100000))} != (2'b00));
  assign _zz_316_ = ({((decode_INSTRUCTION & _zz_319_) == (32'b00000000000000000000000000000000)),{(_zz_320_ == _zz_321_),{_zz_322_,_zz_323_}}} != (4'b0000));
  assign _zz_317_ = (32'b00000000000000000000000000110100);
  assign _zz_318_ = (32'b00000000000000000000000001100100);
  assign _zz_319_ = (32'b00000000000000000000000001000100);
  assign _zz_320_ = (decode_INSTRUCTION & (32'b00000000000000000000000000011000));
  assign _zz_321_ = (32'b00000000000000000000000000000000);
  assign _zz_322_ = ((decode_INSTRUCTION & (32'b00000000000000000110000000000100)) == (32'b00000000000000000010000000000000));
  assign _zz_323_ = ((decode_INSTRUCTION & (32'b00000000000000000101000000000100)) == (32'b00000000000000000001000000000000));
  initial begin
    $readmemb("Up5kAreaEvn.v_toplevel_soc_system_cpu_RegFilePlugin_regFile.bin",RegFilePlugin_regFile);
  end
  always @ (posedge io_clk) begin
    if(_zz_42_) begin
      RegFilePlugin_regFile[execute_RegFilePlugin_regFileWrite_payload_address] <= execute_RegFilePlugin_regFileWrite_payload_data;
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_202_) begin
      _zz_136_ <= RegFilePlugin_regFile[execute_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_203_) begin
      _zz_137_ <= RegFilePlugin_regFile[execute_RegFilePlugin_regFileReadAddress2];
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid(_zz_134_),
    .io_push_ready(_zz_140_),
    .io_push_payload_error(iBus_rsp_payload_error),
    .io_push_payload_inst(iBus_rsp_payload_inst),
    .io_pop_valid(_zz_141_),
    .io_pop_ready(IBusSimplePlugin_rspJoin_rspBufferOutput_ready),
    .io_pop_payload_error(_zz_142_),
    .io_pop_payload_inst(_zz_143_),
    .io_flush(_zz_135_),
    .io_occupancy(_zz_144_),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(GLOBAL_BUFFER_OUTPUT) 
  );
  always @(*) begin
    case(_zz_133_)
      2'b00 : begin
        _zz_138_ = _zz_195_;
        _zz_139_ = execute_REGFILE_WRITE_DATA;
      end
      2'b01 : begin
        _zz_138_ = (4'b0000);
        _zz_139_ = _zz_71_;
      end
      default : begin
        _zz_138_ = _zz_76_;
        _zz_139_ = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
      end
    endcase
  end

  assign decode_SRC1_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign decode_SHIFT_CTRL = _zz_4_;
  assign _zz_5_ = _zz_6_;
  assign decode_ALU_CTRL = _zz_7_;
  assign _zz_8_ = _zz_9_;
  assign decode_RS2_USE = _zz_58_;
  assign decode_SRC2_CTRL = _zz_10_;
  assign _zz_11_ = _zz_12_;
  assign decode_IS_CSR = _zz_46_;
  assign decode_SRC_LESS_UNSIGNED = _zz_45_;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_64_;
  assign decode_RS1_USE = _zz_59_;
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign decode_BRANCH_CTRL = _zz_13_;
  assign _zz_14_ = _zz_15_;
  assign decode_CSR_WRITE_OPCODE = _zz_23_;
  assign decode_CSR_READ_OPCODE = _zz_22_;
  assign decode_ENV_CTRL = _zz_16_;
  assign _zz_17_ = _zz_18_;
  assign decode_SRC_USE_SUB_LESS = _zz_47_;
  assign decode_MEMORY_ENABLE = _zz_56_;
  assign decode_ALU_BITWISE_CTRL = _zz_19_;
  assign _zz_20_ = _zz_21_;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign execute_ENV_CTRL = _zz_24_;
  assign execute_RS2_USE = decode_to_execute_RS2_USE;
  assign execute_RS1_USE = decode_to_execute_RS1_USE;
  assign execute_IS_FENCEI = decode_to_execute_IS_FENCEI;
  always @ (*) begin
    _zz_25_ = decode_INSTRUCTION;
    if(decode_IS_FENCEI)begin
      _zz_25_[12] = 1'b0;
      _zz_25_[22] = 1'b1;
    end
  end

  assign decode_IS_FENCEI = _zz_55_;
  assign execute_BRANCH_CALC = _zz_26_;
  assign execute_BRANCH_DO = _zz_28_;
  assign execute_PC = decode_to_execute_PC;
  assign execute_RS1 = _zz_44_;
  assign execute_BRANCH_CTRL = _zz_27_;
  assign execute_SHIFT_CTRL = _zz_29_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign execute_SRC2_CTRL = _zz_33_;
  assign execute_SRC1_CTRL = _zz_35_;
  assign execute_SRC_ADD_SUB = _zz_32_;
  assign execute_SRC_LESS = _zz_30_;
  assign execute_ALU_CTRL = _zz_37_;
  assign execute_SRC2 = _zz_34_;
  assign execute_SRC1 = _zz_36_;
  assign execute_ALU_BITWISE_CTRL = _zz_39_;
  assign _zz_40_ = execute_INSTRUCTION;
  assign _zz_41_ = execute_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_42_ = 1'b0;
    if(execute_RegFilePlugin_regFileWrite_valid)begin
      _zz_42_ = 1'b1;
    end
  end

  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_49_;
    if((decode_INSTRUCTION[11 : 7] == (5'b00000)))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  always @ (*) begin
    _zz_60_ = execute_REGFILE_WRITE_DATA;
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_ALIGNEMENT_FAULT)) && (! execute_DBusSimplePlugin_cmdSent)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_INSTRUCTION[5])) && (! dBus_rsp_ready)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if((execute_arbitration_isValid && execute_MEMORY_ENABLE))begin
      _zz_60_ = execute_DBusSimplePlugin_rspFormated;
    end
    if(_zz_145_)begin
      _zz_60_ = _zz_111_;
      if(_zz_146_)begin
        if(! execute_LightShifterPlugin_done) begin
          execute_arbitration_haltItself = 1'b1;
        end
      end
    end
    if((execute_arbitration_isValid && execute_IS_CSR))begin
      _zz_60_ = execute_CsrPlugin_readData;
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
  end

  assign execute_MEMORY_ADDRESS_LOW = _zz_62_;
  assign execute_MEMORY_READ_DATA = _zz_61_;
  assign execute_REGFILE_WRITE_DATA = _zz_38_;
  assign execute_RS2 = _zz_43_;
  assign execute_SRC_ADD = _zz_31_;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_ALIGNEMENT_FAULT = _zz_63_;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  assign decode_PC = _zz_66_;
  assign decode_INSTRUCTION = _zz_65_;
  assign decode_arbitration_haltItself = 1'b0;
  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((CsrPlugin_interrupt && decode_arbitration_isValid))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(1'b0)begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_flushAll = 1'b0;
    execute_arbitration_removeIt = 1'b0;
    _zz_73_ = 1'b0;
    _zz_74_ = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    if(_zz_70_)begin
      decode_arbitration_flushAll = 1'b1;
    end
    CsrPlugin_exceptionPortCtrl_emulationRequired = CsrPlugin_exceptionPortCtrl_emulationRequiredReg;
    CsrPlugin_exceptionPortCtrl_exceptionValids_execute = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
    if(execute_exception_agregat_valid)begin
      decode_arbitration_flushAll = 1'b1;
      execute_arbitration_removeIt = 1'b1;
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b1;
      CsrPlugin_exceptionPortCtrl_emulationRequired = 1'b0;
    end
    if(_zz_147_)begin
      _zz_73_ = 1'b1;
      _zz_74_ = {CsrPlugin_mtvec_base,(2'b00)};
      decode_arbitration_flushAll = 1'b1;
    end
    if(_zz_148_)begin
      _zz_74_ = CsrPlugin_mepc;
      _zz_73_ = 1'b1;
      decode_arbitration_flushAll = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_redoIt = 1'b0;
  always @ (*) begin
    execute_arbitration_haltByOther = 1'b0;
    if(((execute_arbitration_isValid && execute_IS_FENCEI) && 1'b0))begin
      execute_arbitration_haltByOther = 1'b1;
    end
    if((execute_arbitration_isValid && (_zz_122_ || _zz_123_)))begin
      execute_arbitration_haltByOther = 1'b1;
    end
  end

  assign execute_arbitration_flushAll = 1'b0;
  assign execute_arbitration_redoIt = 1'b0;
  always @ (*) begin
    _zz_67_ = 1'b0;
    if((CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode || CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute))begin
      _zz_67_ = 1'b1;
    end
  end

  assign _zz_68_ = 1'b0;
  assign IBusSimplePlugin_jump_pcLoad_valid = (_zz_70_ || _zz_73_);
  assign _zz_77_ = {_zz_73_,_zz_70_};
  assign IBusSimplePlugin_jump_pcLoad_payload = (_zz_154_[0] ? _zz_71_ : _zz_74_);
  assign _zz_78_ = (! _zz_67_);
  assign IBusSimplePlugin_fetchPc_output_valid = (IBusSimplePlugin_fetchPc_preOutput_valid && _zz_78_);
  assign IBusSimplePlugin_fetchPc_preOutput_ready = (IBusSimplePlugin_fetchPc_output_ready && _zz_78_);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_preOutput_payload;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_propagatePc = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_iBusRsp_stages_1_input_ready))begin
      IBusSimplePlugin_fetchPc_propagatePc = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_157_);
    IBusSimplePlugin_fetchPc_samplePcNext = 1'b0;
    if(IBusSimplePlugin_fetchPc_propagatePc)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    if(_zz_149_)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
    IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
  end

  assign IBusSimplePlugin_fetchPc_preOutput_valid = _zz_79_;
  assign IBusSimplePlugin_fetchPc_preOutput_payload = IBusSimplePlugin_fetchPc_pc;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_inputSample = 1'b1;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_0_input_valid && ((! IBusSimplePlugin_cmd_valid) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b1;
    end
  end

  assign _zz_80_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_80_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_80_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
  assign _zz_81_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_81_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_81_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = _zz_82_;
  assign _zz_82_ = ((1'b0 && (! _zz_83_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_83_ = _zz_84_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_83_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = IBusSimplePlugin_fetchPc_pcReg;
  assign IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
  assign IBusSimplePlugin_iBusRsp_decodeInput_ready = (! decode_arbitration_isStuck);
  assign decode_arbitration_isValid = (IBusSimplePlugin_iBusRsp_decodeInput_valid && (! IBusSimplePlugin_injector_decodeRemoved));
  assign _zz_66_ = IBusSimplePlugin_iBusRsp_decodeInput_payload_pc;
  assign _zz_65_ = IBusSimplePlugin_iBusRsp_decodeInput_payload_rsp_rawInDecode;
  assign _zz_64_ = (decode_PC + (32'b00000000000000000000000000000100));
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pendingCmdNext = (_zz_158_ - _zz_162_);
  assign IBusSimplePlugin_cmd_valid = ((IBusSimplePlugin_iBusRsp_stages_0_input_valid && IBusSimplePlugin_iBusRsp_stages_0_output_ready) && (IBusSimplePlugin_pendingCmd != (2'b11)));
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_iBusRsp_stages_0_input_payload[31 : 2],(2'b00)};
  assign _zz_134_ = (iBus_rsp_valid && (! (IBusSimplePlugin_rspJoin_discardCounter != (2'b00))));
  assign _zz_135_ = (IBusSimplePlugin_jump_pcLoad_valid || _zz_68_);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_valid = _zz_141_;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error = _zz_142_;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst = _zz_143_;
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBufferOutput_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  assign IBusSimplePlugin_rspJoin_issueDetected = 1'b0;
  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_1_output_valid && IBusSimplePlugin_rspJoin_rspBufferOutput_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_85_ = (! IBusSimplePlugin_rspJoin_issueDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_decodeInput_ready && _zz_85_);
  assign IBusSimplePlugin_iBusRsp_decodeInput_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_85_);
  assign IBusSimplePlugin_iBusRsp_decodeInput_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_decodeInput_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_decodeInput_payload_rsp_rawInDecode = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_decodeInput_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  assign _zz_63_ = (((dBus_cmd_payload_size == (2'b10)) && (dBus_cmd_payload_address[1 : 0] != (2'b00))) || ((dBus_cmd_payload_size == (2'b01)) && (dBus_cmd_payload_address[0 : 0] != (1'b0))));
  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_ALIGNEMENT_FAULT)) && (! execute_DBusSimplePlugin_cmdSent));
  assign dBus_cmd_payload_wr = execute_INSTRUCTION[5];
  assign dBus_cmd_payload_address = execute_SRC_ADD;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_86_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_86_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_86_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_86_;
  assign _zz_62_ = dBus_cmd_payload_address[1 : 0];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_87_ = (4'b0001);
      end
      2'b01 : begin
        _zz_87_ = (4'b0011);
      end
      default : begin
        _zz_87_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_87_ <<< dBus_cmd_payload_address[1 : 0]);
  assign _zz_61_ = dBus_rsp_data;
  always @ (*) begin
    _zz_69_ = execute_ALIGNEMENT_FAULT;
    if((! ((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers))))begin
      _zz_69_ = 1'b0;
    end
  end

  always @ (*) begin
    execute_DBusSimplePlugin_rspShifted = execute_MEMORY_READ_DATA;
    case(execute_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        execute_DBusSimplePlugin_rspShifted[7 : 0] = execute_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        execute_DBusSimplePlugin_rspShifted[15 : 0] = execute_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        execute_DBusSimplePlugin_rspShifted[7 : 0] = execute_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_88_ = (execute_DBusSimplePlugin_rspShifted[7] && (! execute_INSTRUCTION[14]));
  always @ (*) begin
    _zz_89_[31] = _zz_88_;
    _zz_89_[30] = _zz_88_;
    _zz_89_[29] = _zz_88_;
    _zz_89_[28] = _zz_88_;
    _zz_89_[27] = _zz_88_;
    _zz_89_[26] = _zz_88_;
    _zz_89_[25] = _zz_88_;
    _zz_89_[24] = _zz_88_;
    _zz_89_[23] = _zz_88_;
    _zz_89_[22] = _zz_88_;
    _zz_89_[21] = _zz_88_;
    _zz_89_[20] = _zz_88_;
    _zz_89_[19] = _zz_88_;
    _zz_89_[18] = _zz_88_;
    _zz_89_[17] = _zz_88_;
    _zz_89_[16] = _zz_88_;
    _zz_89_[15] = _zz_88_;
    _zz_89_[14] = _zz_88_;
    _zz_89_[13] = _zz_88_;
    _zz_89_[12] = _zz_88_;
    _zz_89_[11] = _zz_88_;
    _zz_89_[10] = _zz_88_;
    _zz_89_[9] = _zz_88_;
    _zz_89_[8] = _zz_88_;
    _zz_89_[7 : 0] = execute_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_90_ = (execute_DBusSimplePlugin_rspShifted[15] && (! execute_INSTRUCTION[14]));
  always @ (*) begin
    _zz_91_[31] = _zz_90_;
    _zz_91_[30] = _zz_90_;
    _zz_91_[29] = _zz_90_;
    _zz_91_[28] = _zz_90_;
    _zz_91_[27] = _zz_90_;
    _zz_91_[26] = _zz_90_;
    _zz_91_[25] = _zz_90_;
    _zz_91_[24] = _zz_90_;
    _zz_91_[23] = _zz_90_;
    _zz_91_[22] = _zz_90_;
    _zz_91_[21] = _zz_90_;
    _zz_91_[20] = _zz_90_;
    _zz_91_[19] = _zz_90_;
    _zz_91_[18] = _zz_90_;
    _zz_91_[17] = _zz_90_;
    _zz_91_[16] = _zz_90_;
    _zz_91_[15 : 0] = execute_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_151_)
      2'b00 : begin
        execute_DBusSimplePlugin_rspFormated = _zz_89_;
      end
      2'b01 : begin
        execute_DBusSimplePlugin_rspFormated = _zz_91_;
      end
      default : begin
        execute_DBusSimplePlugin_rspFormated = execute_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign _zz_93_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001010000)) == (32'b00000000000000000100000001010000));
  assign _zz_94_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000010100)) == (32'b00000000000000000000000000000100));
  assign _zz_95_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_96_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000000100)) == (32'b00000000000000000000000000000100));
  assign _zz_92_ = {({(_zz_204_ == _zz_205_),(_zz_206_ == _zz_207_)} != (2'b00)),{({_zz_208_,_zz_209_} != (2'b00)),{({_zz_210_,_zz_211_} != (3'b000)),{(_zz_212_ != _zz_213_),{_zz_214_,{_zz_215_,_zz_216_}}}}}};
  assign _zz_59_ = _zz_167_[0];
  assign _zz_58_ = _zz_168_[0];
  assign _zz_97_ = _zz_92_[3 : 2];
  assign _zz_57_ = _zz_97_;
  assign _zz_56_ = _zz_169_[0];
  assign _zz_55_ = _zz_170_[0];
  assign _zz_98_ = _zz_92_[7 : 6];
  assign _zz_54_ = _zz_98_;
  assign _zz_99_ = _zz_92_[9 : 8];
  assign _zz_53_ = _zz_99_;
  assign _zz_100_ = _zz_92_[13 : 12];
  assign _zz_52_ = _zz_100_;
  assign _zz_101_ = _zz_92_[15 : 14];
  assign _zz_51_ = _zz_101_;
  assign _zz_102_ = _zz_92_[17 : 16];
  assign _zz_50_ = _zz_102_;
  assign _zz_49_ = _zz_171_[0];
  assign _zz_103_ = _zz_92_[20 : 19];
  assign _zz_48_ = _zz_103_;
  assign _zz_47_ = _zz_172_[0];
  assign _zz_46_ = _zz_173_[0];
  assign _zz_45_ = _zz_174_[0];
  assign execute_RegFilePlugin_srcInstruction = (execute_arbitration_isStuck ? execute_INSTRUCTION : decode_INSTRUCTION);
  assign execute_RegFilePlugin_regFileReadAddress1 = execute_RegFilePlugin_srcInstruction[19 : 15];
  assign execute_RegFilePlugin_regFileReadAddress2 = execute_RegFilePlugin_srcInstruction[24 : 20];
  assign execute_RegFilePlugin_rs1Data = _zz_136_;
  assign execute_RegFilePlugin_rs2Data = _zz_137_;
  assign _zz_44_ = execute_RegFilePlugin_rs1Data;
  assign _zz_43_ = execute_RegFilePlugin_rs2Data;
  assign execute_RegFilePlugin_regFileWrite_valid = (_zz_41_ && execute_arbitration_isFiring);
  assign execute_RegFilePlugin_regFileWrite_payload_address = _zz_40_[11 : 7];
  assign execute_RegFilePlugin_regFileWrite_payload_data = _zz_60_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = execute_SRC1;
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_104_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_104_ = {31'd0, _zz_175_};
      end
      default : begin
        _zz_104_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  assign _zz_38_ = _zz_104_;
  always @ (*) begin
    case(execute_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_105_ = execute_RS1;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_105_ = {29'd0, _zz_176_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_105_ = {execute_INSTRUCTION[31 : 12],(12'b000000000000)};
      end
      default : begin
        _zz_105_ = {27'd0, _zz_177_};
      end
    endcase
  end

  assign _zz_36_ = _zz_105_;
  assign _zz_106_ = _zz_178_[11];
  always @ (*) begin
    _zz_107_[19] = _zz_106_;
    _zz_107_[18] = _zz_106_;
    _zz_107_[17] = _zz_106_;
    _zz_107_[16] = _zz_106_;
    _zz_107_[15] = _zz_106_;
    _zz_107_[14] = _zz_106_;
    _zz_107_[13] = _zz_106_;
    _zz_107_[12] = _zz_106_;
    _zz_107_[11] = _zz_106_;
    _zz_107_[10] = _zz_106_;
    _zz_107_[9] = _zz_106_;
    _zz_107_[8] = _zz_106_;
    _zz_107_[7] = _zz_106_;
    _zz_107_[6] = _zz_106_;
    _zz_107_[5] = _zz_106_;
    _zz_107_[4] = _zz_106_;
    _zz_107_[3] = _zz_106_;
    _zz_107_[2] = _zz_106_;
    _zz_107_[1] = _zz_106_;
    _zz_107_[0] = _zz_106_;
  end

  assign _zz_108_ = _zz_179_[11];
  always @ (*) begin
    _zz_109_[19] = _zz_108_;
    _zz_109_[18] = _zz_108_;
    _zz_109_[17] = _zz_108_;
    _zz_109_[16] = _zz_108_;
    _zz_109_[15] = _zz_108_;
    _zz_109_[14] = _zz_108_;
    _zz_109_[13] = _zz_108_;
    _zz_109_[12] = _zz_108_;
    _zz_109_[11] = _zz_108_;
    _zz_109_[10] = _zz_108_;
    _zz_109_[9] = _zz_108_;
    _zz_109_[8] = _zz_108_;
    _zz_109_[7] = _zz_108_;
    _zz_109_[6] = _zz_108_;
    _zz_109_[5] = _zz_108_;
    _zz_109_[4] = _zz_108_;
    _zz_109_[3] = _zz_108_;
    _zz_109_[2] = _zz_108_;
    _zz_109_[1] = _zz_108_;
    _zz_109_[0] = _zz_108_;
  end

  always @ (*) begin
    case(execute_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_110_ = execute_RS2;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_110_ = {_zz_107_,execute_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_110_ = {_zz_109_,{execute_INSTRUCTION[31 : 25],execute_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_110_ = execute_PC;
      end
    endcase
  end

  assign _zz_34_ = _zz_110_;
  assign execute_SrcPlugin_addSub = _zz_180_;
  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign _zz_32_ = execute_SrcPlugin_addSub;
  assign _zz_31_ = execute_SrcPlugin_addSub;
  assign _zz_30_ = execute_SrcPlugin_less;
  assign execute_LightShifterPlugin_isShift = (execute_SHIFT_CTRL != `ShiftCtrlEnum_defaultEncoding_DISABLE_1);
  assign execute_LightShifterPlugin_amplitude = (execute_LightShifterPlugin_isActive ? execute_LightShifterPlugin_amplitudeReg : execute_SRC2[4 : 0]);
  assign execute_LightShifterPlugin_shiftInput = (execute_LightShifterPlugin_isActive ? execute_LightShifterPlugin_shiftReg : execute_SRC1);
  assign execute_LightShifterPlugin_done = (execute_LightShifterPlugin_amplitude[4 : 1] == (4'b0000));
  always @ (*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
        _zz_111_ = (execute_LightShifterPlugin_shiftInput <<< 1);
      end
      default : begin
        _zz_111_ = _zz_188_;
      end
    endcase
  end

  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_112_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_112_ == (3'b000))) begin
        _zz_113_ = execute_BranchPlugin_eq;
    end else if((_zz_112_ == (3'b001))) begin
        _zz_113_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_112_ & (3'b101)) == (3'b101)))) begin
        _zz_113_ = (! execute_SRC_LESS);
    end else begin
        _zz_113_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_114_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_114_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_114_ = 1'b1;
      end
      default : begin
        _zz_114_ = _zz_113_;
      end
    endcase
  end

  assign _zz_28_ = _zz_114_;
  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_115_ = _zz_190_[19];
  always @ (*) begin
    _zz_116_[10] = _zz_115_;
    _zz_116_[9] = _zz_115_;
    _zz_116_[8] = _zz_115_;
    _zz_116_[7] = _zz_115_;
    _zz_116_[6] = _zz_115_;
    _zz_116_[5] = _zz_115_;
    _zz_116_[4] = _zz_115_;
    _zz_116_[3] = _zz_115_;
    _zz_116_[2] = _zz_115_;
    _zz_116_[1] = _zz_115_;
    _zz_116_[0] = _zz_115_;
  end

  assign _zz_117_ = _zz_191_[11];
  always @ (*) begin
    _zz_118_[19] = _zz_117_;
    _zz_118_[18] = _zz_117_;
    _zz_118_[17] = _zz_117_;
    _zz_118_[16] = _zz_117_;
    _zz_118_[15] = _zz_117_;
    _zz_118_[14] = _zz_117_;
    _zz_118_[13] = _zz_117_;
    _zz_118_[12] = _zz_117_;
    _zz_118_[11] = _zz_117_;
    _zz_118_[10] = _zz_117_;
    _zz_118_[9] = _zz_117_;
    _zz_118_[8] = _zz_117_;
    _zz_118_[7] = _zz_117_;
    _zz_118_[6] = _zz_117_;
    _zz_118_[5] = _zz_117_;
    _zz_118_[4] = _zz_117_;
    _zz_118_[3] = _zz_117_;
    _zz_118_[2] = _zz_117_;
    _zz_118_[1] = _zz_117_;
    _zz_118_[0] = _zz_117_;
  end

  assign _zz_119_ = _zz_192_[11];
  always @ (*) begin
    _zz_120_[18] = _zz_119_;
    _zz_120_[17] = _zz_119_;
    _zz_120_[16] = _zz_119_;
    _zz_120_[15] = _zz_119_;
    _zz_120_[14] = _zz_119_;
    _zz_120_[13] = _zz_119_;
    _zz_120_[12] = _zz_119_;
    _zz_120_[11] = _zz_119_;
    _zz_120_[10] = _zz_119_;
    _zz_120_[9] = _zz_119_;
    _zz_120_[8] = _zz_119_;
    _zz_120_[7] = _zz_119_;
    _zz_120_[6] = _zz_119_;
    _zz_120_[5] = _zz_119_;
    _zz_120_[4] = _zz_119_;
    _zz_120_[3] = _zz_119_;
    _zz_120_[2] = _zz_119_;
    _zz_120_[1] = _zz_119_;
    _zz_120_[0] = _zz_119_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_121_ = {{_zz_116_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_121_ = {_zz_118_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_121_ = {{_zz_120_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_121_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign _zz_26_ = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign _zz_70_ = ((execute_arbitration_isValid && (! execute_arbitration_isStuckByOthers)) && execute_BRANCH_DO);
  assign _zz_71_ = execute_BRANCH_CALC;
  assign _zz_72_ = ((execute_arbitration_isValid && execute_BRANCH_DO) && _zz_71_[1]);
  always @ (*) begin
    _zz_122_ = 1'b0;
    _zz_123_ = 1'b0;
    if(_zz_124_)begin
      if((_zz_125_ == execute_INSTRUCTION[19 : 15]))begin
        _zz_122_ = 1'b1;
      end
      if((_zz_125_ == execute_INSTRUCTION[24 : 20]))begin
        _zz_123_ = 1'b1;
      end
    end
    if((! execute_RS1_USE))begin
      _zz_122_ = 1'b0;
    end
    if((! execute_RS2_USE))begin
      _zz_123_ = 1'b0;
    end
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = (26'b00000000000000000000000000);
  assign CsrPlugin_medeleg = (32'b00000000000000000000000000000000);
  assign CsrPlugin_mideleg = (32'b00000000000000000000000000000000);
  assign _zz_126_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_127_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_128_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  assign CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode = 1'b0;
  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege = CsrPlugin_privilege;
  assign execute_exception_agregat_valid = ((_zz_69_ || _zz_72_) || _zz_75_);
  assign _zz_129_ = {_zz_75_,{_zz_72_,_zz_69_}};
  assign _zz_130_ = (_zz_129_ & (~ _zz_193_));
  assign _zz_131_ = _zz_130_[1];
  assign _zz_132_ = _zz_130_[2];
  assign _zz_133_ = {_zz_132_,_zz_131_};
  assign execute_exception_agregat_payload_code = _zz_138_;
  assign execute_exception_agregat_payload_badAddr = _zz_139_;
  assign CsrPlugin_exceptionPortCtrl_exceptionValids_decode = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  always @ (*) begin
    CsrPlugin_interrupt = 1'b0;
    CsrPlugin_interruptCode = (4'bxxxx);
    CsrPlugin_interruptTargetPrivilege = (2'bxx);
    if(CsrPlugin_mstatus_MIE)begin
      if(((_zz_126_ || _zz_127_) || _zz_128_))begin
        CsrPlugin_interrupt = 1'b1;
      end
      if(_zz_126_)begin
        CsrPlugin_interruptCode = (4'b0111);
        CsrPlugin_interruptTargetPrivilege = (2'b11);
      end
      if(_zz_127_)begin
        CsrPlugin_interruptCode = (4'b0011);
        CsrPlugin_interruptTargetPrivilege = (2'b11);
      end
      if(_zz_128_)begin
        CsrPlugin_interruptCode = (4'b1011);
        CsrPlugin_interruptTargetPrivilege = (2'b11);
      end
    end
    if((! 1'b1))begin
      CsrPlugin_interrupt = 1'b0;
    end
  end

  assign CsrPlugin_exception = (CsrPlugin_exceptionPortCtrl_exceptionValids_execute && 1'b1);
  assign CsrPlugin_lastStageWasWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = ((! execute_arbitration_isValid) && IBusSimplePlugin_injector_nextPcCalc_1);
    if(CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = (CsrPlugin_interrupt && CsrPlugin_pipelineLiberator_done);
  always @ (*) begin
    CsrPlugin_targetPrivilege = CsrPlugin_interruptTargetPrivilege;
    if(CsrPlugin_hadException)begin
      CsrPlugin_targetPrivilege = CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
    end
  end

  always @ (*) begin
    CsrPlugin_trapCause = CsrPlugin_interruptCode;
    if(CsrPlugin_hadException)begin
      CsrPlugin_trapCause = CsrPlugin_exceptionPortCtrl_exceptionContext_code;
    end
  end

  assign contextSwitching = _zz_73_;
  assign _zz_23_ = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == (5'b00000))) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == (5'b00000)))));
  assign _zz_22_ = (decode_INSTRUCTION[13 : 7] != (7'b0100000));
  assign execute_CsrPlugin_blockedBySideEffects = 1'b0;
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = (execute_arbitration_isValid && execute_IS_CSR);
    execute_CsrPlugin_readData = (32'b00000000000000000000000000000000);
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[12 : 11] = CsrPlugin_mstatus_MPP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mstatus_MPIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mstatus_MIE;
      end
      12'b001101000001 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[31 : 0] = CsrPlugin_mepc;
      end
      12'b001100000101 : begin
        if(execute_CSR_WRITE_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      12'b001101000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mip_MEIP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mip_MTIP;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mip_MSIP;
      end
      12'b001101000011 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
        execute_CsrPlugin_readData[31 : 0] = CsrPlugin_mtval;
      end
      12'b001101000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[31 : 0] = CsrPlugin_mscratch;
      end
      12'b001100000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mie_MEIE;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mie_MTIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mie_MSIE;
      end
      12'b001101000010 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
        execute_CsrPlugin_readData[31 : 31] = CsrPlugin_mcause_interrupt;
        execute_CsrPlugin_readData[3 : 0] = CsrPlugin_mcause_exceptionCode;
      end
      default : begin
      end
    endcase
    if((CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]))begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((execute_INSTRUCTION[29 : 28] != CsrPlugin_privilege))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  always @ (*) begin
    _zz_75_ = 1'b0;
    _zz_76_ = (4'bxxxx);
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_ECALL)))begin
      _zz_75_ = 1'b1;
      _zz_76_ = (4'b1011);
    end
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_EBREAK)))begin
      _zz_75_ = 1'b1;
      _zz_76_ = (4'b0011);
    end
  end

  assign execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
  assign execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
  assign execute_CsrPlugin_writeEnable = ((execute_CsrPlugin_writeInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readEnable = ((execute_CsrPlugin_readInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  always @ (*) begin
    case(_zz_153_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readData & (~ execute_SRC1)) : (execute_CsrPlugin_readData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign _zz_21_ = decode_ALU_BITWISE_CTRL;
  assign _zz_19_ = _zz_51_;
  assign _zz_39_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_18_ = decode_ENV_CTRL;
  assign _zz_16_ = _zz_54_;
  assign _zz_24_ = decode_to_execute_ENV_CTRL;
  assign _zz_15_ = decode_BRANCH_CTRL;
  assign _zz_13_ = _zz_52_;
  assign _zz_27_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_12_ = decode_SRC2_CTRL;
  assign _zz_10_ = _zz_48_;
  assign _zz_33_ = decode_to_execute_SRC2_CTRL;
  assign _zz_9_ = decode_ALU_CTRL;
  assign _zz_7_ = _zz_53_;
  assign _zz_37_ = decode_to_execute_ALU_CTRL;
  assign _zz_6_ = decode_SHIFT_CTRL;
  assign _zz_4_ = _zz_50_;
  assign _zz_29_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_3_ = decode_SRC1_CTRL;
  assign _zz_1_ = _zz_57_;
  assign _zz_35_ = decode_to_execute_SRC1_CTRL;
  assign decode_arbitration_isFlushed = (decode_arbitration_flushAll || execute_arbitration_flushAll);
  assign execute_arbitration_isFlushed = execute_arbitration_flushAll;
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (1'b0 || execute_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || 1'b0);
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      CsrPlugin_privilege <= (2'b11);
      IBusSimplePlugin_fetchPc_pcReg <= (32'b00000000000010100000000000000000);
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      _zz_79_ <= 1'b0;
      _zz_84_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      IBusSimplePlugin_pendingCmd <= (2'b00);
      IBusSimplePlugin_rspJoin_discardCounter <= (2'b00);
      execute_DBusSimplePlugin_cmdSent <= 1'b0;
      execute_LightShifterPlugin_isActive <= 1'b0;
      _zz_124_ <= 1'b0;
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mip_MEIP <= 1'b0;
      CsrPlugin_mip_MTIP <= 1'b0;
      CsrPlugin_mip_MSIP <= 1'b0;
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      execute_arbitration_isValid <= 1'b0;
    end else begin
      if(IBusSimplePlugin_fetchPc_propagatePc)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(IBusSimplePlugin_jump_pcLoad_valid)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(_zz_149_)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(IBusSimplePlugin_fetchPc_samplePcNext)begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      _zz_79_ <= 1'b1;
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_68_))begin
        _zz_84_ <= 1'b0;
      end
      if(_zz_82_)begin
        _zz_84_ <= IBusSimplePlugin_iBusRsp_stages_0_output_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_68_))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_1_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_68_))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= IBusSimplePlugin_injector_nextPcCalc_0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_68_))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_68_))begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      end
      IBusSimplePlugin_pendingCmd <= IBusSimplePlugin_pendingCmdNext;
      IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_rspJoin_discardCounter - _zz_164_);
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_68_))begin
        IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_pendingCmd - _zz_166_);
      end
      if((dBus_cmd_valid && dBus_cmd_ready))begin
        execute_DBusSimplePlugin_cmdSent <= 1'b1;
      end
      if((! execute_arbitration_isStuck))begin
        execute_DBusSimplePlugin_cmdSent <= 1'b0;
      end
      if(_zz_145_)begin
        if(_zz_146_)begin
          execute_LightShifterPlugin_isActive <= 1'b1;
          if(execute_LightShifterPlugin_done)begin
            execute_LightShifterPlugin_isActive <= 1'b0;
          end
        end
      end
      if(execute_arbitration_removeIt)begin
        execute_LightShifterPlugin_isActive <= 1'b0;
      end
      _zz_124_ <= (_zz_41_ && execute_arbitration_isFiring);
      CsrPlugin_mip_MEIP <= externalInterrupt;
      CsrPlugin_mip_MTIP <= timerInterrupt;
      if((! execute_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      end
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_147_)begin
        if(_zz_150_)begin
          case(CsrPlugin_targetPrivilege)
            2'b11 : begin
              CsrPlugin_mstatus_MIE <= 1'b0;
              CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
              CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
            end
            default : begin
            end
          endcase
        end
      end
      if(_zz_148_)begin
        case(_zz_152_)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MPIE <= 1'b1;
            CsrPlugin_privilege <= CsrPlugin_mstatus_MPP;
          end
          default : begin
          end
        endcase
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      case(execute_CsrPlugin_csrAddress)
        12'b001100000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
            CsrPlugin_mstatus_MPIE <= _zz_196_[0];
            CsrPlugin_mstatus_MIE <= _zz_197_[0];
          end
        end
        12'b001101000001 : begin
        end
        12'b001100000101 : begin
        end
        12'b001101000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mip_MSIP <= _zz_198_[0];
          end
        end
        12'b001101000011 : begin
        end
        12'b001101000000 : begin
        end
        12'b001100000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mie_MEIE <= _zz_199_[0];
            CsrPlugin_mie_MTIE <= _zz_200_[0];
            CsrPlugin_mie_MSIE <= _zz_201_[0];
          end
        end
        12'b001101000010 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_clk) begin
    if((! execute_arbitration_isStuckByOthers))begin
      execute_LightShifterPlugin_shiftReg <= _zz_60_;
    end
    if(_zz_145_)begin
      if(_zz_146_)begin
        execute_LightShifterPlugin_amplitudeReg <= (execute_LightShifterPlugin_amplitude - (5'b00001));
      end
    end
    _zz_125_ <= _zz_40_[11 : 7];
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    if(execute_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
    CsrPlugin_exceptionPortCtrl_emulationRequiredReg <= CsrPlugin_exceptionPortCtrl_emulationRequired;
    if(execute_exception_agregat_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= execute_exception_agregat_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= execute_exception_agregat_payload_badAddr;
    end
    if(((CsrPlugin_exception && 1'b1) || CsrPlugin_interruptJump))begin
      case(CsrPlugin_privilege)
        2'b11 : begin
          CsrPlugin_mepc <= execute_PC;
        end
        default : begin
        end
      endcase
    end
    if(_zz_147_)begin
      if(_zz_150_)begin
        case(CsrPlugin_targetPrivilege)
          2'b11 : begin
            CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
            CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
            CsrPlugin_mtval <= CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
          end
          default : begin
          end
        endcase
      end
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_FENCEI <= decode_IS_FENCEI;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_20_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= decode_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_17_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_14_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= _zz_25_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1_USE <= decode_RS1_USE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_CTRL <= _zz_11_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2_USE <= decode_RS2_USE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_8_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_5_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1_CTRL <= _zz_2_;
    end
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
      end
      12'b001101000001 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mepc <= execute_CsrPlugin_writeData[31 : 0];
        end
      end
      12'b001100000101 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mtvec_base <= execute_CsrPlugin_writeData[31 : 2];
          CsrPlugin_mtvec_mode <= execute_CsrPlugin_writeData[1 : 0];
        end
      end
      12'b001101000100 : begin
      end
      12'b001101000011 : begin
      end
      12'b001101000000 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mscratch <= execute_CsrPlugin_writeData[31 : 0];
        end
      end
      12'b001100000100 : begin
      end
      12'b001101000010 : begin
      end
      default : begin
      end
    endcase
  end

endmodule

module SimpleBusDecoder (
      input   io_input_cmd_valid,
      output  io_input_cmd_ready,
      input   io_input_cmd_payload_wr,
      input  [19:0] io_input_cmd_payload_address,
      input  [31:0] io_input_cmd_payload_data,
      input  [3:0] io_input_cmd_payload_mask,
      output  io_input_rsp_valid,
      output [31:0] io_input_rsp_payload_data,
      output  io_outputs_0_cmd_valid,
      input   io_outputs_0_cmd_ready,
      output  io_outputs_0_cmd_payload_wr,
      output [19:0] io_outputs_0_cmd_payload_address,
      output [31:0] io_outputs_0_cmd_payload_data,
      output [3:0] io_outputs_0_cmd_payload_mask,
      input   io_outputs_0_rsp_valid,
      input  [31:0] io_outputs_0_rsp_payload_data);
  assign io_outputs_0_cmd_valid = io_input_cmd_valid;
  assign io_input_cmd_ready = io_outputs_0_cmd_ready;
  assign io_outputs_0_cmd_payload_wr = io_input_cmd_payload_wr;
  assign io_outputs_0_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_0_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_0_cmd_payload_mask = io_input_cmd_payload_mask;
  assign io_input_rsp_valid = io_outputs_0_rsp_valid;
  assign io_input_rsp_payload_data = io_outputs_0_rsp_payload_data;
endmodule


//SimpleBusDecoder_1_ remplaced by SimpleBusDecoder

module SimpleBusDecoder_2_ (
      input   io_input_cmd_valid,
      output reg  io_input_cmd_ready,
      input   io_input_cmd_payload_wr,
      input  [19:0] io_input_cmd_payload_address,
      input  [31:0] io_input_cmd_payload_data,
      input  [3:0] io_input_cmd_payload_mask,
      output  io_input_rsp_valid,
      output [31:0] io_input_rsp_payload_data,
      output reg  io_outputs_0_cmd_valid,
      input   io_outputs_0_cmd_ready,
      output  io_outputs_0_cmd_payload_wr,
      output [19:0] io_outputs_0_cmd_payload_address,
      output [31:0] io_outputs_0_cmd_payload_data,
      output [3:0] io_outputs_0_cmd_payload_mask,
      input   io_outputs_0_rsp_valid,
      input  [31:0] io_outputs_0_rsp_0_data,
      output reg  io_outputs_1_cmd_valid,
      input   io_outputs_1_cmd_ready,
      output  io_outputs_1_cmd_payload_wr,
      output [19:0] io_outputs_1_cmd_payload_address,
      output [31:0] io_outputs_1_cmd_payload_data,
      output [3:0] io_outputs_1_cmd_payload_mask,
      input   io_outputs_1_rsp_valid,
      input  [31:0] io_outputs_1_rsp_1_data,
      output reg  io_outputs_2_cmd_valid,
      input   io_outputs_2_cmd_ready,
      output  io_outputs_2_cmd_payload_wr,
      output [19:0] io_outputs_2_cmd_payload_address,
      output [31:0] io_outputs_2_cmd_payload_data,
      output [3:0] io_outputs_2_cmd_payload_mask,
      input   io_outputs_2_rsp_valid,
      input  [31:0] io_outputs_2_rsp_2_data,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  reg [31:0] _zz_4_;
  wire [19:0] _zz_5_;
  wire [19:0] _zz_6_;
  wire [19:0] _zz_7_;
  wire [1:0] _zz_8_;
  wire [0:0] _zz_9_;
  wire [1:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [1:0] _zz_12_;
  wire [1:0] _zz_13_;
  wire  logic_hits_0;
  wire  logic_hits_1;
  wire  logic_hits_2;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  logic_noHit;
  reg [1:0] logic_rspPendingCounter;
  reg  logic_rspHits_0;
  reg  logic_rspHits_1;
  reg  logic_rspHits_2;
  wire  logic_rspPending;
  wire  logic_rspNoHit;
  wire  logic_cmdWait;
  assign _zz_5_ = (20'b11110000000000000000);
  assign _zz_6_ = (20'b11110000000000000000);
  assign _zz_7_ = (20'b10000000000000000000);
  assign _zz_8_ = (logic_rspPendingCounter + _zz_10_);
  assign _zz_9_ = ((io_input_cmd_valid && io_input_cmd_ready) && (! io_input_cmd_payload_wr));
  assign _zz_10_ = {1'd0, _zz_9_};
  assign _zz_11_ = io_input_rsp_valid;
  assign _zz_12_ = {1'd0, _zz_11_};
  assign _zz_13_ = {logic_rspHits_2,logic_rspHits_1};
  always @(*) begin
    case(_zz_13_)
      2'b00 : begin
        _zz_4_ = io_outputs_0_rsp_0_data;
      end
      2'b01 : begin
        _zz_4_ = io_outputs_1_rsp_1_data;
      end
      default : begin
        _zz_4_ = io_outputs_2_rsp_2_data;
      end
    endcase
  end

  assign logic_hits_0 = ((io_input_cmd_payload_address & _zz_5_) == (20'b00000000000000000000));
  always @ (*) begin
    io_outputs_0_cmd_valid = (io_input_cmd_valid && logic_hits_0);
    io_outputs_1_cmd_valid = (io_input_cmd_valid && logic_hits_1);
    io_outputs_2_cmd_valid = (io_input_cmd_valid && logic_hits_2);
    io_input_cmd_ready = ((((logic_hits_0 && io_outputs_0_cmd_ready) || (logic_hits_1 && io_outputs_1_cmd_ready)) || (logic_hits_2 && io_outputs_2_cmd_ready)) || logic_noHit);
    if(logic_cmdWait)begin
      io_input_cmd_ready = 1'b0;
      io_outputs_0_cmd_valid = 1'b0;
      io_outputs_1_cmd_valid = 1'b0;
      io_outputs_2_cmd_valid = 1'b0;
    end
  end

  assign _zz_1_ = io_input_cmd_payload_wr;
  assign io_outputs_0_cmd_payload_wr = _zz_1_;
  assign io_outputs_0_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_0_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_0_cmd_payload_mask = io_input_cmd_payload_mask;
  assign logic_hits_1 = ((io_input_cmd_payload_address & _zz_6_) == (20'b01110000000000000000));
  assign _zz_2_ = io_input_cmd_payload_wr;
  assign io_outputs_1_cmd_payload_wr = _zz_2_;
  assign io_outputs_1_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_1_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_1_cmd_payload_mask = io_input_cmd_payload_mask;
  assign logic_hits_2 = ((io_input_cmd_payload_address & _zz_7_) == (20'b10000000000000000000));
  assign _zz_3_ = io_input_cmd_payload_wr;
  assign io_outputs_2_cmd_payload_wr = _zz_3_;
  assign io_outputs_2_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_2_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_2_cmd_payload_mask = io_input_cmd_payload_mask;
  assign logic_noHit = (! ((logic_hits_0 || logic_hits_1) || logic_hits_2));
  assign logic_rspPending = (logic_rspPendingCounter != (2'b00));
  assign logic_rspNoHit = (! ((logic_rspHits_0 || logic_rspHits_1) || logic_rspHits_2));
  assign io_input_rsp_valid = (((io_outputs_0_rsp_valid || io_outputs_1_rsp_valid) || io_outputs_2_rsp_valid) || (logic_rspPending && logic_rspNoHit));
  assign io_input_rsp_payload_data = _zz_4_;
  assign logic_cmdWait = (((io_input_cmd_valid && logic_rspPending) && (((logic_hits_0 != logic_rspHits_0) || (logic_hits_1 != logic_rspHits_1)) || (logic_hits_2 != logic_rspHits_2))) || (logic_rspPendingCounter == (2'b11)));
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      logic_rspPendingCounter <= (2'b00);
    end else begin
      logic_rspPendingCounter <= (_zz_8_ - _zz_12_);
    end
  end

  always @ (posedge io_clk) begin
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      logic_rspHits_0 <= logic_hits_0;
      logic_rspHits_1 <= logic_hits_1;
      logic_rspHits_2 <= logic_hits_2;
    end
  end

endmodule

module SimpleBusArbiter (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [15:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [15:0] io_output_cmd_payload_address,
      output [31:0] io_output_cmd_payload_data,
      output [3:0] io_output_cmd_payload_mask,
      input   io_output_rsp_valid,
      input  [31:0] io_output_rsp_payload_data);
  assign io_output_cmd_valid = io_inputs_0_cmd_valid;
  assign io_output_cmd_payload_wr = io_inputs_0_cmd_payload_wr;
  assign io_output_cmd_payload_address = io_inputs_0_cmd_payload_address;
  assign io_output_cmd_payload_data = io_inputs_0_cmd_payload_data;
  assign io_output_cmd_payload_mask = io_inputs_0_cmd_payload_mask;
  assign io_inputs_0_cmd_ready = io_output_cmd_ready;
  assign io_inputs_0_rsp_valid = io_output_rsp_valid;
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
endmodule

module SimpleBusArbiter_1_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [5:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [5:0] io_output_cmd_payload_address,
      output [31:0] io_output_cmd_payload_data,
      output [3:0] io_output_cmd_payload_mask,
      input   io_output_rsp_valid,
      input  [31:0] io_output_rsp_payload_data);
  assign io_output_cmd_valid = io_inputs_0_cmd_valid;
  assign io_output_cmd_payload_wr = io_inputs_0_cmd_payload_wr;
  assign io_output_cmd_payload_address = io_inputs_0_cmd_payload_address;
  assign io_output_cmd_payload_data = io_inputs_0_cmd_payload_data;
  assign io_output_cmd_payload_mask = io_inputs_0_cmd_payload_mask;
  assign io_inputs_0_cmd_ready = io_output_cmd_ready;
  assign io_inputs_0_rsp_valid = io_output_rsp_valid;
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
endmodule

module SimpleBusArbiter_2_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [18:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [18:0] io_output_cmd_payload_address,
      output [31:0] io_output_cmd_payload_data,
      output [3:0] io_output_cmd_payload_mask,
      input   io_output_rsp_valid,
      input  [31:0] io_output_rsp_payload_data);
  assign io_output_cmd_valid = io_inputs_0_cmd_valid;
  assign io_output_cmd_payload_wr = io_inputs_0_cmd_payload_wr;
  assign io_output_cmd_payload_address = io_inputs_0_cmd_payload_address;
  assign io_output_cmd_payload_data = io_inputs_0_cmd_payload_data;
  assign io_output_cmd_payload_mask = io_inputs_0_cmd_payload_mask;
  assign io_inputs_0_cmd_ready = io_output_cmd_ready;
  assign io_inputs_0_rsp_valid = io_output_rsp_valid;
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
endmodule

module SimpleBusArbiter_3_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [19:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      input   io_inputs_1_cmd_valid,
      output  io_inputs_1_cmd_ready,
      input   io_inputs_1_cmd_payload_wr,
      input  [19:0] io_inputs_1_cmd_payload_address,
      input  [31:0] io_inputs_1_cmd_payload_data,
      input  [3:0] io_inputs_1_cmd_payload_mask,
      output  io_inputs_1_rsp_valid,
      output [31:0] io_inputs_1_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [19:0] io_output_cmd_payload_address,
      output [31:0] io_output_cmd_payload_data,
      output [3:0] io_output_cmd_payload_mask,
      input   io_output_rsp_valid,
      input  [31:0] io_output_rsp_payload_data,
      input   io_clk,
      input   GLOBAL_BUFFER_OUTPUT);
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire [19:0] _zz_7_;
  wire [31:0] _zz_8_;
  wire [3:0] _zz_9_;
  wire [0:0] _zz_10_;
  wire [1:0] _zz_11_;
  wire  _zz_12_;
  wire [1:0] logic_rspRouteOh;
  reg  logic_rsp_pending;
  reg [1:0] logic_rsp_target;
  wire  _zz_1_;
  assign _zz_12_ = ((io_output_cmd_valid && io_output_cmd_ready) && (! io_output_cmd_payload_wr));
  StreamArbiter logic_arbiter ( 
    .io_inputs_0_0(io_inputs_0_cmd_valid),
    .io_inputs_0_ready(_zz_3_),
    .io_inputs_0_0_wr(io_inputs_0_cmd_payload_wr),
    .io_inputs_0_0_address(io_inputs_0_cmd_payload_address),
    .io_inputs_0_0_data(io_inputs_0_cmd_payload_data),
    .io_inputs_0_0_mask(io_inputs_0_cmd_payload_mask),
    .io_inputs_1_1(io_inputs_1_cmd_valid),
    .io_inputs_1_ready(_zz_4_),
    .io_inputs_1_1_wr(io_inputs_1_cmd_payload_wr),
    .io_inputs_1_1_address(io_inputs_1_cmd_payload_address),
    .io_inputs_1_1_data(io_inputs_1_cmd_payload_data),
    .io_inputs_1_1_mask(io_inputs_1_cmd_payload_mask),
    .io_output_valid(_zz_5_),
    .io_output_ready(_zz_2_),
    .io_output_payload_wr(_zz_6_),
    .io_output_payload_address(_zz_7_),
    .io_output_payload_data(_zz_8_),
    .io_output_payload_mask(_zz_9_),
    .io_chosen(_zz_10_),
    .io_chosenOH(_zz_11_),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(GLOBAL_BUFFER_OUTPUT) 
  );
  assign io_inputs_0_cmd_ready = _zz_3_;
  assign io_inputs_1_cmd_ready = _zz_4_;
  assign logic_rspRouteOh = logic_rsp_target;
  assign _zz_1_ = (! (logic_rsp_pending && (! io_output_rsp_valid)));
  assign _zz_2_ = (io_output_cmd_ready && _zz_1_);
  assign io_output_cmd_valid = (_zz_5_ && _zz_1_);
  assign io_output_cmd_payload_wr = _zz_6_;
  assign io_output_cmd_payload_address = _zz_7_;
  assign io_output_cmd_payload_data = _zz_8_;
  assign io_output_cmd_payload_mask = _zz_9_;
  assign io_inputs_0_rsp_valid = (io_output_rsp_valid && logic_rspRouteOh[0]);
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
  assign io_inputs_1_rsp_valid = (io_output_rsp_valid && logic_rspRouteOh[1]);
  assign io_inputs_1_rsp_payload_data = io_output_rsp_payload_data;
  always @ (posedge io_clk) begin
    if(GLOBAL_BUFFER_OUTPUT) begin
      logic_rsp_pending <= 1'b0;
    end else begin
      if(io_output_rsp_valid)begin
        logic_rsp_pending <= 1'b0;
      end
      if(_zz_12_)begin
        logic_rsp_pending <= 1'b1;
      end
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_12_)begin
      logic_rsp_target <= _zz_11_;
    end
  end

endmodule

module Up5kArea (
      input   io_clk,
      input   io_reset,
      output [2:0] io_leds,
      output  io_serialTx,
      output [0:0] io_flash_ss,
      output  io_flash_sclk,
      output  io_flash_mosi,
      input   io_flash_miso);
  wire  _zz_7_;
  wire  _zz_8_;
  wire  _zz_9_;
  wire [15:0] _zz_10_;
  wire [5:0] _zz_11_;
  wire [18:0] _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire  _zz_16_;
  wire [31:0] _zz_17_;
  wire  _zz_18_;
  wire  _zz_19_;
  wire [31:0] _zz_20_;
  wire  _zz_21_;
  wire [2:0] _zz_22_;
  wire  _zz_23_;
  wire  _zz_24_;
  wire  _zz_25_;
  wire [31:0] _zz_26_;
  wire  _zz_27_;
  wire  _zz_28_;
  wire [0:0] _zz_29_;
  wire  _zz_30_;
  wire [31:0] _zz_31_;
  wire  _zz_32_;
  wire  _zz_33_;
  wire [31:0] _zz_34_;
  wire [31:0] _zz_35_;
  wire [1:0] _zz_36_;
  wire  _zz_37_;
  wire  _zz_38_;
  wire [31:0] _zz_39_;
  wire  _zz_40_;
  wire  _zz_41_;
  wire [19:0] _zz_42_;
  wire [31:0] _zz_43_;
  wire [3:0] _zz_44_;
  wire  _zz_45_;
  wire  _zz_46_;
  wire [31:0] _zz_47_;
  wire  _zz_48_;
  wire  _zz_49_;
  wire [19:0] _zz_50_;
  wire [31:0] _zz_51_;
  wire [3:0] _zz_52_;
  wire  _zz_53_;
  wire  _zz_54_;
  wire [31:0] _zz_55_;
  wire  _zz_56_;
  wire  _zz_57_;
  wire [19:0] _zz_58_;
  wire [31:0] _zz_59_;
  wire [3:0] _zz_60_;
  wire  _zz_61_;
  wire  _zz_62_;
  wire [19:0] _zz_63_;
  wire [31:0] _zz_64_;
  wire [3:0] _zz_65_;
  wire  _zz_66_;
  wire  _zz_67_;
  wire [19:0] _zz_68_;
  wire [31:0] _zz_69_;
  wire [3:0] _zz_70_;
  wire  _zz_71_;
  wire  _zz_72_;
  wire [31:0] _zz_73_;
  wire  _zz_74_;
  wire  _zz_75_;
  wire [15:0] _zz_76_;
  wire [31:0] _zz_77_;
  wire [3:0] _zz_78_;
  wire  _zz_79_;
  wire  _zz_80_;
  wire [31:0] _zz_81_;
  wire  _zz_82_;
  wire  _zz_83_;
  wire [5:0] _zz_84_;
  wire [31:0] _zz_85_;
  wire [3:0] _zz_86_;
  wire  _zz_87_;
  wire  _zz_88_;
  wire [31:0] _zz_89_;
  wire  _zz_90_;
  wire  _zz_91_;
  wire [18:0] _zz_92_;
  wire [31:0] _zz_93_;
  wire [3:0] _zz_94_;
  wire  _zz_95_;
  wire  _zz_96_;
  wire [31:0] _zz_97_;
  wire  _zz_98_;
  wire  _zz_99_;
  wire [31:0] _zz_100_;
  wire  _zz_101_;
  wire  _zz_102_;
  wire [19:0] _zz_103_;
  wire [31:0] _zz_104_;
  wire [3:0] _zz_105_;
  wire  _zz_106_;
  wire  _zz_107_;
  reg  resetCtrl_resetUnbuffered;
  reg [5:0] resetCtrl_resetCounter = (6'b000000);
  reg  resetCtrl_systemResetBuffered;
  wire  system_dBus_cmd_valid;
  wire  system_dBus_cmd_ready;
  wire  system_dBus_cmd_payload_wr;
  wire [19:0] system_dBus_cmd_payload_address;
  wire [31:0] system_dBus_cmd_payload_data;
  wire [3:0] system_dBus_cmd_payload_mask;
  wire  system_dBus_rsp_valid;
  wire [31:0] system_dBus_rsp_payload_data;
  wire  system_iBus_cmd_valid;
  wire  system_iBus_cmd_ready;
  wire  system_iBus_cmd_payload_wr;
  wire [19:0] system_iBus_cmd_payload_address;
  wire [31:0] system_iBus_cmd_payload_data;
  wire [3:0] system_iBus_cmd_payload_mask;
  wire  system_iBus_rsp_valid;
  wire [31:0] system_iBus_rsp_payload_data;
  wire  system_mainBus_cmd_valid;
  wire  system_mainBus_cmd_ready;
  wire  system_mainBus_cmd_payload_wr;
  wire [19:0] system_mainBus_cmd_payload_address;
  wire [31:0] system_mainBus_cmd_payload_data;
  wire [3:0] system_mainBus_cmd_payload_mask;
  wire  system_mainBus_rsp_valid;
  wire [31:0] system_mainBus_rsp_payload_data;
  reg [0:0] io_flash_ss_regNext;
  reg  io_flash_sclk_regNext;
  reg  io_flash_mosi_regNext;
  reg [3:0] _zz_1_;
  wire  system_mainBus_cmd_s2mPipe_valid;
  wire  system_mainBus_cmd_s2mPipe_ready;
  wire  system_mainBus_cmd_s2mPipe_payload_wr;
  wire [19:0] system_mainBus_cmd_s2mPipe_payload_address;
  wire [31:0] system_mainBus_cmd_s2mPipe_payload_data;
  wire [3:0] system_mainBus_cmd_s2mPipe_payload_mask;
  reg  _zz_2_;
  reg  _zz_3_;
  reg [19:0] _zz_4_;
  reg [31:0] _zz_5_;
  reg [3:0] _zz_6_;
  assign _zz_106_ = (! (resetCtrl_resetCounter == (6'b111111)));
  assign _zz_107_ = (system_mainBus_cmd_ready && (! system_mainBus_cmd_s2mPipe_ready));
  BufferCC bufferCC_1_ ( 
    .io_dataIn(io_reset),
    .io_dataOut(_zz_13_),
    .io_clk(io_clk) 
  );
  SB_GB resetCtrl_systemResetBuffered_SB_GB ( 
    .USER_SIGNAL_TO_GLOBAL_BUFFER(resetCtrl_systemResetBuffered),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  Spram system_ram ( 
    .io_bus_cmd_valid(_zz_74_),
    .io_bus_cmd_ready(_zz_15_),
    .io_bus_cmd_payload_wr(_zz_75_),
    .io_bus_cmd_payload_address(_zz_76_),
    .io_bus_cmd_payload_data(_zz_77_),
    .io_bus_cmd_payload_mask(_zz_78_),
    .io_bus_rsp_valid(_zz_16_),
    .io_bus_rsp_payload_data(_zz_17_),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  Peripherals system_peripherals ( 
    .io_bus_cmd_valid(_zz_82_),
    .io_bus_cmd_ready(_zz_18_),
    .io_bus_cmd_payload_wr(_zz_83_),
    .io_bus_cmd_payload_address(_zz_84_),
    .io_bus_cmd_payload_data(_zz_85_),
    .io_bus_cmd_payload_mask(_zz_86_),
    .io_bus_rsp_valid(_zz_19_),
    .io_bus_rsp_payload_data(_zz_20_),
    .io_mTimeInterrupt(_zz_21_),
    .io_leds(_zz_22_),
    .io_serialTx(_zz_23_),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  FlashXpi system_flashXip ( 
    .io_bus_cmd_valid(_zz_90_),
    .io_bus_cmd_ready(_zz_24_),
    .io_bus_cmd_payload_wr(_zz_91_),
    .io_bus_cmd_payload_address(_zz_92_),
    .io_bus_cmd_payload_data(_zz_93_),
    .io_bus_cmd_payload_mask(_zz_94_),
    .io_bus_rsp_valid(_zz_25_),
    .io_bus_rsp_payload_data(_zz_26_),
    .io_flash_ss(_zz_29_),
    .io_flash_sclk(_zz_27_),
    .io_flash_mosi(_zz_28_),
    .io_flash_miso(io_flash_miso),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  VexRiscv system_cpu ( 
    .iBus_cmd_valid(_zz_30_),
    .iBus_cmd_ready(system_iBus_cmd_ready),
    .iBus_cmd_payload_pc(_zz_31_),
    .iBus_rsp_valid(system_iBus_rsp_valid),
    .iBus_rsp_payload_error(_zz_7_),
    .iBus_rsp_payload_inst(system_iBus_rsp_payload_data),
    .timerInterrupt(_zz_21_),
    .externalInterrupt(_zz_8_),
    .dBus_cmd_valid(_zz_32_),
    .dBus_cmd_ready(system_dBus_cmd_ready),
    .dBus_cmd_payload_wr(_zz_33_),
    .dBus_cmd_payload_address(_zz_34_),
    .dBus_cmd_payload_data(_zz_35_),
    .dBus_cmd_payload_size(_zz_36_),
    .dBus_rsp_ready(system_dBus_rsp_valid),
    .dBus_rsp_error(_zz_9_),
    .dBus_rsp_data(system_dBus_rsp_payload_data),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  SimpleBusDecoder system_dBus_decoder ( 
    .io_input_cmd_valid(system_dBus_cmd_valid),
    .io_input_cmd_ready(_zz_37_),
    .io_input_cmd_payload_wr(system_dBus_cmd_payload_wr),
    .io_input_cmd_payload_address(system_dBus_cmd_payload_address),
    .io_input_cmd_payload_data(system_dBus_cmd_payload_data),
    .io_input_cmd_payload_mask(system_dBus_cmd_payload_mask),
    .io_input_rsp_valid(_zz_38_),
    .io_input_rsp_payload_data(_zz_39_),
    .io_outputs_0_cmd_valid(_zz_40_),
    .io_outputs_0_cmd_ready(_zz_95_),
    .io_outputs_0_cmd_payload_wr(_zz_41_),
    .io_outputs_0_cmd_payload_address(_zz_42_),
    .io_outputs_0_cmd_payload_data(_zz_43_),
    .io_outputs_0_cmd_payload_mask(_zz_44_),
    .io_outputs_0_rsp_valid(_zz_96_),
    .io_outputs_0_rsp_payload_data(_zz_97_) 
  );
  SimpleBusDecoder system_iBus_decoder ( 
    .io_input_cmd_valid(system_iBus_cmd_valid),
    .io_input_cmd_ready(_zz_45_),
    .io_input_cmd_payload_wr(system_iBus_cmd_payload_wr),
    .io_input_cmd_payload_address(system_iBus_cmd_payload_address),
    .io_input_cmd_payload_data(system_iBus_cmd_payload_data),
    .io_input_cmd_payload_mask(system_iBus_cmd_payload_mask),
    .io_input_rsp_valid(_zz_46_),
    .io_input_rsp_payload_data(_zz_47_),
    .io_outputs_0_cmd_valid(_zz_48_),
    .io_outputs_0_cmd_ready(_zz_98_),
    .io_outputs_0_cmd_payload_wr(_zz_49_),
    .io_outputs_0_cmd_payload_address(_zz_50_),
    .io_outputs_0_cmd_payload_data(_zz_51_),
    .io_outputs_0_cmd_payload_mask(_zz_52_),
    .io_outputs_0_rsp_valid(_zz_99_),
    .io_outputs_0_rsp_payload_data(_zz_100_) 
  );
  SimpleBusDecoder_2_ system_mainBus_decoder ( 
    .io_input_cmd_valid(system_mainBus_cmd_s2mPipe_valid),
    .io_input_cmd_ready(_zz_53_),
    .io_input_cmd_payload_wr(system_mainBus_cmd_s2mPipe_payload_wr),
    .io_input_cmd_payload_address(system_mainBus_cmd_s2mPipe_payload_address),
    .io_input_cmd_payload_data(system_mainBus_cmd_s2mPipe_payload_data),
    .io_input_cmd_payload_mask(system_mainBus_cmd_s2mPipe_payload_mask),
    .io_input_rsp_valid(_zz_54_),
    .io_input_rsp_payload_data(_zz_55_),
    .io_outputs_0_cmd_valid(_zz_56_),
    .io_outputs_0_cmd_ready(_zz_71_),
    .io_outputs_0_cmd_payload_wr(_zz_57_),
    .io_outputs_0_cmd_payload_address(_zz_58_),
    .io_outputs_0_cmd_payload_data(_zz_59_),
    .io_outputs_0_cmd_payload_mask(_zz_60_),
    .io_outputs_0_rsp_valid(_zz_72_),
    .io_outputs_0_rsp_0_data(_zz_73_),
    .io_outputs_1_cmd_valid(_zz_61_),
    .io_outputs_1_cmd_ready(_zz_79_),
    .io_outputs_1_cmd_payload_wr(_zz_62_),
    .io_outputs_1_cmd_payload_address(_zz_63_),
    .io_outputs_1_cmd_payload_data(_zz_64_),
    .io_outputs_1_cmd_payload_mask(_zz_65_),
    .io_outputs_1_rsp_valid(_zz_80_),
    .io_outputs_1_rsp_1_data(_zz_81_),
    .io_outputs_2_cmd_valid(_zz_66_),
    .io_outputs_2_cmd_ready(_zz_87_),
    .io_outputs_2_cmd_payload_wr(_zz_67_),
    .io_outputs_2_cmd_payload_address(_zz_68_),
    .io_outputs_2_cmd_payload_data(_zz_69_),
    .io_outputs_2_cmd_payload_mask(_zz_70_),
    .io_outputs_2_rsp_valid(_zz_88_),
    .io_outputs_2_rsp_2_data(_zz_89_),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  SimpleBusArbiter system_ram_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_56_),
    .io_inputs_0_cmd_ready(_zz_71_),
    .io_inputs_0_cmd_payload_wr(_zz_57_),
    .io_inputs_0_cmd_payload_address(_zz_10_),
    .io_inputs_0_cmd_payload_data(_zz_59_),
    .io_inputs_0_cmd_payload_mask(_zz_60_),
    .io_inputs_0_rsp_valid(_zz_72_),
    .io_inputs_0_rsp_payload_data(_zz_73_),
    .io_output_cmd_valid(_zz_74_),
    .io_output_cmd_ready(_zz_15_),
    .io_output_cmd_payload_wr(_zz_75_),
    .io_output_cmd_payload_address(_zz_76_),
    .io_output_cmd_payload_data(_zz_77_),
    .io_output_cmd_payload_mask(_zz_78_),
    .io_output_rsp_valid(_zz_16_),
    .io_output_rsp_payload_data(_zz_17_) 
  );
  SimpleBusArbiter_1_ system_peripherals_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_61_),
    .io_inputs_0_cmd_ready(_zz_79_),
    .io_inputs_0_cmd_payload_wr(_zz_62_),
    .io_inputs_0_cmd_payload_address(_zz_11_),
    .io_inputs_0_cmd_payload_data(_zz_64_),
    .io_inputs_0_cmd_payload_mask(_zz_65_),
    .io_inputs_0_rsp_valid(_zz_80_),
    .io_inputs_0_rsp_payload_data(_zz_81_),
    .io_output_cmd_valid(_zz_82_),
    .io_output_cmd_ready(_zz_18_),
    .io_output_cmd_payload_wr(_zz_83_),
    .io_output_cmd_payload_address(_zz_84_),
    .io_output_cmd_payload_data(_zz_85_),
    .io_output_cmd_payload_mask(_zz_86_),
    .io_output_rsp_valid(_zz_19_),
    .io_output_rsp_payload_data(_zz_20_) 
  );
  SimpleBusArbiter_2_ system_flashXip_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_66_),
    .io_inputs_0_cmd_ready(_zz_87_),
    .io_inputs_0_cmd_payload_wr(_zz_67_),
    .io_inputs_0_cmd_payload_address(_zz_12_),
    .io_inputs_0_cmd_payload_data(_zz_69_),
    .io_inputs_0_cmd_payload_mask(_zz_70_),
    .io_inputs_0_rsp_valid(_zz_88_),
    .io_inputs_0_rsp_payload_data(_zz_89_),
    .io_output_cmd_valid(_zz_90_),
    .io_output_cmd_ready(_zz_24_),
    .io_output_cmd_payload_wr(_zz_91_),
    .io_output_cmd_payload_address(_zz_92_),
    .io_output_cmd_payload_data(_zz_93_),
    .io_output_cmd_payload_mask(_zz_94_),
    .io_output_rsp_valid(_zz_25_),
    .io_output_rsp_payload_data(_zz_26_) 
  );
  SimpleBusArbiter_3_ system_mainBus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_40_),
    .io_inputs_0_cmd_ready(_zz_95_),
    .io_inputs_0_cmd_payload_wr(_zz_41_),
    .io_inputs_0_cmd_payload_address(_zz_42_),
    .io_inputs_0_cmd_payload_data(_zz_43_),
    .io_inputs_0_cmd_payload_mask(_zz_44_),
    .io_inputs_0_rsp_valid(_zz_96_),
    .io_inputs_0_rsp_payload_data(_zz_97_),
    .io_inputs_1_cmd_valid(_zz_48_),
    .io_inputs_1_cmd_ready(_zz_98_),
    .io_inputs_1_cmd_payload_wr(_zz_49_),
    .io_inputs_1_cmd_payload_address(_zz_50_),
    .io_inputs_1_cmd_payload_data(_zz_51_),
    .io_inputs_1_cmd_payload_mask(_zz_52_),
    .io_inputs_1_rsp_valid(_zz_99_),
    .io_inputs_1_rsp_payload_data(_zz_100_),
    .io_output_cmd_valid(_zz_101_),
    .io_output_cmd_ready(system_mainBus_cmd_ready),
    .io_output_cmd_payload_wr(_zz_102_),
    .io_output_cmd_payload_address(_zz_103_),
    .io_output_cmd_payload_data(_zz_104_),
    .io_output_cmd_payload_mask(_zz_105_),
    .io_output_rsp_valid(system_mainBus_rsp_valid),
    .io_output_rsp_payload_data(system_mainBus_rsp_payload_data),
    .io_clk(io_clk),
    .GLOBAL_BUFFER_OUTPUT(_zz_14_) 
  );
  always @ (*) begin
    resetCtrl_resetUnbuffered = 1'b0;
    if(_zz_106_)begin
      resetCtrl_resetUnbuffered = 1'b1;
    end
  end

  assign io_serialTx = _zz_23_;
  assign io_leds = _zz_22_;
  assign io_flash_ss = io_flash_ss_regNext;
  assign io_flash_sclk = io_flash_sclk_regNext;
  assign io_flash_mosi = io_flash_mosi_regNext;
  assign _zz_7_ = 1'b0;
  assign system_iBus_cmd_valid = _zz_30_;
  assign system_iBus_cmd_payload_wr = 1'b0;
  assign system_iBus_cmd_payload_address = _zz_31_[19:0];
  assign system_iBus_cmd_payload_data = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
  assign system_iBus_cmd_payload_mask = (4'bxxxx);
  always @ (*) begin
    case(_zz_36_)
      2'b00 : begin
        _zz_1_ = (4'b0001);
      end
      2'b01 : begin
        _zz_1_ = (4'b0011);
      end
      default : begin
        _zz_1_ = (4'b1111);
      end
    endcase
  end

  assign system_dBus_cmd_valid = _zz_32_;
  assign system_dBus_cmd_payload_wr = _zz_33_;
  assign system_dBus_cmd_payload_address = _zz_34_[19:0];
  assign system_dBus_cmd_payload_data = _zz_35_;
  assign system_dBus_cmd_payload_mask = (_zz_1_ <<< _zz_34_[1 : 0]);
  assign _zz_8_ = 1'b0;
  assign system_dBus_cmd_ready = _zz_37_;
  assign system_dBus_rsp_valid = _zz_38_;
  assign system_dBus_rsp_payload_data = _zz_39_;
  assign system_iBus_cmd_ready = _zz_45_;
  assign system_iBus_rsp_valid = _zz_46_;
  assign system_iBus_rsp_payload_data = _zz_47_;
  assign system_mainBus_cmd_s2mPipe_valid = (system_mainBus_cmd_valid || _zz_2_);
  assign system_mainBus_cmd_ready = (! _zz_2_);
  assign system_mainBus_cmd_s2mPipe_payload_wr = (_zz_2_ ? _zz_3_ : system_mainBus_cmd_payload_wr);
  assign system_mainBus_cmd_s2mPipe_payload_address = (_zz_2_ ? _zz_4_ : system_mainBus_cmd_payload_address);
  assign system_mainBus_cmd_s2mPipe_payload_data = (_zz_2_ ? _zz_5_ : system_mainBus_cmd_payload_data);
  assign system_mainBus_cmd_s2mPipe_payload_mask = (_zz_2_ ? _zz_6_ : system_mainBus_cmd_payload_mask);
  assign system_mainBus_cmd_s2mPipe_ready = _zz_53_;
  assign system_mainBus_rsp_valid = _zz_54_;
  assign system_mainBus_rsp_payload_data = _zz_55_;
  assign system_mainBus_cmd_valid = _zz_101_;
  assign system_mainBus_cmd_payload_wr = _zz_102_;
  assign system_mainBus_cmd_payload_address = _zz_103_;
  assign system_mainBus_cmd_payload_data = _zz_104_;
  assign system_mainBus_cmd_payload_mask = _zz_105_;
  assign _zz_10_ = _zz_58_[15:0];
  assign _zz_11_ = _zz_63_[5:0];
  assign _zz_12_ = _zz_68_[18:0];
  always @ (posedge io_clk) begin
    if(_zz_106_)begin
      resetCtrl_resetCounter <= (resetCtrl_resetCounter + (6'b000001));
    end
    if(_zz_13_)begin
      resetCtrl_resetCounter <= (6'b000000);
    end
  end

  always @ (posedge io_clk) begin
    resetCtrl_systemResetBuffered <= resetCtrl_resetUnbuffered;
  end

  always @ (posedge io_clk) begin
    if(_zz_14_) begin
      io_flash_ss_regNext <= (1'b1);
      io_flash_sclk_regNext <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      io_flash_ss_regNext <= _zz_29_;
      io_flash_sclk_regNext <= _zz_27_;
      if(system_mainBus_cmd_s2mPipe_ready)begin
        _zz_2_ <= 1'b0;
      end
      if(_zz_107_)begin
        _zz_2_ <= system_mainBus_cmd_valid;
      end
    end
  end

  always @ (posedge io_clk) begin
    io_flash_mosi_regNext <= _zz_28_;
    if(_zz_107_)begin
      _zz_3_ <= system_mainBus_cmd_payload_wr;
      _zz_4_ <= system_mainBus_cmd_payload_address;
      _zz_5_ <= system_mainBus_cmd_payload_data;
      _zz_6_ <= system_mainBus_cmd_payload_mask;
    end
  end

endmodule

module Up5kAreaEvn (
      input   io_iceClk,
      output  io_serialTx,
      output [0:0] io_flashSpi_ss,
      output  io_flashSpi_sclk,
      output  io_flashSpi_mosi,
      input   io_flashSpi_miso,
      output  io_leds_r,
      output  io_leds_g,
      output  io_leds_b);
  wire  _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire [2:0] _zz_8_;
  wire  _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire [0:0] _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  SB_GB clkBuffer ( 
    .USER_SIGNAL_TO_GLOBAL_BUFFER(io_iceClk),
    .GLOBAL_BUFFER_OUTPUT(_zz_7_) 
  );
  Up5kArea soc ( 
    .io_clk(_zz_7_),
    .io_reset(_zz_1_),
    .io_leds(_zz_8_),
    .io_serialTx(_zz_9_),
    .io_flash_ss(_zz_12_),
    .io_flash_sclk(_zz_10_),
    .io_flash_mosi(_zz_11_),
    .io_flash_miso(io_flashSpi_miso) 
  );
  SB_RGBA_DRV #( 
    .CURRENT_MODE("0b1"),
    .RGB0_CURRENT ("0b000001"),
    .RGB1_CURRENT ("0b000001"),
    .RGB2_CURRENT ("0b000001") 
  ) ledDriver ( 
    .CURREN(_zz_2_),
    .RGBLEDEN(_zz_3_),
    .RGB0PWM(_zz_4_),
    .RGB1PWM(_zz_5_),
    .RGB2PWM(_zz_6_),
    .RGB0(_zz_13_),
    .RGB1(_zz_14_),
    .RGB2(_zz_15_) 
  );
  assign _zz_1_ = 1'b0;
  assign io_flashSpi_ss = _zz_12_;
  assign io_flashSpi_sclk = _zz_10_;
  assign io_flashSpi_mosi = _zz_11_;
  assign io_serialTx = _zz_9_;
  assign _zz_2_ = 1'b1;
  assign _zz_3_ = 1'b1;
  assign _zz_4_ = (! io_serialTx);
  assign _zz_5_ = _zz_8_[0];
  assign _zz_6_ = _zz_8_[1];
  assign io_leds_b = _zz_13_;
  assign io_leds_g = _zz_14_;
  assign io_leds_r = _zz_15_;
endmodule

