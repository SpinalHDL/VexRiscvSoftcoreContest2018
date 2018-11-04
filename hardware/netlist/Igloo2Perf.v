// Generator : SpinalHDL v1.2.2    git head : ec5e7e191d669ded826c34b7aaa74f2563574157
// Date      : 03/11/2018, 18:40:19
// Component : Igloo2Perf


`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

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

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10
`define AluBitwiseCtrlEnum_defaultEncoding_SRC1 2'b11

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

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
      output [1:0] io_occupancy,
      input   io_clk,
      input   resetCtrl_systemReset);
  wire [32:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [32:0] _zz_5_;
  reg  _zz_1_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  reg [0:0] pushPtr_valueNext;
  reg [0:0] pushPtr_value;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  reg [0:0] popPtr_valueNext;
  reg [0:0] popPtr_value;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [32:0] _zz_2_;
  wire [0:0] ptrDif;
  reg [32:0] ram [0:1];
  assign _zz_4_ = _zz_2_[0 : 0];
  assign _zz_5_ = {io_push_payload_inst,io_push_payload_error};
  always @ (posedge io_clk) begin
    if(_zz_1_) begin
      ram[pushPtr_value] <= _zz_5_;
    end
  end

  assign _zz_3_ = ram[popPtr_value];
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

  assign pushPtr_willOverflowIfInc = (pushPtr_value == (1'b1));
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    pushPtr_valueNext = (pushPtr_value + pushPtr_willIncrement);
    if(pushPtr_willClear)begin
      pushPtr_valueNext = (1'b0);
    end
  end

  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = (popPtr_value == (1'b1));
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  always @ (*) begin
    popPtr_valueNext = (popPtr_value + popPtr_willIncrement);
    if(popPtr_willClear)begin
      popPtr_valueNext = (1'b0);
    end
  end

  assign ptrMatch = (pushPtr_value == popPtr_value);
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
      io_pop_payload_error = _zz_4_[0];
      io_pop_payload_inst = _zz_2_[32 : 1];
    end else begin
      io_pop_valid = io_push_valid;
      io_pop_payload_error = io_push_payload_error;
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign _zz_2_ = _zz_3_;
  assign ptrDif = (pushPtr_value - popPtr_value);
  assign io_occupancy = {(risingOccupancy && ptrMatch),ptrDif};
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      pushPtr_value <= (1'b0);
      popPtr_value <= (1'b0);
      risingOccupancy <= 1'b0;
    end else begin
      pushPtr_value <= pushPtr_valueNext;
      popPtr_value <= popPtr_valueNext;
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module BufferCC (
      input   io_dataIn,
      output  io_dataOut,
      input   io_clk,
      input   resetCtrl_progReset);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge io_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module StreamArbiter (
      input   io_inputs_0_0,
      output  io_inputs_0_ready,
      input   io_inputs_0_0_wr,
      input  [14:0] io_inputs_0_0_address,
      input  [31:0] io_inputs_0_0_data,
      input  [3:0] io_inputs_0_0_mask,
      input   io_inputs_1_1,
      output  io_inputs_1_ready,
      input   io_inputs_1_1_wr,
      input  [14:0] io_inputs_1_1_address,
      input  [31:0] io_inputs_1_1_data,
      input  [3:0] io_inputs_1_1_mask,
      output  io_output_valid,
      input   io_output_ready,
      output  io_output_payload_wr,
      output [14:0] io_output_payload_address,
      output [31:0] io_output_payload_data,
      output [3:0] io_output_payload_mask,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg  _zz_4_;
  reg [14:0] _zz_5_;
  reg [31:0] _zz_6_;
  reg [3:0] _zz_7_;
  wire [1:0] _zz_8_;
  wire [1:0] _zz_9_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_1_;
  wire [0:0] _zz_2_;
  wire  _zz_3_;
  assign _zz_8_ = (_zz_1_ & (~ _zz_9_));
  assign _zz_9_ = (_zz_1_ - (2'b01));
  always @(*) begin
    case(_zz_2_)
      1'b0 : begin
        _zz_4_ = io_inputs_0_0_wr;
        _zz_5_ = io_inputs_0_0_address;
        _zz_6_ = io_inputs_0_0_data;
        _zz_7_ = io_inputs_0_0_mask;
      end
      default : begin
        _zz_4_ = io_inputs_1_1_wr;
        _zz_5_ = io_inputs_1_1_address;
        _zz_6_ = io_inputs_1_1_data;
        _zz_7_ = io_inputs_1_1_mask;
      end
    endcase
  end

  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_1_ = {io_inputs_1_1,io_inputs_0_0};
  assign maskProposal_0 = io_inputs_0_0;
  assign maskProposal_1 = _zz_8_[1];
  assign io_output_valid = ((io_inputs_0_0 && maskRouted_0) || (io_inputs_1_1 && maskRouted_1));
  assign _zz_2_ = maskRouted_1;
  assign io_output_payload_wr = _zz_4_;
  assign io_output_payload_address = _zz_5_;
  assign io_output_payload_data = _zz_6_;
  assign io_output_payload_mask = _zz_7_;
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_3_ = io_chosenOH[1];
  assign io_chosen = _zz_3_;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
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

module StreamFork (
      input   io_input_valid,
      output reg  io_input_ready,
      input   io_input_payload_wr,
      input  [14:0] io_input_payload_address,
      input  [31:0] io_input_payload_data,
      input  [3:0] io_input_payload_mask,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output  io_outputs_0_payload_wr,
      output [14:0] io_outputs_0_payload_address,
      output [31:0] io_outputs_0_payload_data,
      output [3:0] io_outputs_0_payload_mask,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output  io_outputs_1_payload_wr,
      output [14:0] io_outputs_1_payload_address,
      output [31:0] io_outputs_1_payload_data,
      output [3:0] io_outputs_1_payload_mask,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_wr = io_input_payload_wr;
  assign io_outputs_0_payload_address = io_input_payload_address;
  assign io_outputs_0_payload_data = io_input_payload_data;
  assign io_outputs_0_payload_mask = io_input_payload_mask;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_wr = io_input_payload_wr;
  assign io_outputs_1_payload_address = io_input_payload_address;
  assign io_outputs_1_payload_data = io_input_payload_data;
  assign io_outputs_1_payload_mask = io_input_payload_mask;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule

module StreamFifoLowLatency_1_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [0:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [0:0] io_pop_payload,
      input   io_flush,
      output [1:0] io_occupancy,
      input   io_clk,
      input   resetCtrl_systemReset);
  wire [0:0] _zz_2_;
  wire [0:0] _zz_3_;
  reg  _zz_1_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  reg [0:0] pushPtr_valueNext;
  reg [0:0] pushPtr_value;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  reg [0:0] popPtr_valueNext;
  reg [0:0] popPtr_value;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [0:0] ptrDif;
  reg [0:0] ram [0:1];
  assign _zz_3_ = io_push_payload;
  always @ (posedge io_clk) begin
    if(_zz_1_) begin
      ram[pushPtr_value] <= _zz_3_;
    end
  end

  assign _zz_2_ = ram[popPtr_value];
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

  assign pushPtr_willOverflowIfInc = (pushPtr_value == (1'b1));
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    pushPtr_valueNext = (pushPtr_value + pushPtr_willIncrement);
    if(pushPtr_willClear)begin
      pushPtr_valueNext = (1'b0);
    end
  end

  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = (popPtr_value == (1'b1));
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  always @ (*) begin
    popPtr_valueNext = (popPtr_value + popPtr_willIncrement);
    if(popPtr_willClear)begin
      popPtr_valueNext = (1'b0);
    end
  end

  assign ptrMatch = (pushPtr_value == popPtr_value);
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  assign io_pop_valid = (! empty);
  assign io_pop_payload = _zz_2_;
  assign ptrDif = (pushPtr_value - popPtr_value);
  assign io_occupancy = {(risingOccupancy && ptrMatch),ptrDif};
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      pushPtr_value <= (1'b0);
      popPtr_value <= (1'b0);
      risingOccupancy <= 1'b0;
    end else begin
      pushPtr_value <= pushPtr_valueNext;
      popPtr_value <= popPtr_valueNext;
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamArbiter_1_ (
      input   io_inputs_0_0,
      output  io_inputs_0_ready,
      input   io_inputs_0_0_wr,
      input  [13:0] io_inputs_0_0_address,
      input  [31:0] io_inputs_0_0_data,
      input  [3:0] io_inputs_0_0_mask,
      input   io_inputs_1_1,
      output  io_inputs_1_ready,
      input   io_inputs_1_1_wr,
      input  [13:0] io_inputs_1_1_address,
      input  [31:0] io_inputs_1_1_data,
      input  [3:0] io_inputs_1_1_mask,
      output  io_output_valid,
      input   io_output_ready,
      output  io_output_payload_wr,
      output [13:0] io_output_payload_address,
      output [31:0] io_output_payload_data,
      output [3:0] io_output_payload_mask,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg  _zz_4_;
  reg [13:0] _zz_5_;
  reg [31:0] _zz_6_;
  reg [3:0] _zz_7_;
  wire [1:0] _zz_8_;
  wire [1:0] _zz_9_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_1_;
  wire [0:0] _zz_2_;
  wire  _zz_3_;
  assign _zz_8_ = (_zz_1_ & (~ _zz_9_));
  assign _zz_9_ = (_zz_1_ - (2'b01));
  always @(*) begin
    case(_zz_2_)
      1'b0 : begin
        _zz_4_ = io_inputs_0_0_wr;
        _zz_5_ = io_inputs_0_0_address;
        _zz_6_ = io_inputs_0_0_data;
        _zz_7_ = io_inputs_0_0_mask;
      end
      default : begin
        _zz_4_ = io_inputs_1_1_wr;
        _zz_5_ = io_inputs_1_1_address;
        _zz_6_ = io_inputs_1_1_data;
        _zz_7_ = io_inputs_1_1_mask;
      end
    endcase
  end

  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_1_ = {io_inputs_1_1,io_inputs_0_0};
  assign maskProposal_0 = io_inputs_0_0;
  assign maskProposal_1 = _zz_8_[1];
  assign io_output_valid = ((io_inputs_0_0 && maskRouted_0) || (io_inputs_1_1 && maskRouted_1));
  assign _zz_2_ = maskRouted_1;
  assign io_output_payload_wr = _zz_4_;
  assign io_output_payload_address = _zz_5_;
  assign io_output_payload_data = _zz_6_;
  assign io_output_payload_mask = _zz_7_;
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_3_ = io_chosenOH[1];
  assign io_chosen = _zz_3_;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
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

module StreamFork_1_ (
      input   io_input_valid,
      output reg  io_input_ready,
      input   io_input_payload_wr,
      input  [13:0] io_input_payload_address,
      input  [31:0] io_input_payload_data,
      input  [3:0] io_input_payload_mask,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output  io_outputs_0_payload_wr,
      output [13:0] io_outputs_0_payload_address,
      output [31:0] io_outputs_0_payload_data,
      output [3:0] io_outputs_0_payload_mask,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output  io_outputs_1_payload_wr,
      output [13:0] io_outputs_1_payload_address,
      output [31:0] io_outputs_1_payload_data,
      output [3:0] io_outputs_1_payload_mask,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_wr = io_input_payload_wr;
  assign io_outputs_0_payload_address = io_input_payload_address;
  assign io_outputs_0_payload_data = io_input_payload_data;
  assign io_outputs_0_payload_mask = io_input_payload_mask;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_wr = io_input_payload_wr;
  assign io_outputs_1_payload_address = io_input_payload_address;
  assign io_outputs_1_payload_data = io_input_payload_data;
  assign io_outputs_1_payload_mask = io_input_payload_mask;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule


//StreamFifoLowLatency_2_ remplaced by StreamFifoLowLatency_1_

module StreamArbiter_2_ (
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
      input   resetCtrl_systemReset);
  reg  _zz_4_;
  reg [19:0] _zz_5_;
  reg [31:0] _zz_6_;
  reg [3:0] _zz_7_;
  wire [1:0] _zz_8_;
  wire [1:0] _zz_9_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_1_;
  wire [0:0] _zz_2_;
  wire  _zz_3_;
  assign _zz_8_ = (_zz_1_ & (~ _zz_9_));
  assign _zz_9_ = (_zz_1_ - (2'b01));
  always @(*) begin
    case(_zz_2_)
      1'b0 : begin
        _zz_4_ = io_inputs_0_0_wr;
        _zz_5_ = io_inputs_0_0_address;
        _zz_6_ = io_inputs_0_0_data;
        _zz_7_ = io_inputs_0_0_mask;
      end
      default : begin
        _zz_4_ = io_inputs_1_1_wr;
        _zz_5_ = io_inputs_1_1_address;
        _zz_6_ = io_inputs_1_1_data;
        _zz_7_ = io_inputs_1_1_mask;
      end
    endcase
  end

  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_1_ = {io_inputs_1_1,io_inputs_0_0};
  assign maskProposal_0 = io_inputs_0_0;
  assign maskProposal_1 = _zz_8_[1];
  assign io_output_valid = ((io_inputs_0_0 && maskRouted_0) || (io_inputs_1_1 && maskRouted_1));
  assign _zz_2_ = maskRouted_1;
  assign io_output_payload_wr = _zz_4_;
  assign io_output_payload_address = _zz_5_;
  assign io_output_payload_data = _zz_6_;
  assign io_output_payload_mask = _zz_7_;
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_3_ = io_chosenOH[1];
  assign io_chosen = _zz_3_;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
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

module StreamFork_2_ (
      input   io_input_valid,
      output reg  io_input_ready,
      input   io_input_payload_wr,
      input  [19:0] io_input_payload_address,
      input  [31:0] io_input_payload_data,
      input  [3:0] io_input_payload_mask,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output  io_outputs_0_payload_wr,
      output [19:0] io_outputs_0_payload_address,
      output [31:0] io_outputs_0_payload_data,
      output [3:0] io_outputs_0_payload_mask,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output  io_outputs_1_payload_wr,
      output [19:0] io_outputs_1_payload_address,
      output [31:0] io_outputs_1_payload_data,
      output [3:0] io_outputs_1_payload_mask,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_wr = io_input_payload_wr;
  assign io_outputs_0_payload_address = io_input_payload_address;
  assign io_outputs_0_payload_data = io_input_payload_data;
  assign io_outputs_0_payload_mask = io_input_payload_mask;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_wr = io_input_payload_wr;
  assign io_outputs_1_payload_address = io_input_payload_address;
  assign io_outputs_1_payload_data = io_input_payload_data;
  assign io_outputs_1_payload_mask = io_input_payload_mask;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule


//StreamFifoLowLatency_3_ remplaced by StreamFifoLowLatency_1_

module BufferCC_1_ (
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

module SimpleBusRam (
      input   io_bus_cmd_valid,
      output  io_bus_cmd_ready,
      input   io_bus_cmd_payload_wr,
      input  [14:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_payload_data,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg [31:0] _zz_4_;
  reg  _zz_1_;
  wire [12:0] _zz_2_;
  wire [31:0] _zz_3_;
  reg [7:0] ram_symbol0 [0:5119] /* verilator public */ ;
  reg [7:0] ram_symbol1 [0:5119] /* verilator public */ ;
  reg [7:0] ram_symbol2 [0:5119] /* verilator public */ ;
  reg [7:0] ram_symbol3 [0:5119] /* verilator public */ ;
  reg [7:0] _zz_5_;
  reg [7:0] _zz_6_;
  reg [7:0] _zz_7_;
  reg [7:0] _zz_8_;
  always @ (*) begin
    _zz_4_ = {_zz_8_, _zz_7_, _zz_6_, _zz_5_};
  end
  always @ (posedge io_clk) begin
    if(io_bus_cmd_payload_mask[0] && io_bus_cmd_valid && io_bus_cmd_payload_wr ) begin
      ram_symbol0[_zz_2_] <= _zz_3_[7 : 0];
    end
    if(io_bus_cmd_payload_mask[1] && io_bus_cmd_valid && io_bus_cmd_payload_wr ) begin
      ram_symbol1[_zz_2_] <= _zz_3_[15 : 8];
    end
    if(io_bus_cmd_payload_mask[2] && io_bus_cmd_valid && io_bus_cmd_payload_wr ) begin
      ram_symbol2[_zz_2_] <= _zz_3_[23 : 16];
    end
    if(io_bus_cmd_payload_mask[3] && io_bus_cmd_valid && io_bus_cmd_payload_wr ) begin
      ram_symbol3[_zz_2_] <= _zz_3_[31 : 24];
    end
    if(io_bus_cmd_valid) begin
      _zz_5_ <= ram_symbol0[_zz_2_];
      _zz_6_ <= ram_symbol1[_zz_2_];
      _zz_7_ <= ram_symbol2[_zz_2_];
      _zz_8_ <= ram_symbol3[_zz_2_];
    end
  end

  assign io_bus_cmd_ready = 1'b1;
  assign io_bus_rsp_valid = _zz_1_;
  assign _zz_2_ = (io_bus_cmd_payload_address >>> 2);
  assign _zz_3_ = io_bus_cmd_payload_data;
  assign io_bus_rsp_payload_data = _zz_4_;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      _zz_1_ <= 1'b0;
    end else begin
      _zz_1_ <= ((io_bus_cmd_valid && io_bus_cmd_ready) && (! io_bus_cmd_payload_wr));
    end
  end

endmodule

module SimpleBusRam_1_ (
      input   io_bus_cmd_valid,
      output  io_bus_cmd_ready,
      input   io_bus_cmd_payload_wr,
      input  [13:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_payload_data,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg [31:0] _zz_4_;
  reg  _zz_1_;
  reg  readLogic_cmd_valid;
  reg  readLogic_cmd_payload_wr;
  reg [13:0] readLogic_cmd_payload_address;
  reg [31:0] readLogic_cmd_payload_data;
  reg [3:0] readLogic_cmd_payload_mask;
  wire [11:0] _zz_2_;
  wire [31:0] _zz_3_;
  reg [7:0] ram_symbol0 [0:4095] /* verilator public */ ;
  reg [7:0] ram_symbol1 [0:4095] /* verilator public */ ;
  reg [7:0] ram_symbol2 [0:4095] /* verilator public */ ;
  reg [7:0] ram_symbol3 [0:4095] /* verilator public */ ;
  reg [7:0] _zz_5_;
  reg [7:0] _zz_6_;
  reg [7:0] _zz_7_;
  reg [7:0] _zz_8_;
  always @ (*) begin
    _zz_4_ = {_zz_8_, _zz_7_, _zz_6_, _zz_5_};
  end
  always @ (negedge io_clk) begin
    if(readLogic_cmd_payload_mask[0] && readLogic_cmd_valid && readLogic_cmd_payload_wr ) begin
      ram_symbol0[_zz_2_] <= _zz_3_[7 : 0];
    end
    if(readLogic_cmd_payload_mask[1] && readLogic_cmd_valid && readLogic_cmd_payload_wr ) begin
      ram_symbol1[_zz_2_] <= _zz_3_[15 : 8];
    end
    if(readLogic_cmd_payload_mask[2] && readLogic_cmd_valid && readLogic_cmd_payload_wr ) begin
      ram_symbol2[_zz_2_] <= _zz_3_[23 : 16];
    end
    if(readLogic_cmd_payload_mask[3] && readLogic_cmd_valid && readLogic_cmd_payload_wr ) begin
      ram_symbol3[_zz_2_] <= _zz_3_[31 : 24];
    end
    if(readLogic_cmd_valid) begin
      _zz_5_ <= ram_symbol0[_zz_2_];
      _zz_6_ <= ram_symbol1[_zz_2_];
      _zz_7_ <= ram_symbol2[_zz_2_];
      _zz_8_ <= ram_symbol3[_zz_2_];
    end
  end

  assign io_bus_cmd_ready = 1'b1;
  assign io_bus_rsp_valid = _zz_1_;
  assign _zz_2_ = (readLogic_cmd_payload_address >>> 2);
  assign _zz_3_ = readLogic_cmd_payload_data;
  assign io_bus_rsp_payload_data = _zz_4_;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      _zz_1_ <= 1'b0;
    end else begin
      _zz_1_ <= ((io_bus_cmd_valid && io_bus_cmd_ready) && (! io_bus_cmd_payload_wr));
    end
  end

  always @ (posedge io_clk) begin
    readLogic_cmd_valid <= io_bus_cmd_valid;
    readLogic_cmd_payload_wr <= io_bus_cmd_payload_wr;
    readLogic_cmd_payload_address <= io_bus_cmd_payload_address;
    readLogic_cmd_payload_data <= io_bus_cmd_payload_data;
    readLogic_cmd_payload_mask <= io_bus_cmd_payload_mask;
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
      input   resetCtrl_systemReset);
  wire [31:0] _zz_5_;
  wire [0:0] _zz_6_;
  wire [3:0] _zz_7_;
  wire [0:0] _zz_8_;
  wire [7:0] _zz_9_;
  wire  mapper_readAtCmd_valid;
  reg [31:0] mapper_readAtCmd_payload;
  reg  mapper_readAtRsp_valid;
  reg [31:0] mapper_readAtRsp_payload;
  wire  mapper_askWrite;
  wire  mapper_askRead;
  wire  mapper_doWrite;
  wire  mapper_doRead;
  reg [2:0] _zz_1_;
  reg [31:0] mTime_counter;
  reg [31:0] mTime_cmp;
  reg  mTime_interrupt;
  reg  _zz_2_;
  reg  serialTx_counter_willIncrement;
  wire  serialTx_counter_willClear;
  reg [3:0] serialTx_counter_valueNext;
  reg [3:0] serialTx_counter_value;
  wire  serialTx_counter_willOverflowIfInc;
  wire  serialTx_counter_willOverflow;
  reg [7:0] serialTx_buffer;
  wire [11:0] serialTx_bitstream;
  wire  serialTx_busy;
  reg  _zz_3_;
  wire  serialTx_timer_willIncrement;
  wire  serialTx_timer_willClear;
  reg [7:0] serialTx_timer_valueNext;
  reg [7:0] serialTx_timer_value;
  wire  serialTx_timer_willOverflowIfInc;
  wire  serialTx_timer_willOverflow;
  reg  _zz_4_;
  assign _zz_5_ = (mTime_counter - mTime_cmp);
  assign _zz_6_ = serialTx_counter_willIncrement;
  assign _zz_7_ = {3'd0, _zz_6_};
  assign _zz_8_ = serialTx_timer_willIncrement;
  assign _zz_9_ = {7'd0, _zz_8_};
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
    _zz_2_ = 1'b0;
    _zz_4_ = 1'b0;
    case(io_bus_cmd_payload_address)
      6'b000100 : begin
        mapper_readAtCmd_payload[2 : 0] = _zz_1_;
      end
      6'b011000 : begin
        if(mapper_doWrite)begin
          _zz_2_ = 1'b1;
        end
      end
      6'b010000 : begin
        mapper_readAtCmd_payload[31 : 0] = mTime_counter;
      end
      6'b000000 : begin
        if(mapper_doWrite)begin
          _zz_4_ = 1'b1;
        end
        mapper_readAtCmd_payload[0 : 0] = serialTx_busy;
      end
      default : begin
      end
    endcase
  end

  assign io_leds = _zz_1_;
  assign io_mTimeInterrupt = mTime_interrupt;
  always @ (*) begin
    serialTx_counter_willIncrement = 1'b0;
    if(((serialTx_counter_value != (4'b0000)) && serialTx_timer_willOverflow))begin
      serialTx_counter_willIncrement = 1'b1;
    end
    if(_zz_4_)begin
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
  assign io_serialTx = _zz_3_;
  assign serialTx_timer_willClear = 1'b0;
  assign serialTx_timer_willOverflowIfInc = (serialTx_timer_value == (8'b11011000));
  assign serialTx_timer_willOverflow = (serialTx_timer_willOverflowIfInc && serialTx_timer_willIncrement);
  always @ (*) begin
    if(serialTx_timer_willOverflow)begin
      serialTx_timer_valueNext = (8'b00000000);
    end else begin
      serialTx_timer_valueNext = (serialTx_timer_value + _zz_9_);
    end
    if(serialTx_timer_willClear)begin
      serialTx_timer_valueNext = (8'b00000000);
    end
  end

  assign serialTx_timer_willIncrement = 1'b1;
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      mapper_readAtRsp_valid <= 1'b0;
      _zz_1_ <= (3'b000);
      mTime_counter <= (32'b00000000000000000000000000000000);
      mTime_cmp <= (32'b00000000000000000000000000000000);
      mTime_interrupt <= 1'b0;
      serialTx_counter_value <= (4'b0000);
      _zz_3_ <= 1'b1;
      serialTx_timer_value <= (8'b00000000);
    end else begin
      mapper_readAtRsp_valid <= mapper_readAtCmd_valid;
      if((! _zz_5_[31]))begin
        mTime_interrupt <= 1'b1;
      end
      if(_zz_2_)begin
        mTime_interrupt <= 1'b0;
      end
      mTime_counter <= (mTime_counter + (32'b00000000000000000000000000000001));
      serialTx_counter_value <= serialTx_counter_valueNext;
      _zz_3_ <= serialTx_bitstream[serialTx_counter_value];
      serialTx_timer_value <= serialTx_timer_valueNext;
      case(io_bus_cmd_payload_address)
        6'b000100 : begin
          if(mapper_doWrite)begin
            _zz_1_ <= io_bus_cmd_payload_data[2 : 0];
          end
        end
        6'b011000 : begin
          if(mapper_doWrite)begin
            mTime_cmp <= io_bus_cmd_payload_data[31 : 0];
          end
        end
        6'b010000 : begin
        end
        6'b000000 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_clk) begin
    mapper_readAtRsp_payload <= mapper_readAtCmd_payload;
    case(io_bus_cmd_payload_address)
      6'b000100 : begin
      end
      6'b011000 : begin
      end
      6'b010000 : begin
      end
      6'b000000 : begin
        if(mapper_doWrite)begin
          serialTx_buffer <= io_bus_cmd_payload_data[7 : 0];
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
      input  [19:0] io_bus_cmd_payload_address,
      input  [31:0] io_bus_cmd_payload_data,
      input  [3:0] io_bus_cmd_payload_mask,
      output  io_bus_rsp_valid,
      output [31:0] io_bus_rsp_payload_data,
      output reg [0:0] io_flash_ss,
      output reg  io_flash_sclk,
      output reg  io_flash_mosi,
      input   io_flash_miso,
      input   io_clk,
      input   resetCtrl_systemReset);
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
  reg [9:0] fsm_counter;
  reg `fsm_enumDefinition_defaultEncoding_type fsm_stateReg;
  reg `fsm_enumDefinition_defaultEncoding_type fsm_stateNext;
  wire [15:0] _zz_1_;
  wire [39:0] _zz_2_;
  assign _zz_5_ = buffer_counter_willIncrement;
  assign _zz_6_ = {4'd0, _zz_5_};
  assign _zz_7_ = (fsm_counter >>> 4);
  assign _zz_8_ = _zz_7_[3:0];
  assign _zz_9_ = {4'd0, io_bus_cmd_payload_address};
  assign _zz_10_ = (fsm_counter >>> 4);
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
        io_flash_sclk = fsm_counter[3];
        io_flash_mosi = _zz_3_;
        if((fsm_counter == (10'b0011111111)))begin
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
        io_flash_sclk = fsm_counter[3];
        io_flash_mosi = _zz_4_;
        if((fsm_counter == (10'b1001111111)))begin
          io_bus_cmd_ready = 1'b1;
          fsm_stateNext = `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD;
        end
      end
      `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD : begin
        io_flash_ss[0] = 1'b0;
        io_flash_sclk = fsm_counter[3];
        if((fsm_counter == (10'b0111111111)))begin
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
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
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
          if((fsm_counter[3 : 0] == (4'b1111)))begin
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
    fsm_counter <= (fsm_counter + (10'b0000000001));
    if(((! (fsm_stateReg == `fsm_enumDefinition_defaultEncoding_fsm_SETUP)) && (fsm_stateNext == `fsm_enumDefinition_defaultEncoding_fsm_SETUP)))begin
      fsm_counter <= (10'b0000000000);
    end
    if(((! (fsm_stateReg == `fsm_enumDefinition_defaultEncoding_fsm_CMD)) && (fsm_stateNext == `fsm_enumDefinition_defaultEncoding_fsm_CMD)))begin
      fsm_counter <= (10'b0000000000);
    end
    if(((! (fsm_stateReg == `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD)) && (fsm_stateNext == `fsm_enumDefinition_defaultEncoding_fsm_PAYLOAD)))begin
      fsm_counter <= (10'b0000000000);
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
      input   resetCtrl_systemReset);
  wire  _zz_178_;
  wire  _zz_179_;
  reg [31:0] _zz_180_;
  reg [31:0] _zz_181_;
  reg [31:0] _zz_182_;
  reg [3:0] _zz_183_;
  reg [31:0] _zz_184_;
  wire  _zz_185_;
  wire  _zz_186_;
  wire  _zz_187_;
  wire [31:0] _zz_188_;
  wire [1:0] _zz_189_;
  wire  _zz_190_;
  wire  _zz_191_;
  wire  _zz_192_;
  wire  _zz_193_;
  wire  _zz_194_;
  wire  _zz_195_;
  wire [1:0] _zz_196_;
  wire [1:0] _zz_197_;
  wire  _zz_198_;
  wire [1:0] _zz_199_;
  wire [1:0] _zz_200_;
  wire [1:0] _zz_201_;
  wire [1:0] _zz_202_;
  wire [2:0] _zz_203_;
  wire [31:0] _zz_204_;
  wire [2:0] _zz_205_;
  wire [0:0] _zz_206_;
  wire [2:0] _zz_207_;
  wire [0:0] _zz_208_;
  wire [2:0] _zz_209_;
  wire [0:0] _zz_210_;
  wire [2:0] _zz_211_;
  wire [2:0] _zz_212_;
  wire [0:0] _zz_213_;
  wire [2:0] _zz_214_;
  wire [0:0] _zz_215_;
  wire [2:0] _zz_216_;
  wire [1:0] _zz_217_;
  wire [1:0] _zz_218_;
  wire [2:0] _zz_219_;
  wire [3:0] _zz_220_;
  wire [0:0] _zz_221_;
  wire [0:0] _zz_222_;
  wire [0:0] _zz_223_;
  wire [0:0] _zz_224_;
  wire [0:0] _zz_225_;
  wire [0:0] _zz_226_;
  wire [0:0] _zz_227_;
  wire [0:0] _zz_228_;
  wire [0:0] _zz_229_;
  wire [0:0] _zz_230_;
  wire [0:0] _zz_231_;
  wire [0:0] _zz_232_;
  wire [0:0] _zz_233_;
  wire [51:0] _zz_234_;
  wire [51:0] _zz_235_;
  wire [51:0] _zz_236_;
  wire [32:0] _zz_237_;
  wire [51:0] _zz_238_;
  wire [49:0] _zz_239_;
  wire [51:0] _zz_240_;
  wire [49:0] _zz_241_;
  wire [51:0] _zz_242_;
  wire [65:0] _zz_243_;
  wire [65:0] _zz_244_;
  wire [31:0] _zz_245_;
  wire [31:0] _zz_246_;
  wire [0:0] _zz_247_;
  wire [5:0] _zz_248_;
  wire [32:0] _zz_249_;
  wire [32:0] _zz_250_;
  wire [31:0] _zz_251_;
  wire [31:0] _zz_252_;
  wire [32:0] _zz_253_;
  wire [32:0] _zz_254_;
  wire [32:0] _zz_255_;
  wire [0:0] _zz_256_;
  wire [32:0] _zz_257_;
  wire [0:0] _zz_258_;
  wire [32:0] _zz_259_;
  wire [0:0] _zz_260_;
  wire [31:0] _zz_261_;
  wire [0:0] _zz_262_;
  wire [2:0] _zz_263_;
  wire [4:0] _zz_264_;
  wire [11:0] _zz_265_;
  wire [11:0] _zz_266_;
  wire [31:0] _zz_267_;
  wire [31:0] _zz_268_;
  wire [32:0] _zz_269_;
  wire [31:0] _zz_270_;
  wire [32:0] _zz_271_;
  wire [19:0] _zz_272_;
  wire [11:0] _zz_273_;
  wire [11:0] _zz_274_;
  wire [0:0] _zz_275_;
  wire [0:0] _zz_276_;
  wire [0:0] _zz_277_;
  wire [0:0] _zz_278_;
  wire [0:0] _zz_279_;
  wire [0:0] _zz_280_;
  wire  _zz_281_;
  wire  _zz_282_;
  wire [0:0] _zz_283_;
  wire [31:0] _zz_284_;
  wire  _zz_285_;
  wire  _zz_286_;
  wire [0:0] _zz_287_;
  wire [0:0] _zz_288_;
  wire  _zz_289_;
  wire [0:0] _zz_290_;
  wire [21:0] _zz_291_;
  wire [31:0] _zz_292_;
  wire [31:0] _zz_293_;
  wire [31:0] _zz_294_;
  wire [31:0] _zz_295_;
  wire  _zz_296_;
  wire  _zz_297_;
  wire  _zz_298_;
  wire [4:0] _zz_299_;
  wire [4:0] _zz_300_;
  wire  _zz_301_;
  wire [0:0] _zz_302_;
  wire [17:0] _zz_303_;
  wire  _zz_304_;
  wire [0:0] _zz_305_;
  wire [1:0] _zz_306_;
  wire  _zz_307_;
  wire [0:0] _zz_308_;
  wire [0:0] _zz_309_;
  wire  _zz_310_;
  wire [0:0] _zz_311_;
  wire [14:0] _zz_312_;
  wire [31:0] _zz_313_;
  wire [31:0] _zz_314_;
  wire [31:0] _zz_315_;
  wire [31:0] _zz_316_;
  wire [31:0] _zz_317_;
  wire  _zz_318_;
  wire [0:0] _zz_319_;
  wire [2:0] _zz_320_;
  wire [0:0] _zz_321_;
  wire [0:0] _zz_322_;
  wire [2:0] _zz_323_;
  wire [2:0] _zz_324_;
  wire  _zz_325_;
  wire [0:0] _zz_326_;
  wire [11:0] _zz_327_;
  wire [31:0] _zz_328_;
  wire  _zz_329_;
  wire [0:0] _zz_330_;
  wire [0:0] _zz_331_;
  wire [31:0] _zz_332_;
  wire [31:0] _zz_333_;
  wire [31:0] _zz_334_;
  wire [31:0] _zz_335_;
  wire  _zz_336_;
  wire [0:0] _zz_337_;
  wire [0:0] _zz_338_;
  wire [0:0] _zz_339_;
  wire [2:0] _zz_340_;
  wire [1:0] _zz_341_;
  wire [1:0] _zz_342_;
  wire  _zz_343_;
  wire [0:0] _zz_344_;
  wire [9:0] _zz_345_;
  wire [31:0] _zz_346_;
  wire [31:0] _zz_347_;
  wire [31:0] _zz_348_;
  wire [31:0] _zz_349_;
  wire [31:0] _zz_350_;
  wire [31:0] _zz_351_;
  wire [31:0] _zz_352_;
  wire [31:0] _zz_353_;
  wire [31:0] _zz_354_;
  wire [31:0] _zz_355_;
  wire [31:0] _zz_356_;
  wire [31:0] _zz_357_;
  wire  _zz_358_;
  wire [0:0] _zz_359_;
  wire [0:0] _zz_360_;
  wire [0:0] _zz_361_;
  wire [0:0] _zz_362_;
  wire [1:0] _zz_363_;
  wire [1:0] _zz_364_;
  wire  _zz_365_;
  wire [0:0] _zz_366_;
  wire [7:0] _zz_367_;
  wire [31:0] _zz_368_;
  wire [31:0] _zz_369_;
  wire [31:0] _zz_370_;
  wire [31:0] _zz_371_;
  wire [31:0] _zz_372_;
  wire [31:0] _zz_373_;
  wire [31:0] _zz_374_;
  wire  _zz_375_;
  wire [0:0] _zz_376_;
  wire [0:0] _zz_377_;
  wire [4:0] _zz_378_;
  wire [4:0] _zz_379_;
  wire  _zz_380_;
  wire [0:0] _zz_381_;
  wire [5:0] _zz_382_;
  wire [31:0] _zz_383_;
  wire [0:0] _zz_384_;
  wire [1:0] _zz_385_;
  wire  _zz_386_;
  wire [0:0] _zz_387_;
  wire [0:0] _zz_388_;
  wire [0:0] _zz_389_;
  wire [0:0] _zz_390_;
  wire [0:0] _zz_391_;
  wire [0:0] _zz_392_;
  wire  _zz_393_;
  wire [0:0] _zz_394_;
  wire [2:0] _zz_395_;
  wire [31:0] _zz_396_;
  wire [31:0] _zz_397_;
  wire  _zz_398_;
  wire  _zz_399_;
  wire [31:0] _zz_400_;
  wire [31:0] _zz_401_;
  wire [31:0] _zz_402_;
  wire [31:0] _zz_403_;
  wire [31:0] _zz_404_;
  wire [31:0] _zz_405_;
  wire [31:0] _zz_406_;
  wire [31:0] _zz_407_;
  wire [31:0] _zz_408_;
  wire [31:0] _zz_409_;
  wire [31:0] _zz_410_;
  wire [0:0] _zz_411_;
  wire [0:0] _zz_412_;
  wire [1:0] _zz_413_;
  wire [1:0] _zz_414_;
  wire  _zz_415_;
  wire [0:0] _zz_416_;
  wire [0:0] _zz_417_;
  wire [31:0] _zz_418_;
  wire  _zz_419_;
  wire [0:0] _zz_420_;
  wire [0:0] _zz_421_;
  wire  _zz_422_;
  wire [0:0] _zz_423_;
  wire [0:0] _zz_424_;
  wire  execute_BRANCH_DO;
  wire  decode_BYPASSABLE_EXECUTE_STAGE;
  wire [31:0] execute_PIPELINED_CSR_READ;
  wire `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_1_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_2_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_3_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_4_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_5_;
  wire `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_6_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_7_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_8_;
  wire  decode_CSR_WRITE_OPCODE;
  wire  execute_BYPASSABLE_MEMORY_STAGE;
  wire  decode_BYPASSABLE_MEMORY_STAGE;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_9_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_10_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_11_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_12_;
  wire `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_13_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_14_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_15_;
  wire [1:0] memory_MEMORY_ADDRESS_LOW;
  wire [1:0] execute_MEMORY_ADDRESS_LOW;
  wire [31:0] execute_SHIFT_RIGHT;
  wire [31:0] execute_MUL_LL;
  wire [31:0] writeBack_REGFILE_WRITE_DATA;
  wire [31:0] execute_REGFILE_WRITE_DATA;
  wire  decode_IS_RS1_SIGNED;
  wire [31:0] memory_MEMORY_READ_DATA;
  wire [33:0] execute_MUL_HL;
  wire [33:0] memory_MUL_HH;
  wire [33:0] execute_MUL_HH;
  wire `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_16_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_17_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_18_;
  wire  decode_MEMORY_ENABLE;
  wire [31:0] writeBack_FORMAL_PC_NEXT;
  wire [31:0] memory_FORMAL_PC_NEXT;
  wire [31:0] execute_FORMAL_PC_NEXT;
  wire [31:0] decode_FORMAL_PC_NEXT;
  wire [31:0] decode_SRC1;
  wire  decode_SRC_LESS_UNSIGNED;
  wire [51:0] memory_MUL_LOW;
  wire [33:0] execute_MUL_LH;
  wire  memory_IS_MUL;
  wire  execute_IS_MUL;
  wire  decode_IS_MUL;
  wire [31:0] memory_PC;
  wire [31:0] decode_SRC2;
  wire  decode_CSR_READ_OPCODE;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_19_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_20_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_21_;
  wire  decode_SRC_USE_SUB_LESS;
  wire  decode_IS_RS2_SIGNED;
  wire  decode_IS_CSR;
  wire  decode_IS_DIV;
  wire [31:0] execute_BRANCH_CALC;
  wire [31:0] memory_BRANCH_CALC;
  wire  memory_BRANCH_DO;
  wire [31:0] _zz_22_;
  wire [31:0] execute_PC;
  wire `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_23_;
  wire  _zz_24_;
  wire  decode_RS2_USE;
  wire  decode_RS1_USE;
  wire [31:0] _zz_25_;
  wire  execute_REGFILE_WRITE_VALID;
  wire  execute_BYPASSABLE_EXECUTE_STAGE;
  wire  memory_REGFILE_WRITE_VALID;
  wire  memory_BYPASSABLE_MEMORY_STAGE;
  wire  writeBack_REGFILE_WRITE_VALID;
  reg [31:0] decode_RS2;
  reg [31:0] decode_RS1;
  wire [31:0] memory_SHIFT_RIGHT;
  wire `ShiftCtrlEnum_defaultEncoding_type memory_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_26_;
  wire [31:0] _zz_27_;
  wire `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_28_;
  wire  _zz_29_;
  wire [31:0] _zz_30_;
  wire  execute_SRC_USE_SUB_LESS;
  wire [31:0] _zz_31_;
  wire  execute_SRC_LESS_UNSIGNED;
  wire [31:0] _zz_32_;
  wire [31:0] _zz_33_;
  wire `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_34_;
  wire [31:0] _zz_35_;
  wire [31:0] _zz_36_;
  wire `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_37_;
  wire [31:0] _zz_38_;
  wire [31:0] execute_SRC_ADD_SUB;
  wire  execute_SRC_LESS;
  wire `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_39_;
  wire [31:0] _zz_40_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_41_;
  wire  execute_IS_RS1_SIGNED;
  wire [31:0] execute_RS1;
  wire  execute_IS_DIV;
  wire  execute_IS_RS2_SIGNED;
  wire  memory_IS_DIV;
  wire  writeBack_IS_MUL;
  wire [33:0] writeBack_MUL_HH;
  wire [51:0] writeBack_MUL_LOW;
  wire [33:0] memory_MUL_HL;
  wire [33:0] memory_MUL_LH;
  wire [31:0] memory_MUL_LL;
  wire [51:0] _zz_42_;
  wire [33:0] _zz_43_;
  wire [33:0] _zz_44_;
  wire [33:0] _zz_45_;
  wire [31:0] _zz_46_;
  wire [31:0] execute_SRC2;
  wire [31:0] _zz_47_;
  wire  _zz_48_;
  reg  _zz_49_;
  wire [31:0] _zz_50_;
  wire [31:0] _zz_51_;
  wire [31:0] decode_INSTRUCTION_ANTICIPATED;
  reg  decode_REGFILE_WRITE_VALID;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_52_;
  wire  _zz_53_;
  wire  _zz_54_;
  wire  _zz_55_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_56_;
  wire  _zz_57_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_58_;
  wire  _zz_59_;
  wire  _zz_60_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_61_;
  wire  _zz_62_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_63_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_64_;
  wire  _zz_65_;
  wire  _zz_66_;
  wire  _zz_67_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_68_;
  wire  _zz_69_;
  wire  _zz_70_;
  wire  _zz_71_;
  wire [31:0] memory_PIPELINED_CSR_READ;
  reg [31:0] _zz_72_;
  wire  memory_IS_CSR;
  wire [31:0] _zz_73_;
  wire [31:0] execute_SRC1;
  wire  execute_CSR_READ_OPCODE;
  wire  execute_CSR_WRITE_OPCODE;
  wire  execute_IS_CSR;
  wire `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_74_;
  wire `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_75_;
  wire  _zz_76_;
  wire  _zz_77_;
  wire `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_78_;
  reg [31:0] _zz_79_;
  wire  writeBack_MEMORY_ENABLE;
  wire [1:0] writeBack_MEMORY_ADDRESS_LOW;
  wire [31:0] writeBack_MEMORY_READ_DATA;
  wire [31:0] memory_REGFILE_WRITE_DATA;
  wire  memory_ALIGNEMENT_FAULT;
  wire [31:0] memory_INSTRUCTION;
  wire  memory_MEMORY_ENABLE;
  wire [31:0] _zz_80_;
  wire [1:0] _zz_81_;
  wire [31:0] execute_RS2;
  wire [31:0] execute_SRC_ADD;
  wire [31:0] execute_INSTRUCTION;
  wire  execute_ALIGNEMENT_FAULT;
  wire  execute_MEMORY_ENABLE;
  wire  _zz_82_;
  reg [31:0] _zz_83_;
  wire [31:0] _zz_84_;
  wire [31:0] _zz_85_;
  wire [31:0] _zz_86_;
  wire [31:0] _zz_87_;
  wire [31:0] writeBack_PC /* verilator public */ ;
  wire [31:0] writeBack_INSTRUCTION /* verilator public */ ;
  wire [31:0] decode_PC /* verilator public */ ;
  wire [31:0] decode_INSTRUCTION /* verilator public */ ;
  reg  decode_arbitration_haltItself /* verilator public */ ;
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
  wire  execute_arbitration_haltByOther;
  reg  execute_arbitration_removeIt;
  reg  execute_arbitration_flushAll;
  wire  execute_arbitration_redoIt;
  reg  execute_arbitration_isValid;
  wire  execute_arbitration_isStuck;
  wire  execute_arbitration_isStuckByOthers;
  wire  execute_arbitration_isFlushed;
  wire  execute_arbitration_isMoving;
  wire  execute_arbitration_isFiring;
  reg  memory_arbitration_haltItself;
  wire  memory_arbitration_haltByOther;
  reg  memory_arbitration_removeIt;
  reg  memory_arbitration_flushAll;
  wire  memory_arbitration_redoIt;
  reg  memory_arbitration_isValid;
  wire  memory_arbitration_isStuck;
  wire  memory_arbitration_isStuckByOthers;
  wire  memory_arbitration_isFlushed;
  wire  memory_arbitration_isMoving;
  wire  memory_arbitration_isFiring;
  wire  writeBack_arbitration_haltItself;
  wire  writeBack_arbitration_haltByOther;
  reg  writeBack_arbitration_removeIt;
  wire  writeBack_arbitration_flushAll;
  wire  writeBack_arbitration_redoIt;
  reg  writeBack_arbitration_isValid /* verilator public */ ;
  wire  writeBack_arbitration_isStuck;
  wire  writeBack_arbitration_isStuckByOthers;
  wire  writeBack_arbitration_isFlushed;
  wire  writeBack_arbitration_isMoving;
  wire  writeBack_arbitration_isFiring /* verilator public */ ;
  reg  _zz_88_;
  wire  _zz_89_;
  reg  _zz_90_;
  reg  _zz_91_;
  reg [31:0] _zz_92_;
  wire  contextSwitching;
  reg [1:0] CsrPlugin_privilege;
  reg  execute_exception_agregat_valid;
  reg [3:0] execute_exception_agregat_payload_code;
  wire [31:0] execute_exception_agregat_payload_badAddr;
  wire  _zz_93_;
  wire [31:0] _zz_94_;
  wire  _zz_95_;
  wire  IBusSimplePlugin_jump_pcLoad_valid;
  wire [31:0] IBusSimplePlugin_jump_pcLoad_payload;
  wire [1:0] _zz_96_;
  wire  _zz_97_;
  wire  IBusSimplePlugin_fetchPc_preOutput_valid;
  wire  IBusSimplePlugin_fetchPc_preOutput_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_preOutput_payload;
  wire  _zz_98_;
  wire  IBusSimplePlugin_fetchPc_output_valid;
  wire  IBusSimplePlugin_fetchPc_output_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_output_payload;
  reg [31:0] IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg  IBusSimplePlugin_fetchPc_inc;
  wire  IBusSimplePlugin_fetchPc_propagatePc;
  reg [31:0] IBusSimplePlugin_fetchPc_pc;
  reg  IBusSimplePlugin_fetchPc_samplePcNext;
  reg  _zz_99_;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_0_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  reg  IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_1_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_2_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_2_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_2_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_2_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_2_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_2_inputSample;
  wire  _zz_100_;
  wire  _zz_101_;
  wire  _zz_102_;
  wire  _zz_103_;
  reg  _zz_104_;
  reg [31:0] _zz_105_;
  wire  _zz_106_;
  reg  _zz_107_;
  reg [31:0] _zz_108_;
  reg  IBusSimplePlugin_iBusRsp_readyForError;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
  wire  IBusSimplePlugin_injector_decodeInput_valid;
  wire  IBusSimplePlugin_injector_decodeInput_ready;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire  IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire  IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg  _zz_109_;
  reg [31:0] _zz_110_;
  reg  _zz_111_;
  reg [31:0] _zz_112_;
  reg  _zz_113_;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg  IBusSimplePlugin_injector_nextPcCalc_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_1;
  reg  IBusSimplePlugin_injector_nextPcCalc_2;
  reg  IBusSimplePlugin_injector_nextPcCalc_3;
  reg  IBusSimplePlugin_injector_decodeRemoved;
  reg [31:0] IBusSimplePlugin_injector_formal_rawInDecode;
  wire  IBusSimplePlugin_cmd_valid;
  wire  IBusSimplePlugin_cmd_ready;
  wire [31:0] IBusSimplePlugin_cmd_payload_pc;
  reg [2:0] IBusSimplePlugin_pendingCmd;
  wire [2:0] IBusSimplePlugin_pendingCmdNext;
  wire  IBusSimplePlugin_cmdFork_pendingFull;
  reg  IBusSimplePlugin_cmdFork_cmdKeep;
  reg  IBusSimplePlugin_cmdFork_cmdFired;
  reg [2:0] IBusSimplePlugin_rspJoin_discardCounter;
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
  wire  _zz_114_;
  reg [31:0] _zz_115_;
  reg [3:0] _zz_116_;
  wire [3:0] execute_DBusSimplePlugin_formalMask;
  reg [31:0] writeBack_DBusSimplePlugin_rspShifted;
  wire  _zz_117_;
  reg [31:0] _zz_118_;
  wire  _zz_119_;
  reg [31:0] _zz_120_;
  reg [31:0] writeBack_DBusSimplePlugin_rspFormated;
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
  wire  _zz_121_;
  wire  _zz_122_;
  wire  _zz_123_;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack;
  wire  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  reg [3:0] CsrPlugin_exceptionPortCtrl_exceptionContext_code;
  reg [31:0] CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
  wire [1:0] CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
  wire  memory_exception_agregat_valid;
  wire [3:0] memory_exception_agregat_payload_code;
  wire [31:0] memory_exception_agregat_payload_badAddr;
  wire [1:0] _zz_124_;
  wire  _zz_125_;
  wire [0:0] _zz_126_;
  reg  CsrPlugin_interrupt;
  reg [3:0] CsrPlugin_interruptCode /* verilator public */ ;
  reg [1:0] CsrPlugin_interruptTargetPrivilege;
  wire  CsrPlugin_exception;
  wire  CsrPlugin_writeBackWasWfi;
  reg  CsrPlugin_pipelineLiberator_done;
  wire  CsrPlugin_interruptJump /* verilator public */ ;
  reg  CsrPlugin_hadException;
  reg [1:0] CsrPlugin_targetPrivilege;
  reg [3:0] CsrPlugin_trapCause;
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
  wire [27:0] _zz_127_;
  wire  _zz_128_;
  wire  _zz_129_;
  wire  _zz_130_;
  wire  _zz_131_;
  wire  _zz_132_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_133_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_134_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_135_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_136_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_137_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_138_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_139_;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress1;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress2;
  wire [31:0] decode_RegFilePlugin_rs1Data;
  wire [31:0] decode_RegFilePlugin_rs2Data;
  reg  writeBack_RegFilePlugin_regFileWrite_valid /* verilator public */ ;
  wire [4:0] writeBack_RegFilePlugin_regFileWrite_payload_address /* verilator public */ ;
  wire [31:0] writeBack_RegFilePlugin_regFileWrite_payload_data /* verilator public */ ;
  reg  _zz_140_;
  reg  execute_MulPlugin_aSigned;
  reg  execute_MulPlugin_bSigned;
  wire [31:0] execute_MulPlugin_a;
  wire [31:0] execute_MulPlugin_b;
  wire [15:0] execute_MulPlugin_aULow;
  wire [15:0] execute_MulPlugin_bULow;
  wire [16:0] execute_MulPlugin_aSLow;
  wire [16:0] execute_MulPlugin_bSLow;
  wire [16:0] execute_MulPlugin_aHigh;
  wire [16:0] execute_MulPlugin_bHigh;
  wire [65:0] writeBack_MulPlugin_result;
  reg [32:0] memory_MulDivIterativePlugin_rs1;
  reg [31:0] memory_MulDivIterativePlugin_rs2;
  reg [64:0] memory_MulDivIterativePlugin_accumulator;
  reg  memory_MulDivIterativePlugin_div_needRevert;
  reg  memory_MulDivIterativePlugin_div_counter_willIncrement;
  reg  memory_MulDivIterativePlugin_div_counter_willClear;
  reg [5:0] memory_MulDivIterativePlugin_div_counter_valueNext;
  reg [5:0] memory_MulDivIterativePlugin_div_counter_value;
  wire  memory_MulDivIterativePlugin_div_counter_willOverflowIfInc;
  wire  memory_MulDivIterativePlugin_div_counter_willOverflow;
  reg  memory_MulDivIterativePlugin_div_done;
  reg [31:0] memory_MulDivIterativePlugin_div_result;
  wire [31:0] _zz_141_;
  wire [32:0] _zz_142_;
  wire [32:0] _zz_143_;
  wire [31:0] _zz_144_;
  wire  _zz_145_;
  wire  _zz_146_;
  reg [32:0] _zz_147_;
  reg [31:0] execute_IntAluPlugin_bitwise;
  reg [31:0] _zz_148_;
  reg [31:0] _zz_149_;
  wire  _zz_150_;
  reg [19:0] _zz_151_;
  wire  _zz_152_;
  reg [19:0] _zz_153_;
  reg [31:0] _zz_154_;
  (* keep *) wire [31:0] execute_SrcPlugin_add;
  (* keep *) wire [31:0] execute_SrcPlugin_sub;
  wire  execute_SrcPlugin_less;
  wire [4:0] execute_FullBarrelShifterPlugin_amplitude;
  reg [31:0] _zz_155_;
  wire [31:0] execute_FullBarrelShifterPlugin_reversed;
  reg [31:0] _zz_156_;
  reg  _zz_157_;
  reg  _zz_158_;
  reg  _zz_159_;
  reg [4:0] _zz_160_;
  reg [31:0] _zz_161_;
  wire  _zz_162_;
  wire  _zz_163_;
  wire  _zz_164_;
  wire  _zz_165_;
  wire  _zz_166_;
  wire  _zz_167_;
  wire  execute_BranchPlugin_eq;
  wire [2:0] _zz_168_;
  reg  _zz_169_;
  reg  _zz_170_;
  wire [31:0] execute_BranchPlugin_branch_src1;
  wire  _zz_171_;
  reg [10:0] _zz_172_;
  wire  _zz_173_;
  reg [19:0] _zz_174_;
  wire  _zz_175_;
  reg [18:0] _zz_176_;
  reg [31:0] _zz_177_;
  wire [31:0] execute_BranchPlugin_branch_src2;
  wire [31:0] execute_BranchPlugin_branchAdder;
  reg [31:0] execute_to_memory_BRANCH_CALC;
  reg  decode_to_execute_IS_DIV;
  reg  execute_to_memory_IS_DIV;
  reg  decode_to_execute_IS_CSR;
  reg  execute_to_memory_IS_CSR;
  reg  decode_to_execute_IS_RS2_SIGNED;
  reg  decode_to_execute_SRC_USE_SUB_LESS;
  reg `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg  decode_to_execute_CSR_READ_OPCODE;
  reg [31:0] decode_to_execute_SRC2;
  reg [31:0] decode_to_execute_PC;
  reg [31:0] execute_to_memory_PC;
  reg [31:0] memory_to_writeBack_PC;
  reg  decode_to_execute_IS_MUL;
  reg  execute_to_memory_IS_MUL;
  reg  memory_to_writeBack_IS_MUL;
  reg [33:0] execute_to_memory_MUL_LH;
  reg [51:0] memory_to_writeBack_MUL_LOW;
  reg  decode_to_execute_SRC_LESS_UNSIGNED;
  reg [31:0] decode_to_execute_SRC1;
  reg [31:0] decode_to_execute_INSTRUCTION;
  reg [31:0] execute_to_memory_INSTRUCTION;
  reg [31:0] memory_to_writeBack_INSTRUCTION;
  reg [31:0] decode_to_execute_FORMAL_PC_NEXT;
  reg [31:0] execute_to_memory_FORMAL_PC_NEXT;
  reg [31:0] memory_to_writeBack_FORMAL_PC_NEXT;
  reg  decode_to_execute_MEMORY_ENABLE;
  reg  execute_to_memory_MEMORY_ENABLE;
  reg  memory_to_writeBack_MEMORY_ENABLE;
  reg `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg  decode_to_execute_REGFILE_WRITE_VALID;
  reg  execute_to_memory_REGFILE_WRITE_VALID;
  reg  memory_to_writeBack_REGFILE_WRITE_VALID;
  reg [33:0] execute_to_memory_MUL_HH;
  reg [33:0] memory_to_writeBack_MUL_HH;
  reg [31:0] decode_to_execute_RS2;
  reg [33:0] execute_to_memory_MUL_HL;
  reg [31:0] memory_to_writeBack_MEMORY_READ_DATA;
  reg  decode_to_execute_IS_RS1_SIGNED;
  reg [31:0] execute_to_memory_REGFILE_WRITE_DATA;
  reg [31:0] memory_to_writeBack_REGFILE_WRITE_DATA;
  reg [31:0] decode_to_execute_RS1;
  reg [31:0] execute_to_memory_MUL_LL;
  reg [31:0] execute_to_memory_SHIFT_RIGHT;
  reg [1:0] execute_to_memory_MEMORY_ADDRESS_LOW;
  reg [1:0] memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg  decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg  execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg  execute_to_memory_ALIGNEMENT_FAULT;
  reg  decode_to_execute_CSR_WRITE_OPCODE;
  reg `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg `ShiftCtrlEnum_defaultEncoding_type execute_to_memory_SHIFT_CTRL;
  reg `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg [31:0] execute_to_memory_PIPELINED_CSR_READ;
  reg  decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg  execute_to_memory_BRANCH_DO;
  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;
  assign _zz_190_ = (memory_arbitration_isValid && memory_IS_DIV);
  assign _zz_191_ = (! memory_MulDivIterativePlugin_div_done);
  assign _zz_192_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_193_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_194_ = (IBusSimplePlugin_fetchPc_preOutput_valid && IBusSimplePlugin_fetchPc_preOutput_ready);
  assign _zz_195_ = (! memory_arbitration_isStuck);
  assign _zz_196_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_197_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_198_ = execute_INSTRUCTION[13];
  assign _zz_199_ = execute_INSTRUCTION[13 : 12];
  assign _zz_200_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_201_ = (_zz_96_ & (~ _zz_202_));
  assign _zz_202_ = (_zz_96_ - (2'b01));
  assign _zz_203_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_204_ = {29'd0, _zz_203_};
  assign _zz_205_ = (IBusSimplePlugin_pendingCmd + _zz_207_);
  assign _zz_206_ = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign _zz_207_ = {2'd0, _zz_206_};
  assign _zz_208_ = iBus_rsp_valid;
  assign _zz_209_ = {2'd0, _zz_208_};
  assign _zz_210_ = (iBus_rsp_valid && (IBusSimplePlugin_rspJoin_discardCounter != (3'b000)));
  assign _zz_211_ = {2'd0, _zz_210_};
  assign _zz_212_ = (IBusSimplePlugin_pendingCmd + _zz_214_);
  assign _zz_213_ = IBusSimplePlugin_cmd_valid;
  assign _zz_214_ = {2'd0, _zz_213_};
  assign _zz_215_ = iBus_rsp_valid;
  assign _zz_216_ = {2'd0, _zz_215_};
  assign _zz_217_ = (_zz_124_ & (~ _zz_218_));
  assign _zz_218_ = (_zz_124_ - (2'b01));
  assign _zz_219_ = (memory_INSTRUCTION[5] ? (3'b110) : (3'b100));
  assign _zz_220_ = {1'd0, _zz_219_};
  assign _zz_221_ = _zz_127_[0 : 0];
  assign _zz_222_ = _zz_127_[1 : 1];
  assign _zz_223_ = _zz_127_[2 : 2];
  assign _zz_224_ = _zz_127_[5 : 5];
  assign _zz_225_ = _zz_127_[6 : 6];
  assign _zz_226_ = _zz_127_[8 : 8];
  assign _zz_227_ = _zz_127_[13 : 13];
  assign _zz_228_ = _zz_127_[16 : 16];
  assign _zz_229_ = _zz_127_[17 : 17];
  assign _zz_230_ = _zz_127_[20 : 20];
  assign _zz_231_ = _zz_127_[23 : 23];
  assign _zz_232_ = _zz_127_[24 : 24];
  assign _zz_233_ = _zz_127_[25 : 25];
  assign _zz_234_ = ($signed(_zz_235_) + $signed(_zz_240_));
  assign _zz_235_ = ($signed(_zz_236_) + $signed(_zz_238_));
  assign _zz_236_ = (52'b0000000000000000000000000000000000000000000000000000);
  assign _zz_237_ = {1'b0,memory_MUL_LL};
  assign _zz_238_ = {{19{_zz_237_[32]}}, _zz_237_};
  assign _zz_239_ = ({16'd0,memory_MUL_LH} <<< 16);
  assign _zz_240_ = {{2{_zz_239_[49]}}, _zz_239_};
  assign _zz_241_ = ({16'd0,memory_MUL_HL} <<< 16);
  assign _zz_242_ = {{2{_zz_241_[49]}}, _zz_241_};
  assign _zz_243_ = {{14{writeBack_MUL_LOW[51]}}, writeBack_MUL_LOW};
  assign _zz_244_ = ({32'd0,writeBack_MUL_HH} <<< 32);
  assign _zz_245_ = writeBack_MUL_LOW[31 : 0];
  assign _zz_246_ = writeBack_MulPlugin_result[63 : 32];
  assign _zz_247_ = memory_MulDivIterativePlugin_div_counter_willIncrement;
  assign _zz_248_ = {5'd0, _zz_247_};
  assign _zz_249_ = {1'd0, memory_MulDivIterativePlugin_rs2};
  assign _zz_250_ = {_zz_141_,(! _zz_143_[32])};
  assign _zz_251_ = _zz_143_[31:0];
  assign _zz_252_ = _zz_142_[31:0];
  assign _zz_253_ = _zz_254_;
  assign _zz_254_ = _zz_255_;
  assign _zz_255_ = ({1'b0,(memory_MulDivIterativePlugin_div_needRevert ? (~ _zz_144_) : _zz_144_)} + _zz_257_);
  assign _zz_256_ = memory_MulDivIterativePlugin_div_needRevert;
  assign _zz_257_ = {32'd0, _zz_256_};
  assign _zz_258_ = _zz_146_;
  assign _zz_259_ = {32'd0, _zz_258_};
  assign _zz_260_ = _zz_145_;
  assign _zz_261_ = {31'd0, _zz_260_};
  assign _zz_262_ = execute_SRC_LESS;
  assign _zz_263_ = (3'b100);
  assign _zz_264_ = decode_INSTRUCTION[19 : 15];
  assign _zz_265_ = decode_INSTRUCTION[31 : 20];
  assign _zz_266_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_267_ = (execute_SRC1 + execute_SRC2);
  assign _zz_268_ = (execute_SRC1 - execute_SRC2);
  assign _zz_269_ = ($signed(_zz_271_) >>> execute_FullBarrelShifterPlugin_amplitude);
  assign _zz_270_ = _zz_269_[31 : 0];
  assign _zz_271_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_FullBarrelShifterPlugin_reversed[31]),execute_FullBarrelShifterPlugin_reversed};
  assign _zz_272_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_273_ = execute_INSTRUCTION[31 : 20];
  assign _zz_274_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_275_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_276_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_277_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_278_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_279_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_280_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_281_ = 1'b1;
  assign _zz_282_ = 1'b1;
  assign _zz_283_ = _zz_97_;
  assign _zz_284_ = (32'b00010000000000000011000001010000);
  assign _zz_285_ = ((decode_INSTRUCTION & (32'b00010000000100000011000001010000)) == (32'b00000000000100000000000001010000));
  assign _zz_286_ = ((decode_INSTRUCTION & (32'b00010000010000000011000001010000)) == (32'b00010000000000000000000001010000));
  assign _zz_287_ = ((decode_INSTRUCTION & (32'b00000010000000000100000001100100)) == (32'b00000010000000000100000000100000));
  assign _zz_288_ = (1'b0);
  assign _zz_289_ = ({(_zz_292_ == _zz_293_),(_zz_294_ == _zz_295_)} != (2'b00));
  assign _zz_290_ = ({_zz_296_,_zz_297_} != (2'b00));
  assign _zz_291_ = {(_zz_298_ != (1'b0)),{(_zz_299_ != _zz_300_),{_zz_301_,{_zz_302_,_zz_303_}}}};
  assign _zz_292_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010000));
  assign _zz_293_ = (32'b00000000000000000010000000000000);
  assign _zz_294_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000000));
  assign _zz_295_ = (32'b00000000000000000001000000000000);
  assign _zz_296_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001100100)) == (32'b00000000000000000000000000100100));
  assign _zz_297_ = ((decode_INSTRUCTION & (32'b00000000000000000100000000010100)) == (32'b00000000000000000100000000010000));
  assign _zz_298_ = ((decode_INSTRUCTION & (32'b00000000000000000110000000010100)) == (32'b00000000000000000010000000010000));
  assign _zz_299_ = {_zz_129_,{_zz_304_,{_zz_305_,_zz_306_}}};
  assign _zz_300_ = (5'b00000);
  assign _zz_301_ = (_zz_131_ != (1'b0));
  assign _zz_302_ = (_zz_307_ != (1'b0));
  assign _zz_303_ = {(_zz_308_ != _zz_309_),{_zz_310_,{_zz_311_,_zz_312_}}};
  assign _zz_304_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000110000)) == (32'b00000000000000000010000000010000));
  assign _zz_305_ = ((decode_INSTRUCTION & _zz_313_) == (32'b00000000000000000010000000100000));
  assign _zz_306_ = {(_zz_314_ == _zz_315_),(_zz_316_ == _zz_317_)};
  assign _zz_307_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001011000)) == (32'b00000000000000000000000001000000));
  assign _zz_308_ = _zz_132_;
  assign _zz_309_ = (1'b0);
  assign _zz_310_ = ({_zz_318_,{_zz_319_,_zz_320_}} != (5'b00000));
  assign _zz_311_ = ({_zz_321_,_zz_322_} != (2'b00));
  assign _zz_312_ = {(_zz_323_ != _zz_324_),{_zz_325_,{_zz_326_,_zz_327_}}};
  assign _zz_313_ = (32'b00000010000000000010000000100000);
  assign _zz_314_ = (decode_INSTRUCTION & (32'b00000010000000000001000000100000));
  assign _zz_315_ = (32'b00000000000000000000000000100000);
  assign _zz_316_ = (decode_INSTRUCTION & (32'b00000000000000000001000000110000));
  assign _zz_317_ = (32'b00000000000000000000000000010000);
  assign _zz_318_ = ((decode_INSTRUCTION & _zz_328_) == (32'b00000000000000000000000001000000));
  assign _zz_319_ = _zz_129_;
  assign _zz_320_ = {_zz_329_,{_zz_330_,_zz_331_}};
  assign _zz_321_ = (_zz_332_ == _zz_333_);
  assign _zz_322_ = (_zz_334_ == _zz_335_);
  assign _zz_323_ = {_zz_336_,{_zz_337_,_zz_338_}};
  assign _zz_324_ = (3'b000);
  assign _zz_325_ = ({_zz_339_,_zz_340_} != (4'b0000));
  assign _zz_326_ = (_zz_341_ != _zz_342_);
  assign _zz_327_ = {_zz_343_,{_zz_344_,_zz_345_}};
  assign _zz_328_ = (32'b00000000000000000000000001000000);
  assign _zz_329_ = ((decode_INSTRUCTION & _zz_346_) == (32'b00000000000000000100000000100000));
  assign _zz_330_ = (_zz_347_ == _zz_348_);
  assign _zz_331_ = (_zz_349_ == _zz_350_);
  assign _zz_332_ = (decode_INSTRUCTION & (32'b00000000000000000111000000110100));
  assign _zz_333_ = (32'b00000000000000000101000000010000);
  assign _zz_334_ = (decode_INSTRUCTION & (32'b00000010000000000111000001100100));
  assign _zz_335_ = (32'b00000000000000000101000000100000);
  assign _zz_336_ = ((decode_INSTRUCTION & _zz_351_) == (32'b01000000000000000001000000010000));
  assign _zz_337_ = (_zz_352_ == _zz_353_);
  assign _zz_338_ = (_zz_354_ == _zz_355_);
  assign _zz_339_ = (_zz_356_ == _zz_357_);
  assign _zz_340_ = {_zz_358_,{_zz_359_,_zz_360_}};
  assign _zz_341_ = {_zz_131_,_zz_130_};
  assign _zz_342_ = (2'b00);
  assign _zz_343_ = ({_zz_361_,_zz_362_} != (2'b00));
  assign _zz_344_ = (_zz_363_ != _zz_364_);
  assign _zz_345_ = {_zz_365_,{_zz_366_,_zz_367_}};
  assign _zz_346_ = (32'b00000000000000000100000000100000);
  assign _zz_347_ = (decode_INSTRUCTION & (32'b00000000000000000000000000110000));
  assign _zz_348_ = (32'b00000000000000000000000000010000);
  assign _zz_349_ = (decode_INSTRUCTION & (32'b00000010000000000000000000100000));
  assign _zz_350_ = (32'b00000000000000000000000000100000);
  assign _zz_351_ = (32'b01000000000000000011000001010100);
  assign _zz_352_ = (decode_INSTRUCTION & (32'b00000000000000000111000000110100));
  assign _zz_353_ = (32'b00000000000000000001000000010000);
  assign _zz_354_ = (decode_INSTRUCTION & (32'b00000010000000000111000001010100));
  assign _zz_355_ = (32'b00000000000000000001000000010000);
  assign _zz_356_ = (decode_INSTRUCTION & (32'b00000000000000000000000001000100));
  assign _zz_357_ = (32'b00000000000000000000000000000000);
  assign _zz_358_ = ((decode_INSTRUCTION & _zz_368_) == (32'b00000000000000000000000000000000));
  assign _zz_359_ = (_zz_369_ == _zz_370_);
  assign _zz_360_ = (_zz_371_ == _zz_372_);
  assign _zz_361_ = (_zz_373_ == _zz_374_);
  assign _zz_362_ = _zz_130_;
  assign _zz_363_ = {_zz_375_,_zz_129_};
  assign _zz_364_ = (2'b00);
  assign _zz_365_ = ({_zz_376_,_zz_377_} != (2'b00));
  assign _zz_366_ = (_zz_378_ != _zz_379_);
  assign _zz_367_ = {_zz_380_,{_zz_381_,_zz_382_}};
  assign _zz_368_ = (32'b00000000000000000000000000011000);
  assign _zz_369_ = (decode_INSTRUCTION & (32'b00000000000000000110000000000100));
  assign _zz_370_ = (32'b00000000000000000010000000000000);
  assign _zz_371_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000100));
  assign _zz_372_ = (32'b00000000000000000001000000000000);
  assign _zz_373_ = (decode_INSTRUCTION & (32'b00000000000000000000000001000100));
  assign _zz_374_ = (32'b00000000000000000000000000000100);
  assign _zz_375_ = ((decode_INSTRUCTION & (32'b00000000000000000001000000000000)) == (32'b00000000000000000001000000000000));
  assign _zz_376_ = _zz_129_;
  assign _zz_377_ = ((decode_INSTRUCTION & _zz_383_) == (32'b00000000000000000010000000000000));
  assign _zz_378_ = {_zz_129_,{_zz_128_,{_zz_384_,_zz_385_}}};
  assign _zz_379_ = (5'b00000);
  assign _zz_380_ = ({_zz_386_,{_zz_387_,_zz_388_}} != (3'b000));
  assign _zz_381_ = ({_zz_389_,_zz_390_} != (2'b00));
  assign _zz_382_ = {(_zz_391_ != _zz_392_),{_zz_393_,{_zz_394_,_zz_395_}}};
  assign _zz_383_ = (32'b00000000000000000011000000000000);
  assign _zz_384_ = (_zz_396_ == _zz_397_);
  assign _zz_385_ = {_zz_398_,_zz_399_};
  assign _zz_386_ = ((decode_INSTRUCTION & _zz_400_) == (32'b00000000000000000000000001000000));
  assign _zz_387_ = (_zz_401_ == _zz_402_);
  assign _zz_388_ = (_zz_403_ == _zz_404_);
  assign _zz_389_ = (_zz_405_ == _zz_406_);
  assign _zz_390_ = (_zz_407_ == _zz_408_);
  assign _zz_391_ = (_zz_409_ == _zz_410_);
  assign _zz_392_ = (1'b0);
  assign _zz_393_ = ({_zz_411_,_zz_412_} != (2'b00));
  assign _zz_394_ = (_zz_413_ != _zz_414_);
  assign _zz_395_ = {_zz_415_,{_zz_416_,_zz_417_}};
  assign _zz_396_ = (decode_INSTRUCTION & (32'b00000000000000000001000000010000));
  assign _zz_397_ = (32'b00000000000000000001000000010000);
  assign _zz_398_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000010000)) == (32'b00000000000000000010000000010000));
  assign _zz_399_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_400_ = (32'b00000000000000000000000001010000);
  assign _zz_401_ = (decode_INSTRUCTION & (32'b00000000000000000000000000110000));
  assign _zz_402_ = (32'b00000000000000000000000000000000);
  assign _zz_403_ = (decode_INSTRUCTION & (32'b00000000010000000011000001000000));
  assign _zz_404_ = (32'b00000000000000000000000001000000);
  assign _zz_405_ = (decode_INSTRUCTION & (32'b00000000000000000000000000110100));
  assign _zz_406_ = (32'b00000000000000000000000000100000);
  assign _zz_407_ = (decode_INSTRUCTION & (32'b00000000000000000000000001100100));
  assign _zz_408_ = (32'b00000000000000000000000000100000);
  assign _zz_409_ = (decode_INSTRUCTION & (32'b00000010000000000100000001110100));
  assign _zz_410_ = (32'b00000010000000000000000000110000);
  assign _zz_411_ = _zz_129_;
  assign _zz_412_ = ((decode_INSTRUCTION & _zz_418_) == (32'b00000000000000000000000000100000));
  assign _zz_413_ = {_zz_129_,_zz_128_};
  assign _zz_414_ = (2'b00);
  assign _zz_415_ = ({_zz_419_,{_zz_420_,_zz_421_}} != (3'b000));
  assign _zz_416_ = (_zz_422_ != (1'b0));
  assign _zz_417_ = ({_zz_423_,_zz_424_} != (2'b00));
  assign _zz_418_ = (32'b00000000000000000000000001110000);
  assign _zz_419_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000001000000));
  assign _zz_420_ = ((decode_INSTRUCTION & (32'b01000000000000000000000000110000)) == (32'b01000000000000000000000000110000));
  assign _zz_421_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000010100)) == (32'b00000000000000000010000000010000));
  assign _zz_422_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000000000));
  assign _zz_423_ = ((decode_INSTRUCTION & (32'b00000000000000000001000001010000)) == (32'b00000000000000000001000001010000));
  assign _zz_424_ = ((decode_INSTRUCTION & (32'b00000000000000000010000001010000)) == (32'b00000000000000000010000001010000));
  always @ (posedge io_clk) begin
    if(_zz_49_) begin
      RegFilePlugin_regFile[writeBack_RegFilePlugin_regFileWrite_payload_address] <= writeBack_RegFilePlugin_regFileWrite_payload_data;
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_281_) begin
      _zz_180_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_282_) begin
      _zz_181_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid(_zz_178_),
    .io_push_ready(_zz_185_),
    .io_push_payload_error(iBus_rsp_payload_error),
    .io_push_payload_inst(iBus_rsp_payload_inst),
    .io_pop_valid(_zz_186_),
    .io_pop_ready(IBusSimplePlugin_rspJoin_rspBufferOutput_ready),
    .io_pop_payload_error(_zz_187_),
    .io_pop_payload_inst(_zz_188_),
    .io_flush(_zz_179_),
    .io_occupancy(_zz_189_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  always @(*) begin
    case(_zz_283_)
      1'b0 : begin
        _zz_182_ = _zz_92_;
      end
      default : begin
        _zz_182_ = _zz_94_;
      end
    endcase
  end

  always @(*) begin
    case(_zz_126_)
      1'b0 : begin
        _zz_183_ = _zz_220_;
        _zz_184_ = memory_REGFILE_WRITE_DATA;
      end
      default : begin
        _zz_183_ = (4'b0000);
        _zz_184_ = _zz_94_;
      end
    endcase
  end

  assign execute_BRANCH_DO = _zz_24_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_57_;
  assign execute_PIPELINED_CSR_READ = _zz_73_;
  assign decode_ALU_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign _zz_4_ = _zz_5_;
  assign decode_SHIFT_CTRL = _zz_6_;
  assign _zz_7_ = _zz_8_;
  assign decode_CSR_WRITE_OPCODE = _zz_77_;
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_60_;
  assign _zz_9_ = _zz_10_;
  assign _zz_11_ = _zz_12_;
  assign decode_ENV_CTRL = _zz_13_;
  assign _zz_14_ = _zz_15_;
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = _zz_81_;
  assign execute_SHIFT_RIGHT = _zz_27_;
  assign execute_MUL_LL = _zz_46_;
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_40_;
  assign decode_IS_RS1_SIGNED = _zz_59_;
  assign memory_MEMORY_READ_DATA = _zz_80_;
  assign execute_MUL_HL = _zz_44_;
  assign memory_MUL_HH = execute_to_memory_MUL_HH;
  assign execute_MUL_HH = _zz_43_;
  assign decode_BRANCH_CTRL = _zz_16_;
  assign _zz_17_ = _zz_18_;
  assign decode_MEMORY_ENABLE = _zz_70_;
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_84_;
  assign decode_SRC1 = _zz_38_;
  assign decode_SRC_LESS_UNSIGNED = _zz_55_;
  assign memory_MUL_LOW = _zz_42_;
  assign execute_MUL_LH = _zz_45_;
  assign memory_IS_MUL = execute_to_memory_IS_MUL;
  assign execute_IS_MUL = decode_to_execute_IS_MUL;
  assign decode_IS_MUL = _zz_67_;
  assign memory_PC = execute_to_memory_PC;
  assign decode_SRC2 = _zz_35_;
  assign decode_CSR_READ_OPCODE = _zz_76_;
  assign decode_ALU_BITWISE_CTRL = _zz_19_;
  assign _zz_20_ = _zz_21_;
  assign decode_SRC_USE_SUB_LESS = _zz_69_;
  assign decode_IS_RS2_SIGNED = _zz_53_;
  assign decode_IS_CSR = _zz_71_;
  assign decode_IS_DIV = _zz_54_;
  assign execute_BRANCH_CALC = _zz_22_;
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_PC = decode_to_execute_PC;
  assign execute_BRANCH_CTRL = _zz_23_;
  assign decode_RS2_USE = _zz_66_;
  assign decode_RS1_USE = _zz_62_;
  assign _zz_25_ = execute_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    decode_RS2 = _zz_50_;
    decode_RS1 = _zz_51_;
    if(_zz_159_)begin
      if((_zz_160_ == decode_INSTRUCTION[19 : 15]))begin
        decode_RS1 = _zz_161_;
      end
      if((_zz_160_ == decode_INSTRUCTION[24 : 20]))begin
        decode_RS2 = _zz_161_;
      end
    end
    if((writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID))begin
      if(1'b1)begin
        if(_zz_162_)begin
          decode_RS1 = _zz_79_;
        end
        if(_zz_163_)begin
          decode_RS2 = _zz_79_;
        end
      end
    end
    if((memory_arbitration_isValid && memory_REGFILE_WRITE_VALID))begin
      if(memory_BYPASSABLE_MEMORY_STAGE)begin
        if(_zz_164_)begin
          decode_RS1 = _zz_72_;
        end
        if(_zz_165_)begin
          decode_RS2 = _zz_72_;
        end
      end
    end
    if((execute_arbitration_isValid && execute_REGFILE_WRITE_VALID))begin
      if(execute_BYPASSABLE_EXECUTE_STAGE)begin
        if(_zz_166_)begin
          decode_RS1 = _zz_25_;
        end
        if(_zz_167_)begin
          decode_RS2 = _zz_25_;
        end
      end
    end
  end

  assign memory_SHIFT_RIGHT = execute_to_memory_SHIFT_RIGHT;
  assign memory_SHIFT_CTRL = _zz_26_;
  assign execute_SHIFT_CTRL = _zz_28_;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign _zz_32_ = decode_PC;
  assign _zz_33_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_34_;
  assign _zz_36_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_37_;
  assign execute_SRC_ADD_SUB = _zz_31_;
  assign execute_SRC_LESS = _zz_29_;
  assign execute_ALU_CTRL = _zz_39_;
  assign execute_ALU_BITWISE_CTRL = _zz_41_;
  assign execute_IS_RS1_SIGNED = decode_to_execute_IS_RS1_SIGNED;
  assign execute_RS1 = decode_to_execute_RS1;
  assign execute_IS_DIV = decode_to_execute_IS_DIV;
  assign execute_IS_RS2_SIGNED = decode_to_execute_IS_RS2_SIGNED;
  assign memory_IS_DIV = execute_to_memory_IS_DIV;
  assign writeBack_IS_MUL = memory_to_writeBack_IS_MUL;
  assign writeBack_MUL_HH = memory_to_writeBack_MUL_HH;
  assign writeBack_MUL_LOW = memory_to_writeBack_MUL_LOW;
  assign memory_MUL_HL = execute_to_memory_MUL_HL;
  assign memory_MUL_LH = execute_to_memory_MUL_LH;
  assign memory_MUL_LL = execute_to_memory_MUL_LL;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign _zz_47_ = writeBack_INSTRUCTION;
  assign _zz_48_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_49_ = 1'b0;
    if(writeBack_RegFilePlugin_regFileWrite_valid)begin
      _zz_49_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = _zz_87_;
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_65_;
    if((decode_INSTRUCTION[11 : 7] == (5'b00000)))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  assign memory_PIPELINED_CSR_READ = execute_to_memory_PIPELINED_CSR_READ;
  always @ (*) begin
    _zz_72_ = memory_REGFILE_WRITE_DATA;
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_INSTRUCTION[5])) && (! dBus_rsp_ready)))begin
      memory_arbitration_haltItself = 1'b1;
    end
    if((memory_arbitration_isValid && memory_IS_CSR))begin
      _zz_72_ = memory_PIPELINED_CSR_READ;
    end
    memory_MulDivIterativePlugin_div_counter_willIncrement = 1'b0;
    if(_zz_190_)begin
      if(_zz_191_)begin
        memory_arbitration_haltItself = 1'b1;
        memory_MulDivIterativePlugin_div_counter_willIncrement = 1'b1;
      end
      _zz_72_ = memory_MulDivIterativePlugin_div_result;
    end
    case(memory_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
        _zz_72_ = _zz_156_;
      end
      `ShiftCtrlEnum_defaultEncoding_SRL_1, `ShiftCtrlEnum_defaultEncoding_SRA_1 : begin
        _zz_72_ = memory_SHIFT_RIGHT;
      end
      default : begin
      end
    endcase
  end

  assign memory_IS_CSR = execute_to_memory_IS_CSR;
  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_74_;
  assign execute_ENV_CTRL = _zz_75_;
  assign writeBack_ENV_CTRL = _zz_78_;
  always @ (*) begin
    _zz_79_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_79_ = writeBack_DBusSimplePlugin_rspFormated;
    end
    if((writeBack_arbitration_isValid && writeBack_IS_MUL))begin
      case(_zz_200_)
        2'b00 : begin
          _zz_79_ = _zz_245_;
        end
        default : begin
          _zz_79_ = _zz_246_;
        end
      endcase
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign memory_ALIGNEMENT_FAULT = execute_to_memory_ALIGNEMENT_FAULT;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_SRC_ADD = _zz_30_;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_ALIGNEMENT_FAULT = _zz_82_;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  always @ (*) begin
    _zz_83_ = memory_FORMAL_PC_NEXT;
    if(_zz_93_)begin
      _zz_83_ = _zz_94_;
    end
  end

  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  assign decode_PC = _zz_86_;
  assign decode_INSTRUCTION = _zz_85_;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    if((decode_arbitration_isValid && (_zz_157_ || _zz_158_)))begin
      decode_arbitration_haltItself = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((CsrPlugin_interrupt && decode_arbitration_isValid))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)) || (memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))))begin
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
    CsrPlugin_exceptionPortCtrl_exceptionValids_execute = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
    if(execute_exception_agregat_valid)begin
      decode_arbitration_flushAll = 1'b1;
      execute_arbitration_removeIt = 1'b1;
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b0;
    end
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_redoIt = 1'b0;
  always @ (*) begin
    execute_arbitration_haltItself = 1'b0;
    if((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_ALIGNEMENT_FAULT)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if((execute_arbitration_isValid && execute_IS_CSR))begin
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
  end

  assign execute_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    execute_arbitration_flushAll = 1'b0;
    if(memory_exception_agregat_valid)begin
      execute_arbitration_flushAll = 1'b1;
    end
    if(_zz_93_)begin
      execute_arbitration_flushAll = 1'b1;
    end
  end

  assign execute_arbitration_redoIt = 1'b0;
  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(memory_exception_agregat_valid)begin
      memory_arbitration_removeIt = 1'b1;
    end
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushAll = 1'b0;
    _zz_91_ = 1'b0;
    _zz_92_ = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    if(_zz_192_)begin
      _zz_91_ = 1'b1;
      _zz_92_ = {CsrPlugin_mtvec_base,(2'b00)};
      memory_arbitration_flushAll = 1'b1;
    end
    if(_zz_193_)begin
      _zz_92_ = CsrPlugin_mepc;
      _zz_91_ = 1'b1;
      memory_arbitration_flushAll = 1'b1;
    end
  end

  assign memory_arbitration_redoIt = 1'b0;
  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushAll = 1'b0;
  assign writeBack_arbitration_redoIt = 1'b0;
  always @ (*) begin
    _zz_88_ = 1'b0;
    if((((CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode || CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute) || CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory) || CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack))begin
      _zz_88_ = 1'b1;
    end
  end

  assign _zz_89_ = 1'b0;
  assign IBusSimplePlugin_jump_pcLoad_valid = (_zz_91_ || _zz_93_);
  assign _zz_96_ = {_zz_93_,_zz_91_};
  assign _zz_97_ = _zz_201_[1];
  assign IBusSimplePlugin_jump_pcLoad_payload = _zz_182_;
  assign _zz_98_ = (! _zz_88_);
  assign IBusSimplePlugin_fetchPc_output_valid = (IBusSimplePlugin_fetchPc_preOutput_valid && _zz_98_);
  assign IBusSimplePlugin_fetchPc_preOutput_ready = (IBusSimplePlugin_fetchPc_output_ready && _zz_98_);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_preOutput_payload;
  assign IBusSimplePlugin_fetchPc_propagatePc = 1'b0;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_204_);
    IBusSimplePlugin_fetchPc_samplePcNext = 1'b0;
    if(IBusSimplePlugin_fetchPc_propagatePc)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    if(_zz_194_)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
    IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
  end

  assign IBusSimplePlugin_fetchPc_preOutput_valid = _zz_99_;
  assign IBusSimplePlugin_fetchPc_preOutput_payload = IBusSimplePlugin_fetchPc_pc;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_inputSample = 1'b1;
  assign IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
  assign _zz_100_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_100_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_100_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
    if(((IBusSimplePlugin_cmd_valid && (! IBusSimplePlugin_cmd_ready)) || (IBusSimplePlugin_cmdFork_pendingFull && (! IBusSimplePlugin_cmdFork_cmdFired))))begin
      IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b1;
    end
  end

  assign _zz_101_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_101_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_101_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_2_halt = 1'b0;
  assign _zz_102_ = (! IBusSimplePlugin_iBusRsp_stages_2_halt);
  assign IBusSimplePlugin_iBusRsp_stages_2_input_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_ready && _zz_102_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_valid = (IBusSimplePlugin_iBusRsp_stages_2_input_valid && _zz_102_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_payload = IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = ((1'b0 && (! _zz_103_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_103_ = _zz_104_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_103_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = _zz_105_;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = ((1'b0 && (! _zz_106_)) || IBusSimplePlugin_iBusRsp_stages_2_input_ready);
  assign _zz_106_ = _zz_107_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_valid = _zz_106_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_payload = _zz_108_;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_109_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_110_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_111_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_112_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_113_;
  assign _zz_87_ = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw);
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  assign decode_arbitration_isValid = (IBusSimplePlugin_injector_decodeInput_valid && (! IBusSimplePlugin_injector_decodeRemoved));
  assign _zz_86_ = IBusSimplePlugin_injector_decodeInput_payload_pc;
  assign _zz_85_ = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  assign _zz_84_ = (decode_PC + (32'b00000000000000000000000000000100));
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pendingCmdNext = (_zz_205_ - _zz_209_);
  assign IBusSimplePlugin_cmdFork_pendingFull = (IBusSimplePlugin_pendingCmd == (3'b111));
  assign IBusSimplePlugin_cmd_valid = (((IBusSimplePlugin_iBusRsp_stages_1_input_valid || IBusSimplePlugin_cmdFork_cmdKeep) && (! IBusSimplePlugin_cmdFork_pendingFull)) && (! IBusSimplePlugin_cmdFork_cmdFired));
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_iBusRsp_stages_1_input_payload[31 : 2],(2'b00)};
  assign _zz_178_ = (iBus_rsp_valid && (! (IBusSimplePlugin_rspJoin_discardCounter != (3'b000))));
  assign _zz_179_ = (IBusSimplePlugin_jump_pcLoad_valid || _zz_89_);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_valid = _zz_186_;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error = _zz_187_;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst = _zz_188_;
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBufferOutput_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  assign IBusSimplePlugin_rspJoin_issueDetected = 1'b0;
  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_2_output_valid && IBusSimplePlugin_rspJoin_rspBufferOutput_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_2_output_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_114_ = (! IBusSimplePlugin_rspJoin_issueDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_inputBeforeStage_ready && _zz_114_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_114_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  assign _zz_82_ = (((dBus_cmd_payload_size == (2'b10)) && (dBus_cmd_payload_address[1 : 0] != (2'b00))) || ((dBus_cmd_payload_size == (2'b01)) && (dBus_cmd_payload_address[0 : 0] != (1'b0))));
  assign dBus_cmd_valid = ((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_removeIt)) && (! execute_ALIGNEMENT_FAULT));
  assign dBus_cmd_payload_wr = execute_INSTRUCTION[5];
  assign dBus_cmd_payload_address = execute_SRC_ADD;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_115_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_115_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_115_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_115_;
  assign _zz_81_ = dBus_cmd_payload_address[1 : 0];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_116_ = (4'b0001);
      end
      2'b01 : begin
        _zz_116_ = (4'b0011);
      end
      default : begin
        _zz_116_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_116_ <<< dBus_cmd_payload_address[1 : 0]);
  assign _zz_80_ = dBus_rsp_data;
  always @ (*) begin
    _zz_90_ = memory_ALIGNEMENT_FAULT;
    if((! (memory_arbitration_isValid && memory_MEMORY_ENABLE)))begin
      _zz_90_ = 1'b0;
    end
  end

  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_117_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_118_[31] = _zz_117_;
    _zz_118_[30] = _zz_117_;
    _zz_118_[29] = _zz_117_;
    _zz_118_[28] = _zz_117_;
    _zz_118_[27] = _zz_117_;
    _zz_118_[26] = _zz_117_;
    _zz_118_[25] = _zz_117_;
    _zz_118_[24] = _zz_117_;
    _zz_118_[23] = _zz_117_;
    _zz_118_[22] = _zz_117_;
    _zz_118_[21] = _zz_117_;
    _zz_118_[20] = _zz_117_;
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
    _zz_118_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_119_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_120_[31] = _zz_119_;
    _zz_120_[30] = _zz_119_;
    _zz_120_[29] = _zz_119_;
    _zz_120_[28] = _zz_119_;
    _zz_120_[27] = _zz_119_;
    _zz_120_[26] = _zz_119_;
    _zz_120_[25] = _zz_119_;
    _zz_120_[24] = _zz_119_;
    _zz_120_[23] = _zz_119_;
    _zz_120_[22] = _zz_119_;
    _zz_120_[21] = _zz_119_;
    _zz_120_[20] = _zz_119_;
    _zz_120_[19] = _zz_119_;
    _zz_120_[18] = _zz_119_;
    _zz_120_[17] = _zz_119_;
    _zz_120_[16] = _zz_119_;
    _zz_120_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_196_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_118_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_120_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = (26'b00000000000000000000000000);
  assign CsrPlugin_medeleg = (32'b00000000000000000000000000000000);
  assign CsrPlugin_mideleg = (32'b00000000000000000000000000000000);
  assign _zz_121_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_122_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_123_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  assign CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode = 1'b0;
  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege = CsrPlugin_privilege;
  assign memory_exception_agregat_valid = (_zz_90_ || _zz_95_);
  assign _zz_124_ = {_zz_95_,_zz_90_};
  assign _zz_125_ = _zz_217_[1];
  assign _zz_126_ = _zz_125_;
  assign memory_exception_agregat_payload_code = _zz_183_;
  assign memory_exception_agregat_payload_badAddr = _zz_184_;
  assign CsrPlugin_exceptionPortCtrl_exceptionValids_decode = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_memory = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
    if(memory_exception_agregat_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b1;
    end
    if(memory_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b0;
    end
  end

  assign CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  always @ (*) begin
    CsrPlugin_interrupt = 1'b0;
    CsrPlugin_interruptCode = (4'bxxxx);
    CsrPlugin_interruptTargetPrivilege = (2'bxx);
    if(CsrPlugin_mstatus_MIE)begin
      if(((_zz_121_ || _zz_122_) || _zz_123_))begin
        CsrPlugin_interrupt = 1'b1;
      end
      if(_zz_121_)begin
        CsrPlugin_interruptCode = (4'b0111);
        CsrPlugin_interruptTargetPrivilege = (2'b11);
      end
      if(_zz_122_)begin
        CsrPlugin_interruptCode = (4'b0011);
        CsrPlugin_interruptTargetPrivilege = (2'b11);
      end
      if(_zz_123_)begin
        CsrPlugin_interruptCode = (4'b1011);
        CsrPlugin_interruptTargetPrivilege = (2'b11);
      end
    end
    if((! 1'b1))begin
      CsrPlugin_interrupt = 1'b0;
    end
  end

  assign CsrPlugin_exception = (CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack && 1'b1);
  assign CsrPlugin_writeBackWasWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = ((! ((execute_arbitration_isValid || memory_arbitration_isValid) || writeBack_arbitration_isValid)) && IBusSimplePlugin_injector_nextPcCalc_3);
    if(((CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute || CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory) || CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack))begin
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

  assign contextSwitching = _zz_91_;
  assign _zz_77_ = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == (5'b00000))) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == (5'b00000)))));
  assign _zz_76_ = (decode_INSTRUCTION[13 : 7] != (7'b0100000));
  assign execute_CsrPlugin_blockedBySideEffects = (memory_arbitration_isValid || writeBack_arbitration_isValid);
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
      12'b001100000001 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
        execute_CsrPlugin_readData[31 : 30] = CsrPlugin_misa_base;
        execute_CsrPlugin_readData[25 : 0] = CsrPlugin_misa_extensions;
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
    execute_exception_agregat_valid = (execute_CsrPlugin_illegalAccess || execute_CsrPlugin_illegalInstruction);
    execute_exception_agregat_payload_code = (4'b0010);
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_ECALL)))begin
      execute_exception_agregat_valid = 1'b1;
      execute_exception_agregat_payload_code = (4'b1011);
    end
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_EBREAK)))begin
      execute_exception_agregat_valid = 1'b1;
      execute_exception_agregat_payload_code = (4'b0011);
    end
  end

  assign execute_exception_agregat_payload_badAddr = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
  assign execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
  assign execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
  assign execute_CsrPlugin_writeEnable = (execute_CsrPlugin_writeInstruction && (! execute_arbitration_isStuck));
  assign execute_CsrPlugin_readEnable = (execute_CsrPlugin_readInstruction && (! execute_arbitration_isStuck));
  always @ (*) begin
    case(_zz_198_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readData & (~ execute_SRC1)) : (execute_CsrPlugin_readData | execute_SRC1));
      end
    endcase
  end

  assign _zz_73_ = execute_CsrPlugin_readData;
  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign _zz_128_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000100000)) == (32'b00000000000000000000000000000000));
  assign _zz_129_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000000100)) == (32'b00000000000000000000000000000100));
  assign _zz_130_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001010000)) == (32'b00000000000000000100000001010000));
  assign _zz_131_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000010100)) == (32'b00000000000000000000000000000100));
  assign _zz_132_ = ((decode_INSTRUCTION & (32'b00000000000000000001000000000000)) == (32'b00000000000000000000000000000000));
  assign _zz_127_ = {(((decode_INSTRUCTION & _zz_284_) == (32'b00000000000000000000000001010000)) != (1'b0)),{({_zz_285_,_zz_286_} != (2'b00)),{(_zz_132_ != (1'b0)),{(_zz_287_ != _zz_288_),{_zz_289_,{_zz_290_,_zz_291_}}}}}};
  assign _zz_71_ = _zz_221_[0];
  assign _zz_70_ = _zz_222_[0];
  assign _zz_69_ = _zz_223_[0];
  assign _zz_133_ = _zz_127_[4 : 3];
  assign _zz_68_ = _zz_133_;
  assign _zz_67_ = _zz_224_[0];
  assign _zz_66_ = _zz_225_[0];
  assign _zz_65_ = _zz_226_[0];
  assign _zz_134_ = _zz_127_[10 : 9];
  assign _zz_64_ = _zz_134_;
  assign _zz_135_ = _zz_127_[12 : 11];
  assign _zz_63_ = _zz_135_;
  assign _zz_62_ = _zz_227_[0];
  assign _zz_136_ = _zz_127_[15 : 14];
  assign _zz_61_ = _zz_136_;
  assign _zz_60_ = _zz_228_[0];
  assign _zz_59_ = _zz_229_[0];
  assign _zz_137_ = _zz_127_[19 : 18];
  assign _zz_58_ = _zz_137_;
  assign _zz_57_ = _zz_230_[0];
  assign _zz_138_ = _zz_127_[22 : 21];
  assign _zz_56_ = _zz_138_;
  assign _zz_55_ = _zz_231_[0];
  assign _zz_54_ = _zz_232_[0];
  assign _zz_53_ = _zz_233_[0];
  assign _zz_139_ = _zz_127_[27 : 26];
  assign _zz_52_ = _zz_139_;
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_180_;
  assign decode_RegFilePlugin_rs2Data = _zz_181_;
  assign _zz_51_ = decode_RegFilePlugin_rs1Data;
  assign _zz_50_ = decode_RegFilePlugin_rs2Data;
  always @ (*) begin
    writeBack_RegFilePlugin_regFileWrite_valid = (_zz_48_ && writeBack_arbitration_isFiring);
    if(_zz_140_)begin
      writeBack_RegFilePlugin_regFileWrite_valid = 1'b1;
    end
  end

  assign writeBack_RegFilePlugin_regFileWrite_payload_address = _zz_47_[11 : 7];
  assign writeBack_RegFilePlugin_regFileWrite_payload_data = _zz_79_;
  assign execute_MulPlugin_a = execute_SRC1;
  assign execute_MulPlugin_b = execute_SRC2;
  always @ (*) begin
    case(_zz_199_)
      2'b01 : begin
        execute_MulPlugin_aSigned = 1'b1;
        execute_MulPlugin_bSigned = 1'b1;
      end
      2'b10 : begin
        execute_MulPlugin_aSigned = 1'b1;
        execute_MulPlugin_bSigned = 1'b0;
      end
      default : begin
        execute_MulPlugin_aSigned = 1'b0;
        execute_MulPlugin_bSigned = 1'b0;
      end
    endcase
  end

  assign execute_MulPlugin_aULow = execute_MulPlugin_a[15 : 0];
  assign execute_MulPlugin_bULow = execute_MulPlugin_b[15 : 0];
  assign execute_MulPlugin_aSLow = {1'b0,execute_MulPlugin_a[15 : 0]};
  assign execute_MulPlugin_bSLow = {1'b0,execute_MulPlugin_b[15 : 0]};
  assign execute_MulPlugin_aHigh = {(execute_MulPlugin_aSigned && execute_MulPlugin_a[31]),execute_MulPlugin_a[31 : 16]};
  assign execute_MulPlugin_bHigh = {(execute_MulPlugin_bSigned && execute_MulPlugin_b[31]),execute_MulPlugin_b[31 : 16]};
  assign _zz_46_ = (execute_MulPlugin_aULow * execute_MulPlugin_bULow);
  assign _zz_45_ = ($signed(execute_MulPlugin_aSLow) * $signed(execute_MulPlugin_bHigh));
  assign _zz_44_ = ($signed(execute_MulPlugin_aHigh) * $signed(execute_MulPlugin_bSLow));
  assign _zz_43_ = ($signed(execute_MulPlugin_aHigh) * $signed(execute_MulPlugin_bHigh));
  assign _zz_42_ = ($signed(_zz_234_) + $signed(_zz_242_));
  assign writeBack_MulPlugin_result = ($signed(_zz_243_) + $signed(_zz_244_));
  always @ (*) begin
    memory_MulDivIterativePlugin_div_counter_willClear = 1'b0;
    if(_zz_195_)begin
      memory_MulDivIterativePlugin_div_counter_willClear = 1'b1;
    end
  end

  assign memory_MulDivIterativePlugin_div_counter_willOverflowIfInc = (memory_MulDivIterativePlugin_div_counter_value == (6'b100001));
  assign memory_MulDivIterativePlugin_div_counter_willOverflow = (memory_MulDivIterativePlugin_div_counter_willOverflowIfInc && memory_MulDivIterativePlugin_div_counter_willIncrement);
  always @ (*) begin
    if(memory_MulDivIterativePlugin_div_counter_willOverflow)begin
      memory_MulDivIterativePlugin_div_counter_valueNext = (6'b000000);
    end else begin
      memory_MulDivIterativePlugin_div_counter_valueNext = (memory_MulDivIterativePlugin_div_counter_value + _zz_248_);
    end
    if(memory_MulDivIterativePlugin_div_counter_willClear)begin
      memory_MulDivIterativePlugin_div_counter_valueNext = (6'b000000);
    end
  end

  assign _zz_141_ = memory_MulDivIterativePlugin_rs1[31 : 0];
  assign _zz_142_ = {memory_MulDivIterativePlugin_accumulator[31 : 0],_zz_141_[31]};
  assign _zz_143_ = (_zz_142_ - _zz_249_);
  assign _zz_144_ = (memory_INSTRUCTION[13] ? memory_MulDivIterativePlugin_accumulator[31 : 0] : memory_MulDivIterativePlugin_rs1[31 : 0]);
  assign _zz_145_ = (execute_RS2[31] && execute_IS_RS2_SIGNED);
  assign _zz_146_ = (1'b0 || ((execute_IS_DIV && execute_RS1[31]) && execute_IS_RS1_SIGNED));
  always @ (*) begin
    _zz_147_[32] = (execute_IS_RS1_SIGNED && execute_RS1[31]);
    _zz_147_[31 : 0] = execute_RS1;
  end

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
        _zz_148_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_148_ = {31'd0, _zz_262_};
      end
      default : begin
        _zz_148_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  assign _zz_40_ = _zz_148_;
  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_149_ = _zz_36_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_149_ = {29'd0, _zz_263_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_149_ = {decode_INSTRUCTION[31 : 12],(12'b000000000000)};
      end
      default : begin
        _zz_149_ = {27'd0, _zz_264_};
      end
    endcase
  end

  assign _zz_38_ = _zz_149_;
  assign _zz_150_ = _zz_265_[11];
  always @ (*) begin
    _zz_151_[19] = _zz_150_;
    _zz_151_[18] = _zz_150_;
    _zz_151_[17] = _zz_150_;
    _zz_151_[16] = _zz_150_;
    _zz_151_[15] = _zz_150_;
    _zz_151_[14] = _zz_150_;
    _zz_151_[13] = _zz_150_;
    _zz_151_[12] = _zz_150_;
    _zz_151_[11] = _zz_150_;
    _zz_151_[10] = _zz_150_;
    _zz_151_[9] = _zz_150_;
    _zz_151_[8] = _zz_150_;
    _zz_151_[7] = _zz_150_;
    _zz_151_[6] = _zz_150_;
    _zz_151_[5] = _zz_150_;
    _zz_151_[4] = _zz_150_;
    _zz_151_[3] = _zz_150_;
    _zz_151_[2] = _zz_150_;
    _zz_151_[1] = _zz_150_;
    _zz_151_[0] = _zz_150_;
  end

  assign _zz_152_ = _zz_266_[11];
  always @ (*) begin
    _zz_153_[19] = _zz_152_;
    _zz_153_[18] = _zz_152_;
    _zz_153_[17] = _zz_152_;
    _zz_153_[16] = _zz_152_;
    _zz_153_[15] = _zz_152_;
    _zz_153_[14] = _zz_152_;
    _zz_153_[13] = _zz_152_;
    _zz_153_[12] = _zz_152_;
    _zz_153_[11] = _zz_152_;
    _zz_153_[10] = _zz_152_;
    _zz_153_[9] = _zz_152_;
    _zz_153_[8] = _zz_152_;
    _zz_153_[7] = _zz_152_;
    _zz_153_[6] = _zz_152_;
    _zz_153_[5] = _zz_152_;
    _zz_153_[4] = _zz_152_;
    _zz_153_[3] = _zz_152_;
    _zz_153_[2] = _zz_152_;
    _zz_153_[1] = _zz_152_;
    _zz_153_[0] = _zz_152_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_154_ = _zz_33_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_154_ = {_zz_151_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_154_ = {_zz_153_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_154_ = _zz_32_;
      end
    endcase
  end

  assign _zz_35_ = _zz_154_;
  assign execute_SrcPlugin_add = _zz_267_;
  assign execute_SrcPlugin_sub = _zz_268_;
  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_sub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign _zz_31_ = (execute_SRC_USE_SUB_LESS ? execute_SrcPlugin_sub : execute_SrcPlugin_add);
  assign _zz_30_ = execute_SrcPlugin_add;
  assign _zz_29_ = execute_SrcPlugin_less;
  assign execute_FullBarrelShifterPlugin_amplitude = execute_SRC2[4 : 0];
  always @ (*) begin
    _zz_155_[0] = execute_SRC1[31];
    _zz_155_[1] = execute_SRC1[30];
    _zz_155_[2] = execute_SRC1[29];
    _zz_155_[3] = execute_SRC1[28];
    _zz_155_[4] = execute_SRC1[27];
    _zz_155_[5] = execute_SRC1[26];
    _zz_155_[6] = execute_SRC1[25];
    _zz_155_[7] = execute_SRC1[24];
    _zz_155_[8] = execute_SRC1[23];
    _zz_155_[9] = execute_SRC1[22];
    _zz_155_[10] = execute_SRC1[21];
    _zz_155_[11] = execute_SRC1[20];
    _zz_155_[12] = execute_SRC1[19];
    _zz_155_[13] = execute_SRC1[18];
    _zz_155_[14] = execute_SRC1[17];
    _zz_155_[15] = execute_SRC1[16];
    _zz_155_[16] = execute_SRC1[15];
    _zz_155_[17] = execute_SRC1[14];
    _zz_155_[18] = execute_SRC1[13];
    _zz_155_[19] = execute_SRC1[12];
    _zz_155_[20] = execute_SRC1[11];
    _zz_155_[21] = execute_SRC1[10];
    _zz_155_[22] = execute_SRC1[9];
    _zz_155_[23] = execute_SRC1[8];
    _zz_155_[24] = execute_SRC1[7];
    _zz_155_[25] = execute_SRC1[6];
    _zz_155_[26] = execute_SRC1[5];
    _zz_155_[27] = execute_SRC1[4];
    _zz_155_[28] = execute_SRC1[3];
    _zz_155_[29] = execute_SRC1[2];
    _zz_155_[30] = execute_SRC1[1];
    _zz_155_[31] = execute_SRC1[0];
  end

  assign execute_FullBarrelShifterPlugin_reversed = ((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SLL_1) ? _zz_155_ : execute_SRC1);
  assign _zz_27_ = _zz_270_;
  always @ (*) begin
    _zz_156_[0] = memory_SHIFT_RIGHT[31];
    _zz_156_[1] = memory_SHIFT_RIGHT[30];
    _zz_156_[2] = memory_SHIFT_RIGHT[29];
    _zz_156_[3] = memory_SHIFT_RIGHT[28];
    _zz_156_[4] = memory_SHIFT_RIGHT[27];
    _zz_156_[5] = memory_SHIFT_RIGHT[26];
    _zz_156_[6] = memory_SHIFT_RIGHT[25];
    _zz_156_[7] = memory_SHIFT_RIGHT[24];
    _zz_156_[8] = memory_SHIFT_RIGHT[23];
    _zz_156_[9] = memory_SHIFT_RIGHT[22];
    _zz_156_[10] = memory_SHIFT_RIGHT[21];
    _zz_156_[11] = memory_SHIFT_RIGHT[20];
    _zz_156_[12] = memory_SHIFT_RIGHT[19];
    _zz_156_[13] = memory_SHIFT_RIGHT[18];
    _zz_156_[14] = memory_SHIFT_RIGHT[17];
    _zz_156_[15] = memory_SHIFT_RIGHT[16];
    _zz_156_[16] = memory_SHIFT_RIGHT[15];
    _zz_156_[17] = memory_SHIFT_RIGHT[14];
    _zz_156_[18] = memory_SHIFT_RIGHT[13];
    _zz_156_[19] = memory_SHIFT_RIGHT[12];
    _zz_156_[20] = memory_SHIFT_RIGHT[11];
    _zz_156_[21] = memory_SHIFT_RIGHT[10];
    _zz_156_[22] = memory_SHIFT_RIGHT[9];
    _zz_156_[23] = memory_SHIFT_RIGHT[8];
    _zz_156_[24] = memory_SHIFT_RIGHT[7];
    _zz_156_[25] = memory_SHIFT_RIGHT[6];
    _zz_156_[26] = memory_SHIFT_RIGHT[5];
    _zz_156_[27] = memory_SHIFT_RIGHT[4];
    _zz_156_[28] = memory_SHIFT_RIGHT[3];
    _zz_156_[29] = memory_SHIFT_RIGHT[2];
    _zz_156_[30] = memory_SHIFT_RIGHT[1];
    _zz_156_[31] = memory_SHIFT_RIGHT[0];
  end

  always @ (*) begin
    _zz_157_ = 1'b0;
    _zz_158_ = 1'b0;
    if((writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID))begin
      if((1'b0 || (! 1'b1)))begin
        if(_zz_162_)begin
          _zz_157_ = 1'b1;
        end
        if(_zz_163_)begin
          _zz_158_ = 1'b1;
        end
      end
    end
    if((memory_arbitration_isValid && memory_REGFILE_WRITE_VALID))begin
      if((1'b0 || (! memory_BYPASSABLE_MEMORY_STAGE)))begin
        if(_zz_164_)begin
          _zz_157_ = 1'b1;
        end
        if(_zz_165_)begin
          _zz_158_ = 1'b1;
        end
      end
    end
    if((execute_arbitration_isValid && execute_REGFILE_WRITE_VALID))begin
      if((1'b0 || (! execute_BYPASSABLE_EXECUTE_STAGE)))begin
        if(_zz_166_)begin
          _zz_157_ = 1'b1;
        end
        if(_zz_167_)begin
          _zz_158_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_157_ = 1'b0;
    end
    if((! decode_RS2_USE))begin
      _zz_158_ = 1'b0;
    end
  end

  assign _zz_162_ = (writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_163_ = (writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign _zz_164_ = (memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_165_ = (memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign _zz_166_ = (execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_167_ = (execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_168_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_168_ == (3'b000))) begin
        _zz_169_ = execute_BranchPlugin_eq;
    end else if((_zz_168_ == (3'b001))) begin
        _zz_169_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_168_ & (3'b101)) == (3'b101)))) begin
        _zz_169_ = (! execute_SRC_LESS);
    end else begin
        _zz_169_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_170_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_170_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_170_ = 1'b1;
      end
      default : begin
        _zz_170_ = _zz_169_;
      end
    endcase
  end

  assign _zz_24_ = _zz_170_;
  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_171_ = _zz_272_[19];
  always @ (*) begin
    _zz_172_[10] = _zz_171_;
    _zz_172_[9] = _zz_171_;
    _zz_172_[8] = _zz_171_;
    _zz_172_[7] = _zz_171_;
    _zz_172_[6] = _zz_171_;
    _zz_172_[5] = _zz_171_;
    _zz_172_[4] = _zz_171_;
    _zz_172_[3] = _zz_171_;
    _zz_172_[2] = _zz_171_;
    _zz_172_[1] = _zz_171_;
    _zz_172_[0] = _zz_171_;
  end

  assign _zz_173_ = _zz_273_[11];
  always @ (*) begin
    _zz_174_[19] = _zz_173_;
    _zz_174_[18] = _zz_173_;
    _zz_174_[17] = _zz_173_;
    _zz_174_[16] = _zz_173_;
    _zz_174_[15] = _zz_173_;
    _zz_174_[14] = _zz_173_;
    _zz_174_[13] = _zz_173_;
    _zz_174_[12] = _zz_173_;
    _zz_174_[11] = _zz_173_;
    _zz_174_[10] = _zz_173_;
    _zz_174_[9] = _zz_173_;
    _zz_174_[8] = _zz_173_;
    _zz_174_[7] = _zz_173_;
    _zz_174_[6] = _zz_173_;
    _zz_174_[5] = _zz_173_;
    _zz_174_[4] = _zz_173_;
    _zz_174_[3] = _zz_173_;
    _zz_174_[2] = _zz_173_;
    _zz_174_[1] = _zz_173_;
    _zz_174_[0] = _zz_173_;
  end

  assign _zz_175_ = _zz_274_[11];
  always @ (*) begin
    _zz_176_[18] = _zz_175_;
    _zz_176_[17] = _zz_175_;
    _zz_176_[16] = _zz_175_;
    _zz_176_[15] = _zz_175_;
    _zz_176_[14] = _zz_175_;
    _zz_176_[13] = _zz_175_;
    _zz_176_[12] = _zz_175_;
    _zz_176_[11] = _zz_175_;
    _zz_176_[10] = _zz_175_;
    _zz_176_[9] = _zz_175_;
    _zz_176_[8] = _zz_175_;
    _zz_176_[7] = _zz_175_;
    _zz_176_[6] = _zz_175_;
    _zz_176_[5] = _zz_175_;
    _zz_176_[4] = _zz_175_;
    _zz_176_[3] = _zz_175_;
    _zz_176_[2] = _zz_175_;
    _zz_176_[1] = _zz_175_;
    _zz_176_[0] = _zz_175_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_177_ = {{_zz_172_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_177_ = {_zz_174_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_177_ = {{_zz_176_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_177_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign _zz_22_ = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign _zz_93_ = (memory_arbitration_isFiring && memory_BRANCH_DO);
  assign _zz_94_ = memory_BRANCH_CALC;
  assign _zz_95_ = ((memory_arbitration_isValid && memory_BRANCH_DO) && _zz_94_[1]);
  assign _zz_21_ = decode_ALU_BITWISE_CTRL;
  assign _zz_19_ = _zz_64_;
  assign _zz_41_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_18_ = decode_BRANCH_CTRL;
  assign _zz_16_ = _zz_58_;
  assign _zz_23_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_37_ = _zz_63_;
  assign _zz_34_ = _zz_68_;
  assign _zz_15_ = decode_ENV_CTRL;
  assign _zz_12_ = execute_ENV_CTRL;
  assign _zz_10_ = memory_ENV_CTRL;
  assign _zz_13_ = _zz_52_;
  assign _zz_75_ = decode_to_execute_ENV_CTRL;
  assign _zz_74_ = execute_to_memory_ENV_CTRL;
  assign _zz_78_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_8_ = decode_SHIFT_CTRL;
  assign _zz_5_ = execute_SHIFT_CTRL;
  assign _zz_6_ = _zz_61_;
  assign _zz_28_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_26_ = execute_to_memory_SHIFT_CTRL;
  assign _zz_3_ = decode_ALU_CTRL;
  assign _zz_1_ = _zz_56_;
  assign _zz_39_ = decode_to_execute_ALU_CTRL;
  assign decode_arbitration_isFlushed = (((decode_arbitration_flushAll || execute_arbitration_flushAll) || memory_arbitration_flushAll) || writeBack_arbitration_flushAll);
  assign execute_arbitration_isFlushed = ((execute_arbitration_flushAll || memory_arbitration_flushAll) || writeBack_arbitration_flushAll);
  assign memory_arbitration_isFlushed = (memory_arbitration_flushAll || writeBack_arbitration_flushAll);
  assign writeBack_arbitration_isFlushed = writeBack_arbitration_flushAll;
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      CsrPlugin_privilege <= (2'b11);
      IBusSimplePlugin_fetchPc_pcReg <= (32'b00000000000000100000000000000000);
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      _zz_99_ <= 1'b0;
      _zz_104_ <= 1'b0;
      _zz_107_ <= 1'b0;
      _zz_109_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_3 <= 1'b0;
      IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      IBusSimplePlugin_pendingCmd <= (3'b000);
      IBusSimplePlugin_cmdFork_cmdKeep <= 1'b0;
      IBusSimplePlugin_cmdFork_cmdFired <= 1'b0;
      IBusSimplePlugin_rspJoin_discardCounter <= (3'b000);
      CsrPlugin_mtvec_mode <= (2'b00);
      CsrPlugin_mtvec_base <= (30'b100000000000000000000000001000);
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
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      _zz_140_ <= 1'b1;
      memory_MulDivIterativePlugin_div_counter_value <= (6'b000000);
      _zz_159_ <= 1'b0;
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      memory_to_writeBack_REGFILE_WRITE_DATA <= (32'b00000000000000000000000000000000);
      memory_to_writeBack_INSTRUCTION <= (32'b00000000000000000000000000000000);
    end else begin
      if(IBusSimplePlugin_fetchPc_propagatePc)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(IBusSimplePlugin_jump_pcLoad_valid)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(_zz_194_)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(IBusSimplePlugin_fetchPc_samplePcNext)begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      _zz_99_ <= 1'b1;
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        _zz_104_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
        _zz_104_ <= IBusSimplePlugin_iBusRsp_stages_0_output_valid;
      end
      if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
        _zz_107_ <= IBusSimplePlugin_iBusRsp_stages_1_output_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        _zz_107_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
        _zz_109_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        _zz_109_ <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_1_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_2_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_injector_decodeInput_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_0 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= IBusSimplePlugin_injector_nextPcCalc_0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_1 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_2 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_2 <= IBusSimplePlugin_injector_nextPcCalc_1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_2 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_3 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_3 <= IBusSimplePlugin_injector_nextPcCalc_2;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_nextPcCalc_3 <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      end
      IBusSimplePlugin_pendingCmd <= IBusSimplePlugin_pendingCmdNext;
      if(IBusSimplePlugin_cmd_valid)begin
        IBusSimplePlugin_cmdFork_cmdKeep <= 1'b1;
      end
      if(IBusSimplePlugin_cmd_ready)begin
        IBusSimplePlugin_cmdFork_cmdKeep <= 1'b0;
      end
      if((IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready))begin
        IBusSimplePlugin_cmdFork_cmdFired <= 1'b1;
      end
      if(IBusSimplePlugin_iBusRsp_stages_1_input_ready)begin
        IBusSimplePlugin_cmdFork_cmdFired <= 1'b0;
      end
      IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_rspJoin_discardCounter - _zz_211_);
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_89_))begin
        IBusSimplePlugin_rspJoin_discardCounter <= (_zz_212_ - _zz_216_);
      end
      CsrPlugin_mip_MEIP <= externalInterrupt;
      CsrPlugin_mip_MTIP <= timerInterrupt;
      if((! execute_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
      end
      if((! memory_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= (CsrPlugin_exceptionPortCtrl_exceptionValids_execute && (! execute_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
      end
      if((! writeBack_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= (CsrPlugin_exceptionPortCtrl_exceptionValids_memory && (! memory_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      end
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_192_)begin
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
      if(_zz_193_)begin
        case(_zz_197_)
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
      _zz_140_ <= 1'b0;
      memory_MulDivIterativePlugin_div_counter_value <= memory_MulDivIterativePlugin_div_counter_valueNext;
      _zz_159_ <= (_zz_48_ && writeBack_arbitration_isFiring);
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= _zz_72_;
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(execute_CsrPlugin_csrAddress)
        12'b001100000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
            CsrPlugin_mstatus_MPIE <= _zz_275_[0];
            CsrPlugin_mstatus_MIE <= _zz_276_[0];
          end
        end
        12'b001101000001 : begin
        end
        12'b001100000101 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mtvec_base <= execute_CsrPlugin_writeData[31 : 2];
            CsrPlugin_mtvec_mode <= execute_CsrPlugin_writeData[1 : 0];
          end
        end
        12'b001101000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mip_MSIP <= _zz_277_[0];
          end
        end
        12'b001101000011 : begin
        end
        12'b001101000000 : begin
        end
        12'b001100000001 : begin
        end
        12'b001100000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mie_MEIE <= _zz_278_[0];
            CsrPlugin_mie_MTIE <= _zz_279_[0];
            CsrPlugin_mie_MSIE <= _zz_280_[0];
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
    if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
      _zz_105_ <= IBusSimplePlugin_iBusRsp_stages_0_output_payload;
    end
    if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
      _zz_108_ <= IBusSimplePlugin_iBusRsp_stages_1_output_payload;
    end
    if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
      _zz_110_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
      _zz_111_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
      _zz_112_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw;
      _zz_113_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_raw;
    end
    if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow memory stage stall when read happend");
    end
    if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_INSTRUCTION[5])) && writeBack_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow writeback stage stall when read happend");
    end
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
    if(execute_exception_agregat_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= execute_exception_agregat_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= execute_exception_agregat_payload_badAddr;
    end
    if(memory_exception_agregat_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= memory_exception_agregat_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= memory_exception_agregat_payload_badAddr;
    end
    if((CsrPlugin_exception || CsrPlugin_interruptJump))begin
      case(CsrPlugin_privilege)
        2'b11 : begin
          CsrPlugin_mepc <= writeBack_PC;
        end
        default : begin
        end
      endcase
    end
    if(_zz_192_)begin
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
    if((memory_MulDivIterativePlugin_div_counter_value == (6'b100000)))begin
      memory_MulDivIterativePlugin_div_done <= 1'b1;
    end
    if((! memory_arbitration_isStuck))begin
      memory_MulDivIterativePlugin_div_done <= 1'b0;
    end
    if(_zz_190_)begin
      if(_zz_191_)begin
        memory_MulDivIterativePlugin_rs1[31 : 0] <= _zz_250_[31:0];
        memory_MulDivIterativePlugin_accumulator[31 : 0] <= ((! _zz_143_[32]) ? _zz_251_ : _zz_252_);
        if((memory_MulDivIterativePlugin_div_counter_value == (6'b100000)))begin
          memory_MulDivIterativePlugin_div_result <= _zz_253_[31:0];
        end
      end
    end
    if(_zz_195_)begin
      memory_MulDivIterativePlugin_accumulator <= (65'b00000000000000000000000000000000000000000000000000000000000000000);
      memory_MulDivIterativePlugin_rs1 <= ((_zz_146_ ? (~ _zz_147_) : _zz_147_) + _zz_259_);
      memory_MulDivIterativePlugin_rs2 <= ((_zz_145_ ? (~ execute_RS2) : execute_RS2) + _zz_261_);
      memory_MulDivIterativePlugin_div_needRevert <= ((_zz_146_ ^ (_zz_145_ && (! execute_INSTRUCTION[13]))) && (! (((execute_RS2 == (32'b00000000000000000000000000000000)) && execute_IS_RS2_SIGNED) && (! execute_INSTRUCTION[13]))));
    end
    _zz_160_ <= _zz_47_[11 : 7];
    _zz_161_ <= _zz_79_;
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_DIV <= decode_IS_DIV;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_DIV <= execute_IS_DIV;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_CSR <= execute_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RS2_SIGNED <= decode_IS_RS2_SIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_20_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_32_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_MUL <= decode_IS_MUL;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_MUL <= execute_IS_MUL;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_IS_MUL <= memory_IS_MUL;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_LH <= execute_MUL_LH;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MUL_LOW <= memory_MUL_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_83_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_17_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_HH <= execute_MUL_HH;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MUL_HH <= memory_MUL_HH;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_33_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_HL <= execute_MUL_HL;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RS1_SIGNED <= decode_IS_RS1_SIGNED;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_25_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_36_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_LL <= execute_MUL_LL;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_RIGHT <= execute_SHIFT_RIGHT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_14_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_11_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_9_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ALIGNEMENT_FAULT <= execute_ALIGNEMENT_FAULT;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_7_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_CTRL <= _zz_4_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_2_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PIPELINED_CSR_READ <= execute_PIPELINED_CSR_READ;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
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
      12'b001100000001 : begin
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

module SerialRxOutput (
      input   io_serialRx,
      output [7:0] io_output,
      input   io_clk,
      input   resetCtrl_progReset);
  wire  _zz_1_;
  wire  _zz_2_;
  reg [8:0] timer;
  wire  timerTick;
  reg [1:0] state;
  reg [2:0] bitCounter;
  wire  serialRx;
  reg [7:0] outputReg;
  assign _zz_2_ = (! serialRx);
  BufferCC bufferCC_2_ ( 
    .io_dataIn(io_serialRx),
    .io_dataOut(_zz_1_),
    .io_clk(io_clk),
    .resetCtrl_progReset(resetCtrl_progReset) 
  );
  assign timerTick = (timer == (9'b000000000));
  assign serialRx = _zz_1_;
  assign io_output = outputReg;
  always @ (posedge io_clk) begin
    timer <= (timer - (9'b000000001));
    case(state)
      2'b00 : begin
        if(_zz_2_)begin
          timer <= (9'b101000100);
        end
      end
      2'b01 : begin
        if(timerTick)begin
          timer <= (9'b011011000);
          bitCounter <= (bitCounter + (3'b001));
        end
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (posedge io_clk or posedge resetCtrl_progReset) begin
    if (resetCtrl_progReset) begin
      state <= (2'b00);
      outputReg <= (8'b00000111);
    end else begin
      case(state)
        2'b00 : begin
          if(_zz_2_)begin
            state <= (2'b01);
          end
        end
        2'b01 : begin
          if(timerTick)begin
            outputReg[bitCounter] <= serialRx;
            if((bitCounter == (3'b111)))begin
              state <= (2'b10);
            end
          end
        end
        2'b10 : begin
          if(timerTick)begin
            state <= (2'b00);
          end
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module SimpleBusDecoder (
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
      input   io_clk,
      input   resetCtrl_systemReset);
  reg [31:0] _zz_3_;
  wire [19:0] _zz_4_;
  wire [2:0] _zz_5_;
  wire [0:0] _zz_6_;
  wire [2:0] _zz_7_;
  wire [0:0] _zz_8_;
  wire [2:0] _zz_9_;
  wire [0:0] _zz_10_;
  wire  hits_0;
  wire  hits_1;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  noHit;
  reg [2:0] rspPendingCounter;
  reg  rspHits_0;
  reg  rspHits_1;
  wire  rspPending;
  wire  rspNoHit;
  wire  cmdWait;
  assign _zz_4_ = (20'b11111100000000000000);
  assign _zz_5_ = (rspPendingCounter + _zz_7_);
  assign _zz_6_ = ((io_input_cmd_valid && io_input_cmd_ready) && (! io_input_cmd_payload_wr));
  assign _zz_7_ = {2'd0, _zz_6_};
  assign _zz_8_ = io_input_rsp_valid;
  assign _zz_9_ = {2'd0, _zz_8_};
  assign _zz_10_ = rspHits_1;
  always @(*) begin
    case(_zz_10_)
      1'b0 : begin
        _zz_3_ = io_outputs_0_rsp_0_data;
      end
      default : begin
        _zz_3_ = io_outputs_1_rsp_1_data;
      end
    endcase
  end

  assign hits_0 = ((io_input_cmd_payload_address & _zz_4_) == (20'b10010000000000000000));
  always @ (*) begin
    io_outputs_0_cmd_valid = (io_input_cmd_valid && hits_0);
    io_outputs_1_cmd_valid = (io_input_cmd_valid && hits_1);
    io_input_cmd_ready = (((hits_0 && io_outputs_0_cmd_ready) || (hits_1 && io_outputs_1_cmd_ready)) || noHit);
    if(cmdWait)begin
      io_input_cmd_ready = 1'b0;
      io_outputs_0_cmd_valid = 1'b0;
      io_outputs_1_cmd_valid = 1'b0;
    end
  end

  assign _zz_1_ = io_input_cmd_payload_wr;
  assign io_outputs_0_cmd_payload_wr = _zz_1_;
  assign io_outputs_0_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_0_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_0_cmd_payload_mask = io_input_cmd_payload_mask;
  assign hits_1 = (! hits_0);
  assign _zz_2_ = io_input_cmd_payload_wr;
  assign io_outputs_1_cmd_payload_wr = _zz_2_;
  assign io_outputs_1_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_1_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_1_cmd_payload_mask = io_input_cmd_payload_mask;
  assign noHit = (! (hits_0 || hits_1));
  assign rspPending = (rspPendingCounter != (3'b000));
  assign rspNoHit = (! (rspHits_0 || rspHits_1));
  assign io_input_rsp_valid = ((io_outputs_0_rsp_valid || io_outputs_1_rsp_valid) || (rspPending && rspNoHit));
  assign io_input_rsp_payload_data = _zz_3_;
  assign cmdWait = (((io_input_cmd_valid && rspPending) && ((hits_0 != rspHits_0) || (hits_1 != rspHits_1))) || (rspPendingCounter == (3'b111)));
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      rspPendingCounter <= (3'b000);
    end else begin
      rspPendingCounter <= (_zz_5_ - _zz_9_);
    end
  end

  always @ (posedge io_clk) begin
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      rspHits_0 <= hits_0;
      rspHits_1 <= hits_1;
    end
  end

endmodule

module SimpleBusDecoder_1_ (
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
      input   io_clk,
      input   resetCtrl_systemReset);
  reg [31:0] _zz_3_;
  wire [19:0] _zz_4_;
  wire [2:0] _zz_5_;
  wire [0:0] _zz_6_;
  wire [2:0] _zz_7_;
  wire [0:0] _zz_8_;
  wire [2:0] _zz_9_;
  wire [0:0] _zz_10_;
  wire  hits_0;
  wire  hits_1;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  noHit;
  reg [2:0] rspPendingCounter;
  reg  rspHits_0;
  reg  rspHits_1;
  wire  rspPending;
  wire  rspNoHit;
  wire  cmdWait;
  assign _zz_4_ = (20'b11111000000000000000);
  assign _zz_5_ = (rspPendingCounter + _zz_7_);
  assign _zz_6_ = ((io_input_cmd_valid && io_input_cmd_ready) && (! io_input_cmd_payload_wr));
  assign _zz_7_ = {2'd0, _zz_6_};
  assign _zz_8_ = io_input_rsp_valid;
  assign _zz_9_ = {2'd0, _zz_8_};
  assign _zz_10_ = rspHits_1;
  always @(*) begin
    case(_zz_10_)
      1'b0 : begin
        _zz_3_ = io_outputs_0_rsp_0_data;
      end
      default : begin
        _zz_3_ = io_outputs_1_rsp_1_data;
      end
    endcase
  end

  assign hits_0 = ((io_input_cmd_payload_address & _zz_4_) == (20'b10000000000000000000));
  always @ (*) begin
    io_outputs_0_cmd_valid = (io_input_cmd_valid && hits_0);
    io_outputs_1_cmd_valid = (io_input_cmd_valid && hits_1);
    io_input_cmd_ready = (((hits_0 && io_outputs_0_cmd_ready) || (hits_1 && io_outputs_1_cmd_ready)) || noHit);
    if(cmdWait)begin
      io_input_cmd_ready = 1'b0;
      io_outputs_0_cmd_valid = 1'b0;
      io_outputs_1_cmd_valid = 1'b0;
    end
  end

  assign _zz_1_ = io_input_cmd_payload_wr;
  assign io_outputs_0_cmd_payload_wr = _zz_1_;
  assign io_outputs_0_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_0_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_0_cmd_payload_mask = io_input_cmd_payload_mask;
  assign hits_1 = (! hits_0);
  assign _zz_2_ = io_input_cmd_payload_wr;
  assign io_outputs_1_cmd_payload_wr = _zz_2_;
  assign io_outputs_1_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_1_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_1_cmd_payload_mask = io_input_cmd_payload_mask;
  assign noHit = (! (hits_0 || hits_1));
  assign rspPending = (rspPendingCounter != (3'b000));
  assign rspNoHit = (! (rspHits_0 || rspHits_1));
  assign io_input_rsp_valid = ((io_outputs_0_rsp_valid || io_outputs_1_rsp_valid) || (rspPending && rspNoHit));
  assign io_input_rsp_payload_data = _zz_3_;
  assign cmdWait = (((io_input_cmd_valid && rspPending) && ((hits_0 != rspHits_0) || (hits_1 != rspHits_1))) || (rspPendingCounter == (3'b111)));
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      rspPendingCounter <= (3'b000);
    end else begin
      rspPendingCounter <= (_zz_5_ - _zz_9_);
    end
  end

  always @ (posedge io_clk) begin
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      rspHits_0 <= hits_0;
      rspHits_1 <= hits_1;
    end
  end

endmodule

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
      output reg  io_outputs_3_cmd_valid,
      input   io_outputs_3_cmd_ready,
      output  io_outputs_3_cmd_payload_wr,
      output [19:0] io_outputs_3_cmd_payload_address,
      output [31:0] io_outputs_3_cmd_payload_data,
      output [3:0] io_outputs_3_cmd_payload_mask,
      input   io_outputs_3_rsp_valid,
      input  [31:0] io_outputs_3_rsp_3_data,
      input   io_clk,
      input   resetCtrl_systemReset);
  reg [31:0] _zz_7_;
  wire [19:0] _zz_8_;
  wire [19:0] _zz_9_;
  wire [19:0] _zz_10_;
  wire [19:0] _zz_11_;
  wire [2:0] _zz_12_;
  wire [0:0] _zz_13_;
  wire [2:0] _zz_14_;
  wire [0:0] _zz_15_;
  wire [2:0] _zz_16_;
  wire [1:0] _zz_17_;
  wire  hits_0;
  wire  hits_1;
  wire  hits_2;
  wire  hits_3;
  wire  _zz_1_;
  wire  _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire  noHit;
  reg [2:0] rspPendingCounter;
  reg  rspHits_0;
  reg  rspHits_1;
  reg  rspHits_2;
  reg  rspHits_3;
  wire  rspPending;
  wire  rspNoHit;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  cmdWait;
  assign _zz_8_ = (20'b11111000000000000000);
  assign _zz_9_ = (20'b11111100000000000000);
  assign _zz_10_ = (20'b11111111111100000000);
  assign _zz_11_ = (20'b10000000000000000000);
  assign _zz_12_ = (rspPendingCounter + _zz_14_);
  assign _zz_13_ = ((io_input_cmd_valid && io_input_cmd_ready) && (! io_input_cmd_payload_wr));
  assign _zz_14_ = {2'd0, _zz_13_};
  assign _zz_15_ = io_input_rsp_valid;
  assign _zz_16_ = {2'd0, _zz_15_};
  assign _zz_17_ = {_zz_6_,_zz_5_};
  always @(*) begin
    case(_zz_17_)
      2'b00 : begin
        _zz_7_ = io_outputs_0_rsp_0_data;
      end
      2'b01 : begin
        _zz_7_ = io_outputs_1_rsp_1_data;
      end
      2'b10 : begin
        _zz_7_ = io_outputs_2_rsp_2_data;
      end
      default : begin
        _zz_7_ = io_outputs_3_rsp_3_data;
      end
    endcase
  end

  assign hits_0 = ((io_input_cmd_payload_address & _zz_8_) == (20'b10000000000000000000));
  always @ (*) begin
    io_outputs_0_cmd_valid = (io_input_cmd_valid && hits_0);
    io_outputs_1_cmd_valid = (io_input_cmd_valid && hits_1);
    io_outputs_2_cmd_valid = (io_input_cmd_valid && hits_2);
    io_outputs_3_cmd_valid = (io_input_cmd_valid && hits_3);
    io_input_cmd_ready = (((((hits_0 && io_outputs_0_cmd_ready) || (hits_1 && io_outputs_1_cmd_ready)) || (hits_2 && io_outputs_2_cmd_ready)) || (hits_3 && io_outputs_3_cmd_ready)) || noHit);
    if(cmdWait)begin
      io_input_cmd_ready = 1'b0;
      io_outputs_0_cmd_valid = 1'b0;
      io_outputs_1_cmd_valid = 1'b0;
      io_outputs_2_cmd_valid = 1'b0;
      io_outputs_3_cmd_valid = 1'b0;
    end
  end

  assign _zz_1_ = io_input_cmd_payload_wr;
  assign io_outputs_0_cmd_payload_wr = _zz_1_;
  assign io_outputs_0_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_0_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_0_cmd_payload_mask = io_input_cmd_payload_mask;
  assign hits_1 = ((io_input_cmd_payload_address & _zz_9_) == (20'b10010000000000000000));
  assign _zz_2_ = io_input_cmd_payload_wr;
  assign io_outputs_1_cmd_payload_wr = _zz_2_;
  assign io_outputs_1_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_1_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_1_cmd_payload_mask = io_input_cmd_payload_mask;
  assign hits_2 = ((io_input_cmd_payload_address & _zz_10_) == (20'b11110000000000000000));
  assign _zz_3_ = io_input_cmd_payload_wr;
  assign io_outputs_2_cmd_payload_wr = _zz_3_;
  assign io_outputs_2_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_2_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_2_cmd_payload_mask = io_input_cmd_payload_mask;
  assign hits_3 = ((io_input_cmd_payload_address & _zz_11_) == (20'b00000000000000000000));
  assign _zz_4_ = io_input_cmd_payload_wr;
  assign io_outputs_3_cmd_payload_wr = _zz_4_;
  assign io_outputs_3_cmd_payload_address = io_input_cmd_payload_address;
  assign io_outputs_3_cmd_payload_data = io_input_cmd_payload_data;
  assign io_outputs_3_cmd_payload_mask = io_input_cmd_payload_mask;
  assign noHit = (! (((hits_0 || hits_1) || hits_2) || hits_3));
  assign rspPending = (rspPendingCounter != (3'b000));
  assign rspNoHit = (! (((rspHits_0 || rspHits_1) || rspHits_2) || rspHits_3));
  assign io_input_rsp_valid = ((((io_outputs_0_rsp_valid || io_outputs_1_rsp_valid) || io_outputs_2_rsp_valid) || io_outputs_3_rsp_valid) || (rspPending && rspNoHit));
  assign _zz_5_ = (rspHits_1 || rspHits_3);
  assign _zz_6_ = (rspHits_2 || rspHits_3);
  assign io_input_rsp_payload_data = _zz_7_;
  assign cmdWait = (((io_input_cmd_valid && rspPending) && ((((hits_0 != rspHits_0) || (hits_1 != rspHits_1)) || (hits_2 != rspHits_2)) || (hits_3 != rspHits_3))) || (rspPendingCounter == (3'b111)));
  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      rspPendingCounter <= (3'b000);
    end else begin
      rspPendingCounter <= (_zz_12_ - _zz_16_);
    end
  end

  always @ (posedge io_clk) begin
    if((io_input_cmd_valid && io_input_cmd_ready))begin
      rspHits_0 <= hits_0;
      rspHits_1 <= hits_1;
      rspHits_2 <= hits_2;
      rspHits_3 <= hits_3;
    end
  end

endmodule

module SimpleBusArbiter (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [14:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      input   io_inputs_1_cmd_valid,
      output  io_inputs_1_cmd_ready,
      input   io_inputs_1_cmd_payload_wr,
      input  [14:0] io_inputs_1_cmd_payload_address,
      input  [31:0] io_inputs_1_cmd_payload_data,
      input  [3:0] io_inputs_1_cmd_payload_mask,
      output  io_inputs_1_rsp_valid,
      output [31:0] io_inputs_1_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [14:0] io_output_cmd_payload_address,
      output [31:0] io_output_cmd_payload_data,
      output [3:0] io_output_cmd_payload_mask,
      input   io_output_rsp_valid,
      input  [31:0] io_output_rsp_payload_data,
      input   io_clk,
      input   resetCtrl_systemReset);
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire [14:0] _zz_8_;
  wire [31:0] _zz_9_;
  wire [3:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [1:0] _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire [14:0] _zz_16_;
  wire [31:0] _zz_17_;
  wire [3:0] _zz_18_;
  wire  _zz_19_;
  wire  _zz_20_;
  wire [14:0] _zz_21_;
  wire [31:0] _zz_22_;
  wire [3:0] _zz_23_;
  wire  _zz_24_;
  wire  _zz_25_;
  wire [0:0] _zz_26_;
  wire [1:0] _zz_27_;
  reg  _zz_1_;
  reg  _zz_2_;
  StreamArbiter logic_arbiter ( 
    .io_inputs_0_0(io_inputs_0_cmd_valid),
    .io_inputs_0_ready(_zz_4_),
    .io_inputs_0_0_wr(io_inputs_0_cmd_payload_wr),
    .io_inputs_0_0_address(io_inputs_0_cmd_payload_address),
    .io_inputs_0_0_data(io_inputs_0_cmd_payload_data),
    .io_inputs_0_0_mask(io_inputs_0_cmd_payload_mask),
    .io_inputs_1_1(io_inputs_1_cmd_valid),
    .io_inputs_1_ready(_zz_5_),
    .io_inputs_1_1_wr(io_inputs_1_cmd_payload_wr),
    .io_inputs_1_1_address(io_inputs_1_cmd_payload_address),
    .io_inputs_1_1_data(io_inputs_1_cmd_payload_data),
    .io_inputs_1_1_mask(io_inputs_1_cmd_payload_mask),
    .io_output_valid(_zz_6_),
    .io_output_ready(_zz_13_),
    .io_output_payload_wr(_zz_7_),
    .io_output_payload_address(_zz_8_),
    .io_output_payload_data(_zz_9_),
    .io_output_payload_mask(_zz_10_),
    .io_chosen(_zz_11_),
    .io_chosenOH(_zz_12_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  StreamFork streamFork_3_ ( 
    .io_input_valid(_zz_6_),
    .io_input_ready(_zz_13_),
    .io_input_payload_wr(_zz_7_),
    .io_input_payload_address(_zz_8_),
    .io_input_payload_data(_zz_9_),
    .io_input_payload_mask(_zz_10_),
    .io_outputs_0_valid(_zz_14_),
    .io_outputs_0_ready(io_output_cmd_ready),
    .io_outputs_0_payload_wr(_zz_15_),
    .io_outputs_0_payload_address(_zz_16_),
    .io_outputs_0_payload_data(_zz_17_),
    .io_outputs_0_payload_mask(_zz_18_),
    .io_outputs_1_valid(_zz_19_),
    .io_outputs_1_ready(_zz_1_),
    .io_outputs_1_payload_wr(_zz_20_),
    .io_outputs_1_payload_address(_zz_21_),
    .io_outputs_1_payload_data(_zz_22_),
    .io_outputs_1_payload_mask(_zz_23_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  StreamFifoLowLatency_1_ streamFifoLowLatency_4_ ( 
    .io_push_valid(_zz_2_),
    .io_push_ready(_zz_24_),
    .io_push_payload(_zz_11_),
    .io_pop_valid(_zz_25_),
    .io_pop_ready(io_output_rsp_valid),
    .io_pop_payload(_zz_26_),
    .io_flush(_zz_3_),
    .io_occupancy(_zz_27_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  assign io_inputs_0_cmd_ready = _zz_4_;
  assign io_inputs_1_cmd_ready = _zz_5_;
  assign io_output_cmd_valid = _zz_14_;
  assign io_output_cmd_payload_wr = _zz_15_;
  assign io_output_cmd_payload_address = _zz_16_;
  assign io_output_cmd_payload_data = _zz_17_;
  assign io_output_cmd_payload_mask = _zz_18_;
  always @ (*) begin
    _zz_2_ = _zz_19_;
    _zz_1_ = _zz_24_;
    if(_zz_20_)begin
      _zz_2_ = 1'b0;
      _zz_1_ = 1'b1;
    end
  end

  assign io_inputs_0_rsp_valid = (io_output_rsp_valid && (_zz_26_ == (1'b0)));
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
  assign io_inputs_1_rsp_valid = (io_output_rsp_valid && (_zz_26_ == (1'b1)));
  assign io_inputs_1_rsp_payload_data = io_output_rsp_payload_data;
  assign _zz_3_ = 1'b0;
endmodule

module SimpleBusArbiter_1_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [13:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      input   io_inputs_1_cmd_valid,
      output  io_inputs_1_cmd_ready,
      input   io_inputs_1_cmd_payload_wr,
      input  [13:0] io_inputs_1_cmd_payload_address,
      input  [31:0] io_inputs_1_cmd_payload_data,
      input  [3:0] io_inputs_1_cmd_payload_mask,
      output  io_inputs_1_rsp_valid,
      output [31:0] io_inputs_1_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [13:0] io_output_cmd_payload_address,
      output [31:0] io_output_cmd_payload_data,
      output [3:0] io_output_cmd_payload_mask,
      input   io_output_rsp_valid,
      input  [31:0] io_output_rsp_payload_data,
      input   io_clk,
      input   resetCtrl_systemReset);
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire [13:0] _zz_8_;
  wire [31:0] _zz_9_;
  wire [3:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [1:0] _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire [13:0] _zz_16_;
  wire [31:0] _zz_17_;
  wire [3:0] _zz_18_;
  wire  _zz_19_;
  wire  _zz_20_;
  wire [13:0] _zz_21_;
  wire [31:0] _zz_22_;
  wire [3:0] _zz_23_;
  wire  _zz_24_;
  wire  _zz_25_;
  wire [0:0] _zz_26_;
  wire [1:0] _zz_27_;
  reg  _zz_1_;
  reg  _zz_2_;
  StreamArbiter_1_ logic_arbiter ( 
    .io_inputs_0_0(io_inputs_0_cmd_valid),
    .io_inputs_0_ready(_zz_4_),
    .io_inputs_0_0_wr(io_inputs_0_cmd_payload_wr),
    .io_inputs_0_0_address(io_inputs_0_cmd_payload_address),
    .io_inputs_0_0_data(io_inputs_0_cmd_payload_data),
    .io_inputs_0_0_mask(io_inputs_0_cmd_payload_mask),
    .io_inputs_1_1(io_inputs_1_cmd_valid),
    .io_inputs_1_ready(_zz_5_),
    .io_inputs_1_1_wr(io_inputs_1_cmd_payload_wr),
    .io_inputs_1_1_address(io_inputs_1_cmd_payload_address),
    .io_inputs_1_1_data(io_inputs_1_cmd_payload_data),
    .io_inputs_1_1_mask(io_inputs_1_cmd_payload_mask),
    .io_output_valid(_zz_6_),
    .io_output_ready(_zz_13_),
    .io_output_payload_wr(_zz_7_),
    .io_output_payload_address(_zz_8_),
    .io_output_payload_data(_zz_9_),
    .io_output_payload_mask(_zz_10_),
    .io_chosen(_zz_11_),
    .io_chosenOH(_zz_12_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  StreamFork_1_ streamFork_3_ ( 
    .io_input_valid(_zz_6_),
    .io_input_ready(_zz_13_),
    .io_input_payload_wr(_zz_7_),
    .io_input_payload_address(_zz_8_),
    .io_input_payload_data(_zz_9_),
    .io_input_payload_mask(_zz_10_),
    .io_outputs_0_valid(_zz_14_),
    .io_outputs_0_ready(io_output_cmd_ready),
    .io_outputs_0_payload_wr(_zz_15_),
    .io_outputs_0_payload_address(_zz_16_),
    .io_outputs_0_payload_data(_zz_17_),
    .io_outputs_0_payload_mask(_zz_18_),
    .io_outputs_1_valid(_zz_19_),
    .io_outputs_1_ready(_zz_1_),
    .io_outputs_1_payload_wr(_zz_20_),
    .io_outputs_1_payload_address(_zz_21_),
    .io_outputs_1_payload_data(_zz_22_),
    .io_outputs_1_payload_mask(_zz_23_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  StreamFifoLowLatency_1_ streamFifoLowLatency_4_ ( 
    .io_push_valid(_zz_2_),
    .io_push_ready(_zz_24_),
    .io_push_payload(_zz_11_),
    .io_pop_valid(_zz_25_),
    .io_pop_ready(io_output_rsp_valid),
    .io_pop_payload(_zz_26_),
    .io_flush(_zz_3_),
    .io_occupancy(_zz_27_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  assign io_inputs_0_cmd_ready = _zz_4_;
  assign io_inputs_1_cmd_ready = _zz_5_;
  assign io_output_cmd_valid = _zz_14_;
  assign io_output_cmd_payload_wr = _zz_15_;
  assign io_output_cmd_payload_address = _zz_16_;
  assign io_output_cmd_payload_data = _zz_17_;
  assign io_output_cmd_payload_mask = _zz_18_;
  always @ (*) begin
    _zz_2_ = _zz_19_;
    _zz_1_ = _zz_24_;
    if(_zz_20_)begin
      _zz_2_ = 1'b0;
      _zz_1_ = 1'b1;
    end
  end

  assign io_inputs_0_rsp_valid = (io_output_rsp_valid && (_zz_26_ == (1'b0)));
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
  assign io_inputs_1_rsp_valid = (io_output_rsp_valid && (_zz_26_ == (1'b1)));
  assign io_inputs_1_rsp_payload_data = io_output_rsp_payload_data;
  assign _zz_3_ = 1'b0;
endmodule

module SimpleBusArbiter_2_ (
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

module SimpleBusArbiter_3_ (
      input   io_inputs_0_cmd_valid,
      output  io_inputs_0_cmd_ready,
      input   io_inputs_0_cmd_payload_wr,
      input  [19:0] io_inputs_0_cmd_payload_address,
      input  [31:0] io_inputs_0_cmd_payload_data,
      input  [3:0] io_inputs_0_cmd_payload_mask,
      output  io_inputs_0_rsp_valid,
      output [31:0] io_inputs_0_rsp_payload_data,
      output  io_output_cmd_valid,
      input   io_output_cmd_ready,
      output  io_output_cmd_payload_wr,
      output [19:0] io_output_cmd_payload_address,
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

module SimpleBusArbiter_4_ (
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
      input   resetCtrl_systemReset);
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire [19:0] _zz_8_;
  wire [31:0] _zz_9_;
  wire [3:0] _zz_10_;
  wire [0:0] _zz_11_;
  wire [1:0] _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire [19:0] _zz_16_;
  wire [31:0] _zz_17_;
  wire [3:0] _zz_18_;
  wire  _zz_19_;
  wire  _zz_20_;
  wire [19:0] _zz_21_;
  wire [31:0] _zz_22_;
  wire [3:0] _zz_23_;
  wire  _zz_24_;
  wire  _zz_25_;
  wire [0:0] _zz_26_;
  wire [1:0] _zz_27_;
  reg  _zz_1_;
  reg  _zz_2_;
  StreamArbiter_2_ logic_arbiter ( 
    .io_inputs_0_0(io_inputs_0_cmd_valid),
    .io_inputs_0_ready(_zz_4_),
    .io_inputs_0_0_wr(io_inputs_0_cmd_payload_wr),
    .io_inputs_0_0_address(io_inputs_0_cmd_payload_address),
    .io_inputs_0_0_data(io_inputs_0_cmd_payload_data),
    .io_inputs_0_0_mask(io_inputs_0_cmd_payload_mask),
    .io_inputs_1_1(io_inputs_1_cmd_valid),
    .io_inputs_1_ready(_zz_5_),
    .io_inputs_1_1_wr(io_inputs_1_cmd_payload_wr),
    .io_inputs_1_1_address(io_inputs_1_cmd_payload_address),
    .io_inputs_1_1_data(io_inputs_1_cmd_payload_data),
    .io_inputs_1_1_mask(io_inputs_1_cmd_payload_mask),
    .io_output_valid(_zz_6_),
    .io_output_ready(_zz_13_),
    .io_output_payload_wr(_zz_7_),
    .io_output_payload_address(_zz_8_),
    .io_output_payload_data(_zz_9_),
    .io_output_payload_mask(_zz_10_),
    .io_chosen(_zz_11_),
    .io_chosenOH(_zz_12_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  StreamFork_2_ streamFork_3_ ( 
    .io_input_valid(_zz_6_),
    .io_input_ready(_zz_13_),
    .io_input_payload_wr(_zz_7_),
    .io_input_payload_address(_zz_8_),
    .io_input_payload_data(_zz_9_),
    .io_input_payload_mask(_zz_10_),
    .io_outputs_0_valid(_zz_14_),
    .io_outputs_0_ready(io_output_cmd_ready),
    .io_outputs_0_payload_wr(_zz_15_),
    .io_outputs_0_payload_address(_zz_16_),
    .io_outputs_0_payload_data(_zz_17_),
    .io_outputs_0_payload_mask(_zz_18_),
    .io_outputs_1_valid(_zz_19_),
    .io_outputs_1_ready(_zz_1_),
    .io_outputs_1_payload_wr(_zz_20_),
    .io_outputs_1_payload_address(_zz_21_),
    .io_outputs_1_payload_data(_zz_22_),
    .io_outputs_1_payload_mask(_zz_23_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  StreamFifoLowLatency_1_ streamFifoLowLatency_4_ ( 
    .io_push_valid(_zz_2_),
    .io_push_ready(_zz_24_),
    .io_push_payload(_zz_11_),
    .io_pop_valid(_zz_25_),
    .io_pop_ready(io_output_rsp_valid),
    .io_pop_payload(_zz_26_),
    .io_flush(_zz_3_),
    .io_occupancy(_zz_27_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  assign io_inputs_0_cmd_ready = _zz_4_;
  assign io_inputs_1_cmd_ready = _zz_5_;
  assign io_output_cmd_valid = _zz_14_;
  assign io_output_cmd_payload_wr = _zz_15_;
  assign io_output_cmd_payload_address = _zz_16_;
  assign io_output_cmd_payload_data = _zz_17_;
  assign io_output_cmd_payload_mask = _zz_18_;
  always @ (*) begin
    _zz_2_ = _zz_19_;
    _zz_1_ = _zz_24_;
    if(_zz_20_)begin
      _zz_2_ = 1'b0;
      _zz_1_ = 1'b1;
    end
  end

  assign io_inputs_0_rsp_valid = (io_output_rsp_valid && (_zz_26_ == (1'b0)));
  assign io_inputs_0_rsp_payload_data = io_output_rsp_payload_data;
  assign io_inputs_1_rsp_valid = (io_output_rsp_valid && (_zz_26_ == (1'b1)));
  assign io_inputs_1_rsp_payload_data = io_output_rsp_payload_data;
  assign _zz_3_ = 1'b0;
endmodule

module Igloo2Perf (
      input   io_clk,
      input   io_reset,
      output [2:0] io_leds,
      output  io_serialTx,
      input   io_serialRx,
      output [0:0] io_flash_ss,
      output  io_flash_sclk,
      output  io_flash_mosi,
      input   io_flash_miso);
  wire  _zz_40_;
  wire  _zz_41_;
  wire  _zz_42_;
  wire  _zz_43_;
  wire [14:0] _zz_44_;
  wire [13:0] _zz_45_;
  wire [5:0] _zz_46_;
  wire  _zz_47_;
  wire  _zz_48_;
  wire  _zz_49_;
  wire [31:0] _zz_50_;
  wire  _zz_51_;
  wire  _zz_52_;
  wire [31:0] _zz_53_;
  wire  _zz_54_;
  wire  _zz_55_;
  wire [31:0] _zz_56_;
  wire  _zz_57_;
  wire [2:0] _zz_58_;
  wire  _zz_59_;
  wire  _zz_60_;
  wire  _zz_61_;
  wire [31:0] _zz_62_;
  wire  _zz_63_;
  wire  _zz_64_;
  wire [0:0] _zz_65_;
  wire  _zz_66_;
  wire [31:0] _zz_67_;
  wire  _zz_68_;
  wire  _zz_69_;
  wire [31:0] _zz_70_;
  wire [31:0] _zz_71_;
  wire [1:0] _zz_72_;
  wire [7:0] _zz_73_;
  wire  _zz_74_;
  wire  _zz_75_;
  wire [31:0] _zz_76_;
  wire  _zz_77_;
  wire  _zz_78_;
  wire [19:0] _zz_79_;
  wire [31:0] _zz_80_;
  wire [3:0] _zz_81_;
  wire  _zz_82_;
  wire  _zz_83_;
  wire [19:0] _zz_84_;
  wire [31:0] _zz_85_;
  wire [3:0] _zz_86_;
  wire  _zz_87_;
  wire  _zz_88_;
  wire [31:0] _zz_89_;
  wire  _zz_90_;
  wire  _zz_91_;
  wire [19:0] _zz_92_;
  wire [31:0] _zz_93_;
  wire [3:0] _zz_94_;
  wire  _zz_95_;
  wire  _zz_96_;
  wire [19:0] _zz_97_;
  wire [31:0] _zz_98_;
  wire [3:0] _zz_99_;
  wire  _zz_100_;
  wire  _zz_101_;
  wire [31:0] _zz_102_;
  wire  _zz_103_;
  wire  _zz_104_;
  wire [19:0] _zz_105_;
  wire [31:0] _zz_106_;
  wire [3:0] _zz_107_;
  wire  _zz_108_;
  wire  _zz_109_;
  wire [19:0] _zz_110_;
  wire [31:0] _zz_111_;
  wire [3:0] _zz_112_;
  wire  _zz_113_;
  wire  _zz_114_;
  wire [19:0] _zz_115_;
  wire [31:0] _zz_116_;
  wire [3:0] _zz_117_;
  wire  _zz_118_;
  wire  _zz_119_;
  wire [19:0] _zz_120_;
  wire [31:0] _zz_121_;
  wire [3:0] _zz_122_;
  wire  _zz_123_;
  wire  _zz_124_;
  wire [31:0] _zz_125_;
  wire  _zz_126_;
  wire  _zz_127_;
  wire [31:0] _zz_128_;
  wire  _zz_129_;
  wire  _zz_130_;
  wire [14:0] _zz_131_;
  wire [31:0] _zz_132_;
  wire [3:0] _zz_133_;
  wire  _zz_134_;
  wire  _zz_135_;
  wire [31:0] _zz_136_;
  wire  _zz_137_;
  wire  _zz_138_;
  wire [31:0] _zz_139_;
  wire  _zz_140_;
  wire  _zz_141_;
  wire [13:0] _zz_142_;
  wire [31:0] _zz_143_;
  wire [3:0] _zz_144_;
  wire  _zz_145_;
  wire  _zz_146_;
  wire [31:0] _zz_147_;
  wire  _zz_148_;
  wire  _zz_149_;
  wire [5:0] _zz_150_;
  wire [31:0] _zz_151_;
  wire [3:0] _zz_152_;
  wire  _zz_153_;
  wire  _zz_154_;
  wire [31:0] _zz_155_;
  wire  _zz_156_;
  wire  _zz_157_;
  wire [19:0] _zz_158_;
  wire [31:0] _zz_159_;
  wire [3:0] _zz_160_;
  wire  _zz_161_;
  wire  _zz_162_;
  wire [31:0] _zz_163_;
  wire  _zz_164_;
  wire  _zz_165_;
  wire [31:0] _zz_166_;
  wire  _zz_167_;
  wire  _zz_168_;
  wire [19:0] _zz_169_;
  wire [31:0] _zz_170_;
  wire [3:0] _zz_171_;
  wire  _zz_172_;
  wire  _zz_173_;
  wire  _zz_174_;
  wire  _zz_175_;
  wire  _zz_176_;
  wire  _zz_177_;
  wire [1:0] _zz_178_;
  reg  resetCtrl_mainClkResetUnbuffered;
  reg [14:0] resetCtrl_systemClkResetCounter = (15'b000000000000000);
  wire [14:0] _zz_1_;
  reg  resetCtrl_systemResetBuffered;
  wire  resetCtrl_systemReset;
  reg  resetCtrl_progResetBuffered;
  wire  resetCtrl_progReset;
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
  wire  system_slowBus_cmd_valid;
  wire  system_slowBus_cmd_ready;
  wire  system_slowBus_cmd_payload_wr;
  wire [19:0] system_slowBus_cmd_payload_address;
  wire [31:0] system_slowBus_cmd_payload_data;
  wire [3:0] system_slowBus_cmd_payload_mask;
  wire  system_slowBus_rsp_valid;
  wire [31:0] system_slowBus_rsp_payload_data;
  wire  _zz_2_;
  wire [31:0] _zz_3_;
  reg  _zz_4_;
  reg  _zz_5_;
  reg [31:0] _zz_6_;
  reg [31:0] _zz_7_;
  reg [1:0] _zz_8_;
  reg [3:0] _zz_9_;
  reg  prog_ssReg;
  reg  prog_sclkReg;
  reg  prog_mosiReg;
  reg  io_input_rsp_regNext_valid;
  reg [31:0] io_input_rsp_regNext_payload_data;
  wire  _zz_10_;
  reg  _zz_11_;
  reg  _zz_12_;
  reg  _zz_13_;
  reg [19:0] _zz_14_;
  reg [31:0] _zz_15_;
  reg [3:0] _zz_16_;
  wire  _zz_17_;
  reg  _zz_18_;
  reg  _zz_19_;
  reg  _zz_20_;
  reg [19:0] _zz_21_;
  reg [31:0] _zz_22_;
  reg [3:0] _zz_23_;
  wire  _zz_24_;
  wire  _zz_25_;
  reg  _zz_26_;
  reg  _zz_27_;
  reg  _zz_28_;
  reg [14:0] _zz_29_;
  reg [31:0] _zz_30_;
  reg [3:0] _zz_31_;
  wire  _zz_32_;
  wire  _zz_33_;
  reg  _zz_34_;
  reg  _zz_35_;
  reg  _zz_36_;
  reg [13:0] _zz_37_;
  reg [31:0] _zz_38_;
  reg [3:0] _zz_39_;
  assign _zz_172_ = (resetCtrl_systemClkResetCounter != _zz_1_);
  assign _zz_173_ = (_zz_42_ && (! _zz_2_));
  assign _zz_174_ = (! _zz_11_);
  assign _zz_175_ = (! _zz_18_);
  assign _zz_176_ = (! _zz_26_);
  assign _zz_177_ = (! _zz_34_);
  assign _zz_178_ = (_zz_4_ ? _zz_8_ : _zz_72_);
  BufferCC_1_ bufferCC_2_ ( 
    .io_dataIn(io_reset),
    .io_dataOut(_zz_47_),
    .io_clk(io_clk) 
  );
  SimpleBusRam system_iRam ( 
    .io_bus_cmd_valid(_zz_129_),
    .io_bus_cmd_ready(_zz_48_),
    .io_bus_cmd_payload_wr(_zz_130_),
    .io_bus_cmd_payload_address(_zz_131_),
    .io_bus_cmd_payload_data(_zz_132_),
    .io_bus_cmd_payload_mask(_zz_133_),
    .io_bus_rsp_valid(_zz_49_),
    .io_bus_rsp_payload_data(_zz_50_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SimpleBusRam_1_ system_dRam ( 
    .io_bus_cmd_valid(_zz_140_),
    .io_bus_cmd_ready(_zz_51_),
    .io_bus_cmd_payload_wr(_zz_141_),
    .io_bus_cmd_payload_address(_zz_142_),
    .io_bus_cmd_payload_data(_zz_143_),
    .io_bus_cmd_payload_mask(_zz_144_),
    .io_bus_rsp_valid(_zz_52_),
    .io_bus_rsp_payload_data(_zz_53_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  Peripherals system_peripherals ( 
    .io_bus_cmd_valid(_zz_148_),
    .io_bus_cmd_ready(_zz_54_),
    .io_bus_cmd_payload_wr(_zz_149_),
    .io_bus_cmd_payload_address(_zz_150_),
    .io_bus_cmd_payload_data(_zz_151_),
    .io_bus_cmd_payload_mask(_zz_152_),
    .io_bus_rsp_valid(_zz_55_),
    .io_bus_rsp_payload_data(_zz_56_),
    .io_mTimeInterrupt(_zz_57_),
    .io_leds(_zz_58_),
    .io_serialTx(_zz_59_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  FlashXpi system_flashXip ( 
    .io_bus_cmd_valid(_zz_156_),
    .io_bus_cmd_ready(_zz_60_),
    .io_bus_cmd_payload_wr(_zz_157_),
    .io_bus_cmd_payload_address(_zz_158_),
    .io_bus_cmd_payload_data(_zz_159_),
    .io_bus_cmd_payload_mask(_zz_160_),
    .io_bus_rsp_valid(_zz_61_),
    .io_bus_rsp_payload_data(_zz_62_),
    .io_flash_ss(_zz_65_),
    .io_flash_sclk(_zz_63_),
    .io_flash_mosi(_zz_64_),
    .io_flash_miso(io_flash_miso),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  VexRiscv system_cpu ( 
    .iBus_cmd_valid(_zz_66_),
    .iBus_cmd_ready(system_iBus_cmd_ready),
    .iBus_cmd_payload_pc(_zz_67_),
    .iBus_rsp_valid(system_iBus_rsp_valid),
    .iBus_rsp_payload_error(_zz_40_),
    .iBus_rsp_payload_inst(system_iBus_rsp_payload_data),
    .timerInterrupt(_zz_57_),
    .externalInterrupt(_zz_41_),
    .dBus_cmd_valid(_zz_68_),
    .dBus_cmd_ready(_zz_42_),
    .dBus_cmd_payload_wr(_zz_69_),
    .dBus_cmd_payload_address(_zz_70_),
    .dBus_cmd_payload_data(_zz_71_),
    .dBus_cmd_payload_size(_zz_72_),
    .dBus_rsp_ready(system_dBus_rsp_valid),
    .dBus_rsp_error(_zz_43_),
    .dBus_rsp_data(system_dBus_rsp_payload_data),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SerialRxOutput prog_ctrl ( 
    .io_serialRx(io_serialRx),
    .io_output(_zz_73_),
    .io_clk(io_clk),
    .resetCtrl_progReset(resetCtrl_progReset) 
  );
  SimpleBusDecoder system_dBus_decoder ( 
    .io_input_cmd_valid(system_dBus_cmd_valid),
    .io_input_cmd_ready(_zz_74_),
    .io_input_cmd_payload_wr(system_dBus_cmd_payload_wr),
    .io_input_cmd_payload_address(system_dBus_cmd_payload_address),
    .io_input_cmd_payload_data(system_dBus_cmd_payload_data),
    .io_input_cmd_payload_mask(system_dBus_cmd_payload_mask),
    .io_input_rsp_valid(_zz_75_),
    .io_input_rsp_payload_data(_zz_76_),
    .io_outputs_0_cmd_valid(_zz_77_),
    .io_outputs_0_cmd_ready(_zz_134_),
    .io_outputs_0_cmd_payload_wr(_zz_78_),
    .io_outputs_0_cmd_payload_address(_zz_79_),
    .io_outputs_0_cmd_payload_data(_zz_80_),
    .io_outputs_0_cmd_payload_mask(_zz_81_),
    .io_outputs_0_rsp_valid(_zz_135_),
    .io_outputs_0_rsp_0_data(_zz_136_),
    .io_outputs_1_cmd_valid(_zz_82_),
    .io_outputs_1_cmd_ready(_zz_12_),
    .io_outputs_1_cmd_payload_wr(_zz_83_),
    .io_outputs_1_cmd_payload_address(_zz_84_),
    .io_outputs_1_cmd_payload_data(_zz_85_),
    .io_outputs_1_cmd_payload_mask(_zz_86_),
    .io_outputs_1_rsp_valid(_zz_162_),
    .io_outputs_1_rsp_1_data(_zz_163_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SimpleBusDecoder_1_ system_iBus_decoder ( 
    .io_input_cmd_valid(system_iBus_cmd_valid),
    .io_input_cmd_ready(_zz_87_),
    .io_input_cmd_payload_wr(system_iBus_cmd_payload_wr),
    .io_input_cmd_payload_address(system_iBus_cmd_payload_address),
    .io_input_cmd_payload_data(system_iBus_cmd_payload_data),
    .io_input_cmd_payload_mask(system_iBus_cmd_payload_mask),
    .io_input_rsp_valid(_zz_88_),
    .io_input_rsp_payload_data(_zz_89_),
    .io_outputs_0_cmd_valid(_zz_90_),
    .io_outputs_0_cmd_ready(_zz_123_),
    .io_outputs_0_cmd_payload_wr(_zz_91_),
    .io_outputs_0_cmd_payload_address(_zz_92_),
    .io_outputs_0_cmd_payload_data(_zz_93_),
    .io_outputs_0_cmd_payload_mask(_zz_94_),
    .io_outputs_0_rsp_valid(_zz_124_),
    .io_outputs_0_rsp_0_data(_zz_125_),
    .io_outputs_1_cmd_valid(_zz_95_),
    .io_outputs_1_cmd_ready(_zz_19_),
    .io_outputs_1_cmd_payload_wr(_zz_96_),
    .io_outputs_1_cmd_payload_address(_zz_97_),
    .io_outputs_1_cmd_payload_data(_zz_98_),
    .io_outputs_1_cmd_payload_mask(_zz_99_),
    .io_outputs_1_rsp_valid(_zz_165_),
    .io_outputs_1_rsp_1_data(_zz_166_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SimpleBusDecoder_2_ system_slowBus_decoder ( 
    .io_input_cmd_valid(system_slowBus_cmd_valid),
    .io_input_cmd_ready(_zz_100_),
    .io_input_cmd_payload_wr(system_slowBus_cmd_payload_wr),
    .io_input_cmd_payload_address(system_slowBus_cmd_payload_address),
    .io_input_cmd_payload_data(system_slowBus_cmd_payload_data),
    .io_input_cmd_payload_mask(system_slowBus_cmd_payload_mask),
    .io_input_rsp_valid(_zz_101_),
    .io_input_rsp_payload_data(_zz_102_),
    .io_outputs_0_cmd_valid(_zz_103_),
    .io_outputs_0_cmd_ready(_zz_27_),
    .io_outputs_0_cmd_payload_wr(_zz_104_),
    .io_outputs_0_cmd_payload_address(_zz_105_),
    .io_outputs_0_cmd_payload_data(_zz_106_),
    .io_outputs_0_cmd_payload_mask(_zz_107_),
    .io_outputs_0_rsp_valid(_zz_127_),
    .io_outputs_0_rsp_0_data(_zz_128_),
    .io_outputs_1_cmd_valid(_zz_108_),
    .io_outputs_1_cmd_ready(_zz_35_),
    .io_outputs_1_cmd_payload_wr(_zz_109_),
    .io_outputs_1_cmd_payload_address(_zz_110_),
    .io_outputs_1_cmd_payload_data(_zz_111_),
    .io_outputs_1_cmd_payload_mask(_zz_112_),
    .io_outputs_1_rsp_valid(_zz_138_),
    .io_outputs_1_rsp_1_data(_zz_139_),
    .io_outputs_2_cmd_valid(_zz_113_),
    .io_outputs_2_cmd_ready(_zz_145_),
    .io_outputs_2_cmd_payload_wr(_zz_114_),
    .io_outputs_2_cmd_payload_address(_zz_115_),
    .io_outputs_2_cmd_payload_data(_zz_116_),
    .io_outputs_2_cmd_payload_mask(_zz_117_),
    .io_outputs_2_rsp_valid(_zz_146_),
    .io_outputs_2_rsp_2_data(_zz_147_),
    .io_outputs_3_cmd_valid(_zz_118_),
    .io_outputs_3_cmd_ready(_zz_153_),
    .io_outputs_3_cmd_payload_wr(_zz_119_),
    .io_outputs_3_cmd_payload_address(_zz_120_),
    .io_outputs_3_cmd_payload_data(_zz_121_),
    .io_outputs_3_cmd_payload_mask(_zz_122_),
    .io_outputs_3_rsp_valid(_zz_154_),
    .io_outputs_3_rsp_3_data(_zz_155_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SimpleBusArbiter system_iRam_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_90_),
    .io_inputs_0_cmd_ready(_zz_123_),
    .io_inputs_0_cmd_payload_wr(_zz_91_),
    .io_inputs_0_cmd_payload_address(_zz_44_),
    .io_inputs_0_cmd_payload_data(_zz_93_),
    .io_inputs_0_cmd_payload_mask(_zz_94_),
    .io_inputs_0_rsp_valid(_zz_124_),
    .io_inputs_0_rsp_payload_data(_zz_125_),
    .io_inputs_1_cmd_valid(_zz_26_),
    .io_inputs_1_cmd_ready(_zz_126_),
    .io_inputs_1_cmd_payload_wr(_zz_28_),
    .io_inputs_1_cmd_payload_address(_zz_29_),
    .io_inputs_1_cmd_payload_data(_zz_30_),
    .io_inputs_1_cmd_payload_mask(_zz_31_),
    .io_inputs_1_rsp_valid(_zz_127_),
    .io_inputs_1_rsp_payload_data(_zz_128_),
    .io_output_cmd_valid(_zz_129_),
    .io_output_cmd_ready(_zz_48_),
    .io_output_cmd_payload_wr(_zz_130_),
    .io_output_cmd_payload_address(_zz_131_),
    .io_output_cmd_payload_data(_zz_132_),
    .io_output_cmd_payload_mask(_zz_133_),
    .io_output_rsp_valid(_zz_49_),
    .io_output_rsp_payload_data(_zz_50_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SimpleBusArbiter_1_ system_dRam_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_77_),
    .io_inputs_0_cmd_ready(_zz_134_),
    .io_inputs_0_cmd_payload_wr(_zz_78_),
    .io_inputs_0_cmd_payload_address(_zz_45_),
    .io_inputs_0_cmd_payload_data(_zz_80_),
    .io_inputs_0_cmd_payload_mask(_zz_81_),
    .io_inputs_0_rsp_valid(_zz_135_),
    .io_inputs_0_rsp_payload_data(_zz_136_),
    .io_inputs_1_cmd_valid(_zz_34_),
    .io_inputs_1_cmd_ready(_zz_137_),
    .io_inputs_1_cmd_payload_wr(_zz_36_),
    .io_inputs_1_cmd_payload_address(_zz_37_),
    .io_inputs_1_cmd_payload_data(_zz_38_),
    .io_inputs_1_cmd_payload_mask(_zz_39_),
    .io_inputs_1_rsp_valid(_zz_138_),
    .io_inputs_1_rsp_payload_data(_zz_139_),
    .io_output_cmd_valid(_zz_140_),
    .io_output_cmd_ready(_zz_51_),
    .io_output_cmd_payload_wr(_zz_141_),
    .io_output_cmd_payload_address(_zz_142_),
    .io_output_cmd_payload_data(_zz_143_),
    .io_output_cmd_payload_mask(_zz_144_),
    .io_output_rsp_valid(_zz_52_),
    .io_output_rsp_payload_data(_zz_53_),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  SimpleBusArbiter_2_ system_peripherals_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_113_),
    .io_inputs_0_cmd_ready(_zz_145_),
    .io_inputs_0_cmd_payload_wr(_zz_114_),
    .io_inputs_0_cmd_payload_address(_zz_46_),
    .io_inputs_0_cmd_payload_data(_zz_116_),
    .io_inputs_0_cmd_payload_mask(_zz_117_),
    .io_inputs_0_rsp_valid(_zz_146_),
    .io_inputs_0_rsp_payload_data(_zz_147_),
    .io_output_cmd_valid(_zz_148_),
    .io_output_cmd_ready(_zz_54_),
    .io_output_cmd_payload_wr(_zz_149_),
    .io_output_cmd_payload_address(_zz_150_),
    .io_output_cmd_payload_data(_zz_151_),
    .io_output_cmd_payload_mask(_zz_152_),
    .io_output_rsp_valid(_zz_55_),
    .io_output_rsp_payload_data(_zz_56_) 
  );
  SimpleBusArbiter_3_ system_flashXip_io_bus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_118_),
    .io_inputs_0_cmd_ready(_zz_153_),
    .io_inputs_0_cmd_payload_wr(_zz_119_),
    .io_inputs_0_cmd_payload_address(_zz_120_),
    .io_inputs_0_cmd_payload_data(_zz_121_),
    .io_inputs_0_cmd_payload_mask(_zz_122_),
    .io_inputs_0_rsp_valid(_zz_154_),
    .io_inputs_0_rsp_payload_data(_zz_155_),
    .io_output_cmd_valid(_zz_156_),
    .io_output_cmd_ready(_zz_60_),
    .io_output_cmd_payload_wr(_zz_157_),
    .io_output_cmd_payload_address(_zz_158_),
    .io_output_cmd_payload_data(_zz_159_),
    .io_output_cmd_payload_mask(_zz_160_),
    .io_output_rsp_valid(_zz_61_),
    .io_output_rsp_payload_data(_zz_62_) 
  );
  SimpleBusArbiter_4_ system_slowBus_arbiter ( 
    .io_inputs_0_cmd_valid(_zz_11_),
    .io_inputs_0_cmd_ready(_zz_161_),
    .io_inputs_0_cmd_payload_wr(_zz_13_),
    .io_inputs_0_cmd_payload_address(_zz_14_),
    .io_inputs_0_cmd_payload_data(_zz_15_),
    .io_inputs_0_cmd_payload_mask(_zz_16_),
    .io_inputs_0_rsp_valid(_zz_162_),
    .io_inputs_0_rsp_payload_data(_zz_163_),
    .io_inputs_1_cmd_valid(_zz_18_),
    .io_inputs_1_cmd_ready(_zz_164_),
    .io_inputs_1_cmd_payload_wr(_zz_20_),
    .io_inputs_1_cmd_payload_address(_zz_21_),
    .io_inputs_1_cmd_payload_data(_zz_22_),
    .io_inputs_1_cmd_payload_mask(_zz_23_),
    .io_inputs_1_rsp_valid(_zz_165_),
    .io_inputs_1_rsp_payload_data(_zz_166_),
    .io_output_cmd_valid(_zz_167_),
    .io_output_cmd_ready(system_slowBus_cmd_ready),
    .io_output_cmd_payload_wr(_zz_168_),
    .io_output_cmd_payload_address(_zz_169_),
    .io_output_cmd_payload_data(_zz_170_),
    .io_output_cmd_payload_mask(_zz_171_),
    .io_output_rsp_valid(system_slowBus_rsp_valid),
    .io_output_rsp_payload_data(system_slowBus_rsp_payload_data),
    .io_clk(io_clk),
    .resetCtrl_systemReset(resetCtrl_systemReset) 
  );
  always @ (*) begin
    resetCtrl_mainClkResetUnbuffered = 1'b0;
    if(_zz_172_)begin
      resetCtrl_mainClkResetUnbuffered = 1'b1;
    end
  end

  assign _zz_1_[14 : 0] = (15'b111111111111111);
  assign resetCtrl_systemReset = resetCtrl_systemResetBuffered;
  assign resetCtrl_progReset = resetCtrl_progResetBuffered;
  assign io_serialTx = _zz_59_;
  assign io_leds = _zz_58_;
  assign system_iBus_cmd_valid = _zz_66_;
  assign system_iBus_cmd_payload_wr = 1'b0;
  assign system_iBus_cmd_payload_address = _zz_67_[19:0];
  assign system_iBus_cmd_payload_data = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
  assign system_iBus_cmd_payload_mask = (4'bxxxx);
  assign _zz_40_ = 1'b0;
  assign _zz_42_ = (! _zz_4_);
  assign _zz_3_ = (_zz_4_ ? _zz_6_ : _zz_70_);
  assign system_dBus_cmd_valid = (_zz_68_ || _zz_4_);
  assign system_dBus_cmd_payload_wr = (_zz_4_ ? _zz_5_ : _zz_69_);
  assign system_dBus_cmd_payload_address = _zz_3_[19:0];
  assign system_dBus_cmd_payload_data = (_zz_4_ ? _zz_7_ : _zz_71_);
  always @ (*) begin
    case(_zz_178_)
      2'b00 : begin
        _zz_9_ = (4'b0001);
      end
      2'b01 : begin
        _zz_9_ = (4'b0011);
      end
      default : begin
        _zz_9_ = (4'b1111);
      end
    endcase
  end

  assign system_dBus_cmd_payload_mask = (_zz_9_ <<< _zz_3_[1 : 0]);
  assign _zz_2_ = system_dBus_cmd_ready;
  assign _zz_41_ = 1'b0;
  assign io_flash_ss[0] = prog_ssReg;
  assign io_flash_sclk = prog_sclkReg;
  assign io_flash_mosi = prog_mosiReg;
  assign system_dBus_cmd_ready = _zz_74_;
  assign system_dBus_rsp_valid = _zz_75_;
  assign system_dBus_rsp_payload_data = _zz_76_;
  assign system_iBus_cmd_ready = _zz_87_;
  assign system_iBus_rsp_valid = _zz_88_;
  assign system_iBus_rsp_payload_data = _zz_89_;
  assign system_slowBus_cmd_ready = _zz_100_;
  assign system_slowBus_rsp_valid = io_input_rsp_regNext_valid;
  assign system_slowBus_rsp_payload_data = io_input_rsp_regNext_payload_data;
  assign system_slowBus_cmd_valid = _zz_167_;
  assign system_slowBus_cmd_payload_wr = _zz_168_;
  assign system_slowBus_cmd_payload_address = _zz_169_;
  assign system_slowBus_cmd_payload_data = _zz_170_;
  assign system_slowBus_cmd_payload_mask = _zz_171_;
  assign _zz_45_ = _zz_79_[13:0];
  assign _zz_10_ = _zz_161_;
  assign _zz_44_ = _zz_92_[14:0];
  assign _zz_17_ = _zz_164_;
  assign _zz_24_ = _zz_103_;
  assign _zz_25_ = _zz_126_;
  assign _zz_32_ = _zz_108_;
  assign _zz_33_ = _zz_137_;
  assign _zz_46_ = _zz_115_[5:0];
  always @ (posedge io_clk) begin
    if(_zz_172_)begin
      resetCtrl_systemClkResetCounter <= (resetCtrl_systemClkResetCounter + (15'b000000000000001));
    end
    if(_zz_47_)begin
      resetCtrl_systemClkResetCounter <= (15'b000000000000000);
    end
  end

  always @ (posedge io_clk) begin
    resetCtrl_systemResetBuffered <= resetCtrl_mainClkResetUnbuffered;
    resetCtrl_progResetBuffered <= resetCtrl_mainClkResetUnbuffered;
    if(_zz_73_[7])begin
      resetCtrl_systemResetBuffered <= 1'b1;
    end
  end

  always @ (posedge io_clk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      _zz_4_ <= 1'b0;
      io_input_rsp_regNext_valid <= 1'b0;
      _zz_11_ <= 1'b0;
      _zz_12_ <= 1'b1;
      _zz_18_ <= 1'b0;
      _zz_19_ <= 1'b1;
      _zz_26_ <= 1'b0;
      _zz_27_ <= 1'b1;
      _zz_34_ <= 1'b0;
      _zz_35_ <= 1'b1;
    end else begin
      if(_zz_2_)begin
        _zz_4_ <= 1'b0;
      end
      if(_zz_173_)begin
        _zz_4_ <= _zz_68_;
      end
      io_input_rsp_regNext_valid <= _zz_101_;
      if(_zz_174_)begin
        _zz_11_ <= _zz_82_;
        _zz_12_ <= (! _zz_82_);
      end else begin
        _zz_11_ <= (! _zz_10_);
        _zz_12_ <= _zz_10_;
      end
      if(_zz_175_)begin
        _zz_18_ <= _zz_95_;
        _zz_19_ <= (! _zz_95_);
      end else begin
        _zz_18_ <= (! _zz_17_);
        _zz_19_ <= _zz_17_;
      end
      if(_zz_176_)begin
        _zz_26_ <= _zz_24_;
        _zz_27_ <= (! _zz_24_);
      end else begin
        _zz_26_ <= (! _zz_25_);
        _zz_27_ <= _zz_25_;
      end
      if(_zz_177_)begin
        _zz_34_ <= _zz_32_;
        _zz_35_ <= (! _zz_32_);
      end else begin
        _zz_34_ <= (! _zz_33_);
        _zz_35_ <= _zz_33_;
      end
    end
  end

  always @ (posedge io_clk) begin
    if(_zz_173_)begin
      _zz_5_ <= _zz_69_;
      _zz_6_ <= _zz_70_;
      _zz_7_ <= _zz_71_;
      _zz_8_ <= _zz_72_;
    end
    io_input_rsp_regNext_payload_data <= _zz_102_;
    if(_zz_174_)begin
      _zz_13_ <= _zz_83_;
      _zz_14_ <= _zz_84_;
      _zz_15_ <= _zz_85_;
      _zz_16_ <= _zz_86_;
    end
    if(_zz_175_)begin
      _zz_20_ <= _zz_96_;
      _zz_21_ <= _zz_97_;
      _zz_22_ <= _zz_98_;
      _zz_23_ <= _zz_99_;
    end
    if(_zz_176_)begin
      _zz_28_ <= _zz_104_;
      _zz_29_ <= _zz_105_[14:0];
      _zz_30_ <= _zz_106_;
      _zz_31_ <= _zz_107_;
    end
    if(_zz_177_)begin
      _zz_36_ <= _zz_109_;
      _zz_37_ <= _zz_110_[13:0];
      _zz_38_ <= _zz_111_;
      _zz_39_ <= _zz_112_;
    end
  end

  always @ (posedge io_clk or posedge resetCtrl_progReset) begin
    if (resetCtrl_progReset) begin
      prog_ssReg <= 1'b1;
      prog_sclkReg <= 1'b1;
      prog_mosiReg <= 1'b1;
    end else begin
      if(_zz_73_[6])begin
        prog_ssReg <= _zz_73_[0];
        prog_sclkReg <= _zz_73_[1];
        prog_mosiReg <= _zz_73_[2];
      end else begin
        prog_ssReg <= _zz_65_[0];
        prog_sclkReg <= _zz_63_;
        prog_mosiReg <= _zz_64_;
      end
    end
  end

endmodule

