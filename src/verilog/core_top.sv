`default_nettype none

module core_top (
    input  wire [7:0]  intrpt,
    input  wire        aclk,
    input  wire        aresetn,

    output wire [3:0]  arid,
    output wire [31:0] araddr,
    output wire [3:0]  arlen,
    output wire [2:0]  arsize,
    output wire [1:0]  arburst,
    output wire [1:0]  arlock,
    output wire [3:0]  arcache,
    output wire [2:0]  arprot,
    output wire        arvalid,
    input  wire        arready,
    input  wire [3:0]  rid,
    input  wire [31:0] rdata,
    input  wire [1:0]  rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    output wire [3:0]  awid,
    output wire [31:0] awaddr,
    output wire [3:0]  awlen,
    output wire [2:0]  awsize,
    output wire [1:0]  awburst,
    output wire [1:0]  awlock,
    output wire [3:0]  awcache,
    output wire [2:0]  awprot,
    output wire        awvalid,
    input  wire        awready,
    output wire [3:0]  wid,
    output wire [31:0] wdata,
    output wire [3:0]  wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,
    input  wire [3:0]  bid,
    input  wire [1:0]  bresp,
    input  wire        bvalid,
    output wire        bready,

    input  wire        break_point,
    input  wire        infor_flag,
    input  wire [4:0]  reg_num,
    output wire        ws_valid,
    output wire [31:0] rf_rdata,
    output wire [31:0] debug0_wb_pc,
    output wire [3:0]  debug0_wb_rf_wen,
    output wire [4:0]  debug0_wb_rf_wnum,
    output wire [31:0] debug0_wb_rf_wdata,
    output wire [31:0] debug0_wb_inst
);

    wire [3:0] cpu_awid;
    wire [7:0] cpu_arlen;
    wire [7:0] cpu_awlen;
    wire       cmt0_valid;
    wire       cmt0_rd_valid;
    wire [31:0] cmt0_pc;
    wire [31:0] cmt0_inst;
    wire [31:0] cmt0_data;
    wire [4:0]  cmt0_rd;
    wire       cmt1_valid;
    wire       cmt1_rd_valid;
    wire [31:0] cmt1_pc;
    wire [31:0] cmt1_inst;
    wire [31:0] cmt1_data;
    wire [4:0]  cmt1_rd;
    wire       cmt2_valid;
    wire       cmt2_rd_valid;
    wire [31:0] cmt2_pc;
    wire [31:0] cmt2_inst;
    wire [31:0] cmt2_data;
    wire [4:0]  cmt2_rd;
    wire       excp_valid;
    wire [31:0] excp_pc;
    wire [5:0]  excp_code;
    reg  [3:0] write_id;

    always @(posedge aclk) begin
        if (!aresetn)
            write_id <= 4'b0;
        else if (awvalid && awready)
            write_id <= cpu_awid;
    end

    // CPUSTCore emits only single-beat or 16-beat transactions, both AXI3 legal.
    assign arlock  = 2'b00;
    assign arcache = 4'b0000;
    assign arprot  = 3'b000;
    assign awlock  = 2'b00;
    assign awcache = 4'b0000;
    assign awprot  = 3'b000;
    assign wid     = (awvalid && awready) ? cpu_awid : write_id;
    assign arlen   = cpu_arlen[3:0];
    assign awlen   = cpu_awlen[3:0];

    assign ws_valid           = 1'b0;
    assign rf_rdata           = 32'b0;
    assign debug0_wb_pc       = cmt0_pc;
    assign debug0_wb_rf_wen   = {4{cmt0_valid && cmt0_rd_valid}};
    assign debug0_wb_rf_wnum  = cmt0_rd;
    assign debug0_wb_rf_wdata = cmt0_data;
    assign debug0_wb_inst     = cmt0_inst;

    CPU u_cpustcore (
        .clock                 (aclk),
        .reset                 (~aresetn),
        .io_hardwareInterrupt  (intrpt),
        .io_axi_ar_valid       (arvalid),
        .io_axi_ar_ready       (arready),
        .io_axi_ar_addr        (araddr),
        .io_axi_ar_len         (cpu_arlen),
        .io_axi_ar_size        (arsize),
        .io_axi_ar_burst       (arburst),
        .io_axi_ar_id          (arid),
        .io_axi_r_valid        (rvalid),
        .io_axi_r_ready        (rready),
        .io_axi_r_data         (rdata),
        .io_axi_r_last         (rlast),
        .io_axi_r_resp         (rresp),
        .io_axi_r_id           (rid),
        .io_axi_aw_valid       (awvalid),
        .io_axi_aw_ready       (awready),
        .io_axi_aw_addr        (awaddr),
        .io_axi_aw_len         (cpu_awlen),
        .io_axi_aw_size        (awsize),
        .io_axi_aw_burst       (awburst),
        .io_axi_aw_id          (cpu_awid),
        .io_axi_w_valid        (wvalid),
        .io_axi_w_ready        (wready),
        .io_axi_w_data         (wdata),
        .io_axi_w_strb         (wstrb),
        .io_axi_w_last         (wlast),
        .io_axi_b_valid        (bvalid),
        .io_axi_b_ready        (bready),
        .io_axi_b_resp         (bresp),
        .io_axi_b_id           (bid),
        .io_cmt_0_valid        (cmt0_valid),
        .io_cmt_0_pc           (cmt0_pc),
        .io_cmt_0_inst         (cmt0_inst),
        .io_cmt_0_data         (cmt0_data),
        .io_cmt_0_rd_valid     (cmt0_rd_valid),
        .io_cmt_0_rd           (cmt0_rd),
        .io_cmt_0_exception    (),
        .io_cmt_0_exception_code(),
        .io_cmt_1_valid        (cmt1_valid),
        .io_cmt_1_pc           (cmt1_pc),
        .io_cmt_1_inst         (cmt1_inst),
        .io_cmt_1_data         (cmt1_data),
        .io_cmt_1_rd_valid     (cmt1_rd_valid),
        .io_cmt_1_rd           (cmt1_rd),
        .io_cmt_1_exception    (),
        .io_cmt_1_exception_code(),
        .io_cmt_2_valid        (cmt2_valid),
        .io_cmt_2_pc           (cmt2_pc),
        .io_cmt_2_inst         (cmt2_inst),
        .io_cmt_2_data         (cmt2_data),
        .io_cmt_2_rd_valid     (cmt2_rd_valid),
        .io_cmt_2_rd           (cmt2_rd),
        .io_cmt_2_exception    (),
        .io_cmt_2_exception_code(),
        .io_cmt_3_valid        (),
        .io_cmt_3_pc           (),
        .io_cmt_3_inst         (),
        .io_cmt_3_data         (),
        .io_cmt_3_rd_valid     (),
        .io_cmt_3_rd           (),
        .io_cmt_3_exception    (),
        .io_cmt_3_exception_code(),
        .io_cmt_rf_0           (),
        .io_cmt_rf_1           (),
        .io_cmt_rf_2           (),
        .io_cmt_rf_3           (),
        .io_cmt_rf_4           (),
        .io_cmt_rf_5           (),
        .io_cmt_rf_6           (),
        .io_cmt_rf_7           (),
        .io_cmt_rf_8           (),
        .io_cmt_rf_9           (),
        .io_cmt_rf_10          (),
        .io_cmt_rf_11          (),
        .io_cmt_rf_12          (),
        .io_cmt_rf_13          (),
        .io_cmt_rf_14          (),
        .io_cmt_rf_15          (),
        .io_cmt_rf_16          (),
        .io_cmt_rf_17          (),
        .io_cmt_rf_18          (),
        .io_cmt_rf_19          (),
        .io_cmt_rf_20          (),
        .io_cmt_rf_21          (),
        .io_cmt_rf_22          (),
        .io_cmt_rf_23          (),
        .io_cmt_rf_24          (),
        .io_cmt_rf_25          (),
        .io_cmt_rf_26          (),
        .io_cmt_rf_27          (),
        .io_cmt_rf_28          (),
        .io_cmt_rf_29          (),
        .io_cmt_rf_30          (),
        .io_cmt_rf_31          (),
        .io_cmt_tlbfill_valid  (),
        .io_cmt_tlbfill_idx    (),
        .io_excp_valid         (excp_valid),
        .io_excp_pc            (excp_pc),
        .io_excp_code          (excp_code)
    );

    assign awid = cpu_awid;

`ifdef DIFFTEST_EN
    DifftestInstrCommit difftest_commit_0 (
        .clock(aclk), .coreid(0), .index(0), .valid(cmt0_valid),
        .pc({32'b0, cmt0_pc}), .instr(cmt0_inst), .skip(0),
        .is_TLBFILL(0), .TLBFILL_index(0), .is_CNTinst(0),
        .timer_64_value(0), .wen(cmt0_rd_valid), .wdest({3'b0, cmt0_rd}),
        .wdata({32'b0, cmt0_data}), .csr_rstat(0), .csr_data(0)
    );

    DifftestInstrCommit difftest_commit_1 (
        .clock(aclk), .coreid(0), .index(1), .valid(cmt1_valid),
        .pc({32'b0, cmt1_pc}), .instr(cmt1_inst), .skip(0),
        .is_TLBFILL(0), .TLBFILL_index(0), .is_CNTinst(0),
        .timer_64_value(0), .wen(cmt1_rd_valid), .wdest({3'b0, cmt1_rd}),
        .wdata({32'b0, cmt1_data}), .csr_rstat(0), .csr_data(0)
    );

    DifftestInstrCommit difftest_commit_2 (
        .clock(aclk), .coreid(0), .index(2), .valid(cmt2_valid),
        .pc({32'b0, cmt2_pc}), .instr(cmt2_inst), .skip(0),
        .is_TLBFILL(0), .TLBFILL_index(0), .is_CNTinst(0),
        .timer_64_value(0), .wen(cmt2_rd_valid), .wdest({3'b0, cmt2_rd}),
        .wdata({32'b0, cmt2_data}), .csr_rstat(0), .csr_data(0)
    );

    DifftestExcpEvent difftest_exception (
        .clock(aclk), .coreid(0), .excp_valid(excp_valid), .eret(0),
        .intrNo(0), .cause({26'b0, excp_code}),
        .exceptionPC({32'b0, excp_pc}), .exceptionInst(0)
    );
`endif

endmodule

`default_nettype wire
