module drc_regmap #(
    // CSR 
    parameter DRC_BASE_ADDR     = 32'h8000_0000,
    // AXI4 Slave
    parameter S_DATA_W          = 32,
    parameter S_ADDR_W          = 32,
    parameter MST_ID_W          = 5,
    parameter ATX_LEN_W         = 8,
    parameter ATX_SIZE_W        = 3,
    parameter ATX_RESP_W        = 2,
    // Image configure
    parameter IMG_DIM_MAX       = 640,
    parameter IMG_DIM_W         = $clog2(IMG_DIM_MAX)
) (
    
    input                           aclk,
    input                           aresetn,
    // AXI4 Slave Interface            
    // -- AW channel         
    input   [MST_ID_W-1:0]          s_awid_i,
    input   [S_ADDR_W-1:0]          s_awaddr_i,
    input   [1:0]                   s_awburst_i,
    input   [ATX_LEN_W-1:0]         s_awlen_i,
    input                           s_awvalid_i,
    output                          s_awready_o,
    // -- W channel          
    input   [S_DATA_W-1:0]          s_wdata_i,
    input                           s_wlast_i,
    input                           s_wvalid_i,
    output                          s_wready_o,
    // -- B channel          
    output  [MST_ID_W-1:0]          s_bid_o,
    output  [ATX_RESP_W-1:0]        s_bresp_o,
    output                          s_bvalid_o,
    input                           s_bready_i,
    // -- AR channel         
    input   [MST_ID_W-1:0]          s_arid_i,
    input   [S_ADDR_W-1:0]          s_araddr_i,
    input   [1:0]                   s_arburst_i,
    input   [ATX_LEN_W-1:0]         s_arlen_i,
    input                           s_arvalid_i,
    output                          s_arready_o,
    // -- R channel          
    output  [MST_ID_W-1:0]          s_rid_o,
    output  [S_DATA_W-1:0]          s_rdata_o,
    output  [ATX_RESP_W-1:0]        s_rresp_o,
    output                          s_rlast_o,
    output                          s_rvalid_o,
    input                           s_rready_i,
    // Camera RX CSRs
    output                          cam_rx_en,      // Enable camera RX
    output                          cam_pwdn,       // Power down camera
    output  [1:0]                   cam_rx_mode,    // Camera RX mode
    output                          cam_rx_start,   // Start camera RX
    input                           cam_rx_start_qed,// Start signal is queued
    input   [2:0]                   cam_rx_state,   // Camera RX state
    input   [IMG_DIM_W*2-1:0]       cam_rx_len,     // Camera RX pixel length
    // Camera interrupt/trap
    output                          irq_msk_frm_comp, // IRQ mask Frame completion
    output                          irq_msk_frm_err,  // IRQ mask Frame error
    // Image format
    output  [IMG_DIM_W-1:0]         img_width,      // Image width
    output  [IMG_DIM_W-1:0]         img_height      // Image height

);
    // Local paramters
    localparam RW_REG_ADDR      = DRC_BASE_ADDR + 32'h0000_0000;    // Read-Write registers
    localparam RW1S_REG_ADDR    = DRC_BASE_ADDR + 32'h0000_0010;    // Write-to-set registers
    localparam RO_REG_ADDR      = DRC_BASE_ADDR + 32'h0000_0020;    // Read only registers
    localparam RW_REG_NUM       = 16;     // 16 RW registers (use 6/16 registers) - Detail in specification
    localparam RW1S_REG_NUM     = 1;      // 1/1 RW1S register - Detail in specification
    localparam RO_REG_NUM       = 16;     // 16 RO registers (use 2/16 registers) - Detail in specification
    localparam RW1S_REG_OFFSET  = 1;

    // Internal variables
    genvar i;

    // Internal signals
    wire [S_DATA_W*RW_REG_NUM-1:0]      rw_reg_flat;
    wire [S_DATA_W*RO_REG_NUM-1:0]      ro_reg_flat;
    wire [S_DATA_W*RW1S_REG_NUM-1:0]    rw1s_rd_dat_flat;
    wire [S_DATA_W-1:0]     rw_reg      [0:RW_REG_NUM-1];
    wire [S_DATA_W-1:0]     ro_reg      [0:RO_REG_NUM-1];
    wire [S_DATA_W-1:0]     rw1s_rd_dat [0:RW1S_REG_NUM-1];
    wire [RW1S_REG_NUM-1:0] rw1s_rd_vld;
    wire [RW1S_REG_NUM-1:0] rw1s_rd_rdy;

    // Module instantiation
    axi4_ctrl #(
        .AXI4_CTRL_CONF     (1),    // RW register 
        .AXI4_CTRL_STAT     (1),    // RO register
        .AXI4_CTRL_MEM      (0),
        .AXI4_CTRL_WR_ST    (1),    // RW1S register
        .AXI4_CTRL_RD_ST    (0),
        .DATA_W             (S_DATA_W),
        .ADDR_W             (S_ADDR_W),
        .MST_ID_W           (MST_ID_W),
        .CONF_BASE_ADDR     (RW_REG_ADDR),
        .CONF_OFFSET        (8'h01),// Word-access
        .CONF_DATA_W        (S_DATA_W),
        .CONF_REG_NUM       (RW_REG_NUM),
        .STAT_BASE_ADDR     (RO_REG_ADDR),
        .STAT_OFFSET        (8'h01),// Word-access
        .STAT_REG_NUM       (RO_REG_NUM),
        .ST_WR_BASE_ADDR    (RW1S_REG_ADDR),
        .ST_WR_OFFSET       (RW1S_REG_OFFSET),
        .ST_WR_FIFO_NUM     (RW1S_REG_NUM),
        .ST_WR_FIFO_DEPTH   (2)
    ) ac (
        .clk                (aclk),
        .rst_n              (aresetn),
        .m_awid_i           (s_awid_i),
        .m_awaddr_i         (s_awaddr_i),
        .m_awburst_i        (s_awburst_i),
        .m_awlen_i          (s_awlen_i),
        .m_awvalid_i        (s_awvalid_i),
        .m_wdata_i          (s_wdata_i),
        .m_wlast_i          (s_wlast_i),
        .m_wvalid_i         (s_wvalid_i),
        .m_bready_i         (s_bready_i),
        .m_arid_i           (s_arid_i),
        .m_araddr_i         (s_araddr_i),
        .m_arburst_i        (s_arburst_i),
        .m_arlen_i          (s_arlen_i),
        .m_arvalid_i        (s_arvalid_i),
        .m_rready_i         (s_rready_i),
        .stat_reg_i         (ro_reg_flat),
        .mem_wr_rdy_i       (),
        .mem_rd_data_i      (),
        .mem_rd_rdy_i       (),
        .wr_st_rd_vld_i     (rw1s_rd_vld),
        .rd_st_wr_data_i    (),
        .rd_st_wr_vld_i     (),
        .m_awready_o        (s_awready_o),
        .m_wready_o         (s_wready_o),
        .m_bid_o            (s_bid_o),
        .m_bresp_o          (s_bresp_o),
        .m_bvalid_o         (s_bvalid_o),
        .m_arready_o        (s_arready_o),
        .m_rid_o            (s_rid_o),
        .m_rdata_o          (s_rdata_o),
        .m_rresp_o          (s_rresp_o),
        .m_rlast_o          (s_rlast_o),
        .m_rvalid_o         (s_rvalid_o),
        .conf_reg_o         (rw_reg_flat),
        .mem_wr_data_o      (),
        .mem_wr_addr_o      (),
        .mem_wr_vld_o       (),
        .mem_rd_addr_o      (),
        .mem_rd_vld_o       (),
        .wr_st_rd_data_o    (rw1s_rd_dat_flat),
        .wr_st_rd_rdy_o     (rw1s_rd_rdy),
        .rd_st_wr_rdy_o     ()
    );
    // Registers Mapping
    // -- RW registers (Base 0x0000)
    assign cam_rx_en        = rw_reg[ 'h00 ][0];
    assign cam_pwdn         = rw_reg[ 'h01 ][0];
    assign cam_rx_mode      = rw_reg[ 'h02 ][1:0];

    assign irq_msk_frm_comp = rw_reg[ 'h03 ][0];
    assign irq_msk_frm_err  = rw_reg[ 'h03 ][1];

    assign img_width        = rw_reg[ 'h04 ][IMG_DIM_W-1:0];
    assign img_height       = rw_reg[ 'h05 ][IMG_DIM_W-1:0];
    

    // -- RO registers  (Base 0x0020)
    assign ro_reg[ 'h00 ]   = {{(S_DATA_W-2){1'b0}},            cam_rx_state};
    assign ro_reg[ 'h01 ]   = {{(S_DATA_W-IMG_DIM_W*2){1'b0}},  cam_rx_len};

    // -- RW1S registers (Base 0x0010)
    assign rw1s_rd_vld      = cam_rx_start_qed;
    assign cam_rx_start     = rw1s_rd_rdy;

    // Deflatten signals
generate
    for (i = 0; i < RW_REG_NUM; i = i + 1) begin    : RW_REG_FLAT
        assign rw_reg[i] = rw_reg_flat[(i+1)*S_DATA_W-1 -: S_DATA_W];
    end

    for (i = 0; i < RO_REG_NUM; i = i + 1) begin    : RO_REG_FLAT
        assign ro_reg_flat[(i+1)*S_DATA_W-1 -: S_DATA_W] = ro_reg[i];
    end

    for (i = 0; i < RW1S_REG_NUM; i = i + 1) begin  : RW1S_REG_FLAT
        assign rw1s_rd_dat[i] = rw1s_rd_dat_flat[(i+1)*S_DATA_W-1 -: S_DATA_W];
    end
endgenerate 

endmodule