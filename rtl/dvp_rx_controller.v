module dvp_rx_controller #(
    // System
    parameter INTERNAL_CLK      = 125_000_000,
    // Memory Mapping
    parameter DRC_BASE_ADDR     = 32'h8000_0000,    // Memory mapping - BASE
    parameter DMA_SEL_BIT       = 28,   // Select bit of DMA -> ADDR[DMA_SEL_BIT]
    // DMA AXI interface
    parameter DMA_DATA_W        = 256,
    parameter DMA_ADDR_W        = 32,
    // Master AXI interface
    parameter S_DATA_W          = 32,
    parameter S_ADDR_W          = 32,
    parameter MST_ID_W          = 5,
    parameter ATX_LEN_W         = 8,
    parameter ATX_SIZE_W        = 3,
    parameter ATX_RESP_W        = 2,
    // DVP configuration
    parameter DVP_DATA_W        = 8,
    parameter DVP_FIFO_D        = 4,   // DVP FIFO depth 
    parameter DVP_CAPTURE_TYPE  = "PCLK_EDGE", // "ASYNC_FIFO": Use asynchronous FIFO to capture data || "PCLK_EDGE": Synchronize rising egde of PCLK to capture the data
    /*
    + With the "ASYNC_FIFO" capture type, the controller can support higher frequency on the DVP interface. However, you should ONLY use PCLK as an external clock for the controller if you make sure that the PCLK connection is dedicated for a clock line.
    + With the "PCLK_EDGE" capture type, the controller operates more stably but at lower speed than the "ASYNC_FIFO" capture type.
    */
    // Image 
    parameter PXL_GRAYSCALE     = 1,    // Resize (Pixel Grayscale) - 0: DISABLE || 1 : ENABLE 
    parameter FRM_DOWNSCALE     = 1,    // Resize (Frame Downscale) - 0: DISABLE || 1 : ENABLE
    parameter FRM_COL_NUM       = 640,  // Maximum columns in 1 frame
    parameter FRM_ROW_NUM       = 480,  // Maximum rows in 1 frame
    parameter DOWNSCALE_TYPE    = "AVR-POOLING"  // Downscale Type - "AVR-POOLING": Average Pooling || "MAX-POOLING": Max pooling
) (
    input                       clk,
    input                       rst_n,
    // DVP RX interface
    input   [DVP_DATA_W-1:0]    dvp_d_i,
    input                       dvp_href_i,
    input                       dvp_vsync_i,
    input                       dvp_hsync_i,
    input                       dvp_pclk_i,
    output                      dvp_xclk_o,
    output                      dvp_pwdn_o,
    // AXI4 interface (configuration)
    // -- AW channel
    input   [MST_ID_W-1:0]      s_awid_i,
    input   [S_ADDR_W-1:0]      s_awaddr_i,
    input   [1:0]               s_awburst_i,
    input   [ATX_LEN_W-1:0]     s_awlen_i,
    input                       s_awvalid_i,
    output                      s_awready_o,
    // -- W channel
    input   [S_DATA_W-1:0]      s_wdata_i,
    input                       s_wlast_i,
    input                       s_wvalid_i,
    output                      s_wready_o,
    // -- B channel
    output  [MST_ID_W-1:0]      s_bid_o,
    output  [ATX_RESP_W-1:0]    s_bresp_o,
    output                      s_bvalid_o,
    input                       s_bready_i,
    // -- AR channel
    input   [MST_ID_W-1:0]      s_arid_i,
    input   [S_ADDR_W-1:0]      s_araddr_i,
    input   [1:0]               s_arburst_i,
    input   [ATX_LEN_W-1:0]     s_arlen_i,
    input                       s_arvalid_i,
    output                      s_arready_o,
    // -- R channel
    output  [MST_ID_W-1:0]      s_rid_o,
    output  [S_DATA_W-1:0]      s_rdata_o,
    output  [ATX_RESP_W-1:0]    s_rresp_o,
    output                      s_rlast_o,
    output                      s_rvalid_o,
    input                       s_rready_i,
    // AXI4 interface (pixels streaming)
    // -- AW channel         
    output  [MST_ID_W-1:0]      m_awid_o,
    output  [DMA_ADDR_W-1:0]    m_awaddr_o,
    output  [ATX_LEN_W-1:0]     m_awlen_o,
    output  [1:0]               m_awburst_o,
    output                      m_awvalid_o,
    input                       m_awready_i,
    // -- W channel          
    output  [DMA_DATA_W-1:0]    m_wdata_o,
    output                      m_wlast_o,
    output                      m_wvalid_o,
    input                       m_wready_i,
    // -- B channel
    input   [MST_ID_W-1:0]      m_bid_i,
    input   [ATX_RESP_W-1:0]    m_bresp_i,
    input                       m_bvalid_i,
    output                      m_bready_o,
    // Interrupt and Trap
    output                      drc_irq, // Caused by frame completion
    output                      drc_trap,// Caused by pixel misalignment
    output                      dma_irq, // Caused by frame transaction completion
    output                      dma_trap // Caused by wrong address mapping
);
    // Local parameters 
    localparam RGB_PXL_W        = 16;
    localparam GS_PXL_W         = 8;
    localparam PXL_INFO_W       = 1 + 1 + DVP_DATA_W; // VSYNC + HSYNC + PIXEL_W
    localparam IMG_DIM_MAX      = (FRM_COL_NUM > FRM_ROW_NUM) ? FRM_COL_NUM : FRM_ROW_NUM;
    localparam IMG_DIM_W        = $clog2(IMG_DIM_MAX);
    localparam PROC_PXL_W       = PXL_GRAYSCALE ? GS_PXL_W : RGB_PXL_W; // Processed Pixel width
    localparam DMA_BASE_ADDR    = DRC_BASE_ADDR |  (1 << DMA_SEL_BIT);  // DMA Memory Address:    ADDR[DMA_SEL_BIT] = 1
    localparam DRM_BASE_ADDR    = DRC_BASE_ADDR & ~(1 << DMA_SEL_BIT);  // RegMap Memory Address: ADDR[DMA_SEL_BIT] = 0
    // Internal signal
    // Camera RX CSRs
    wire                        cam_rx_en;      // Enable camera RX
    wire                        cam_pwdn;       // Power down camera
    wire    [1:0]               cam_rx_mode;    // Camera RX mode
    wire                        cam_rx_start;   // Start camera RX
    wire                        cam_rx_start_qed; // Start signal is queued
    wire    [2:0]               cam_rx_state;   // Camera RX state
    wire    [IMG_DIM_W*2-1:0]   cam_rx_len;     // Camera RX pixel length
    wire                        irq_msk_frm_comp; // IRQ mask Frame completion
    wire                        irq_msk_frm_err;  // IRQ mask Frame error
    wire    [IMG_DIM_W-1:0]     img_width;      // Image width
    wire    [IMG_DIM_W-1:0]     img_height;     // Image height
    // Interrupt and Trap
    wire                        int_dma_irq;
    wire                        int_dma_trap;
    // Pixel FIFO -> DRC Control State
    wire    [PXL_INFO_W-1:0]    pxl_info_dat;
    wire                        pxl_info_vld;
    wire                        pxl_info_rdy;
    // DRC Control State -> Resizer
    wire    [RGB_PXL_W-1:0]     rgb_pxl_dat;
    wire                        rgb_pxl_last;
    wire                        rgb_pxl_vld;
    wire                        rgb_pxl_rdy;
    // Resizer -> Memory Aligner
    wire    [PROC_PXL_W-1:0]    proc_pxl_dat;
    wire                        proc_pxl_last;
    wire                        proc_pxl_vld;
    wire                        proc_pxl_rdy;
    // Memory Aligner -> DMA
    wire [MST_ID_W-1:0]         int_tid;    
    wire                        int_tdest;
    wire [DMA_DATA_W-1:0]       int_tdata;
    wire                        int_tvalid;
    wire [(DMA_DATA_W/8)-1:0]   int_tkeep;  // All bytes is valid
    wire [(DMA_DATA_W/8)-1:0]   int_tstrb;  // All bytes is valid
    wire                        int_tlast;  // Assert when last pixel of the frame is sent
    wire                        int_tready;
    // AXI Dispatch -> DMA & DRC
    wire    [MST_ID_W-1:0]      s_awid      [0:1];
    wire    [S_ADDR_W-1:0]      s_awaddr    [0:1];
    wire    [1:0]               s_awburst   [0:1];
    wire    [ATX_LEN_W-1:0]     s_awlen     [0:1];
    wire                        s_awvalid   [0:1];
    wire                        s_awready   [0:1];
    wire    [S_DATA_W-1:0]      s_wdata     [0:1];
    wire                        s_wlast     [0:1];
    wire                        s_wvalid    [0:1];
    wire                        s_wready    [0:1];
    wire    [MST_ID_W-1:0]      s_bid       [0:1];
    wire    [ATX_RESP_W-1:0]    s_bresp     [0:1];
    wire                        s_bvalid    [0:1];
    wire                        s_bready    [0:1];
    wire    [MST_ID_W-1:0]      s_arid      [0:1];
    wire    [S_ADDR_W-1:0]      s_araddr    [0:1];
    wire    [1:0]               s_arburst   [0:1];
    wire    [ATX_LEN_W-1:0]     s_arlen     [0:1];
    wire                        s_arvalid   [0:1];
    wire                        s_arready   [0:1];
    wire    [MST_ID_W-1:0]      s_rid       [0:1];
    wire    [S_DATA_W-1:0]      s_rdata     [0:1];
    wire    [ATX_RESP_W-1:0]    s_rresp     [0:1];
    wire                        s_rlast     [0:1];
    wire                        s_rvalid    [0:1];
    wire                        s_rready    [0:1];
    // Flattened signals
    wire    [2*MST_ID_W-1:0]    flat_s_awid;
    wire    [2*S_ADDR_W-1:0]    flat_s_awaddr;
    wire    [3:0]               flat_s_awburst;
    wire    [2*ATX_LEN_W-1:0]   flat_s_awlen;
    wire    [1:0]               flat_s_awvalid;
    wire    [1:0]               flat_s_awready;
    wire    [2*S_DATA_W-1:0]    flat_s_wdata;
    wire    [1:0]               flat_s_wlast;
    wire    [1:0]               flat_s_wvalid;
    wire    [1:0]               flat_s_wready;
    wire    [2*MST_ID_W-1:0]    flat_s_bid;
    wire    [2*ATX_RESP_W-1:0]  flat_s_bresp;
    wire    [1:0]               flat_s_bvalid;
    wire    [1:0]               flat_s_bready;
    wire    [2*MST_ID_W-1:0]    flat_s_arid;
    wire    [2*S_ADDR_W-1:0]    flat_s_araddr;
    wire    [3:0]               flat_s_arburst;
    wire    [2*ATX_LEN_W-1:0]   flat_s_arlen;
    wire    [1:0]               flat_s_arvalid;
    wire    [1:0]               flat_s_arready;
    wire    [2*MST_ID_W-1:0]    flat_s_rid;
    wire    [2*S_DATA_W-1:0]    flat_s_rdata;
    wire    [2*ATX_RESP_W-1:0]  flat_s_rresp;
    wire    [1:0]               flat_s_rlast;
    wire    [1:0]               flat_s_rvalid;
    wire    [1:0]               flat_s_rready;


    // Module instantiation
    // -- XCLK Generator
    drc_xclk_gen #(
        .INTL_CLK_PERIOD    (INTERNAL_CLK)
    ) xg (
        .clk                (clk),
        .rst_n              (rst_n),
        .cam_rx_en          (cam_rx_en),
        .cam_pwdn           (cam_pwdn),
        .dvp_xclk_o         (dvp_xclk_o),
        .dvp_pwdn_o         (dvp_pwdn_o)
    );
    // -- DVP Data FIFO
    drc_dvp_data_fifo #(
        .DVP_DATA_W         (DVP_DATA_W),
        .DVP_CAPTURE_TYPE   (DVP_CAPTURE_TYPE),
        .PXL_INFO_W         (PXL_INFO_W),
        .PXL_FIFO_D         (DVP_FIFO_D)
    ) ddf (
        .clk                (clk),
        .rst_n              (rst_n),
        .cam_rx_en          (cam_rx_en),
        .dvp_d_i            (dvp_d_i),
        .dvp_href_i         (dvp_href_i),
        .dvp_vsync_i        (dvp_vsync_i),
        .dvp_hsync_i        (dvp_hsync_i),
        .dvp_pclk_i         (dvp_pclk_i),
        .pxl_info_dat       (pxl_info_dat),
        .pxl_info_vld       (pxl_info_vld),
        .pxl_info_rdy       (pxl_info_rdy)
    );
    // Control State
    drc_ctrl_state #(
        .DVP_DATA_W         (DVP_DATA_W),
        .PXL_INFO_W         (PXL_INFO_W),
        .RGB_PXL_W          (RGB_PXL_W),
        .IMG_DIM_MAX        (IMG_DIM_MAX),
        .IMG_DIM_W          (IMG_DIM_W)
    ) cs (
        .clk                (clk),
        .rst_n              (rst_n),
        .bwd_pxl_info_dat   (pxl_info_dat),
        .bwd_pxl_info_vld   (pxl_info_vld),
        .bwd_pxl_info_rdy   (pxl_info_rdy),

        .fwd_pxl_dat        (rgb_pxl_dat),
        .fwd_pxl_last       (rgb_pxl_last),
        .fwd_pxl_vld        (rgb_pxl_vld),
        .fwd_pxl_rdy        (rgb_pxl_rdy),

        .cam_rx_en          (cam_rx_en),
        .cam_rx_mode        (cam_rx_mode),
        .cam_rx_start       (cam_rx_start),
        .cam_rx_start_qed   (cam_rx_start_qed),
        .cam_rx_state       (cam_rx_state),
        .cam_rx_len         (cam_rx_len),
        .irq_msk_frm_comp   (irq_msk_frm_comp),
        .irq_msk_frm_err    (irq_msk_frm_err),
        .img_width          (img_width),
        .img_height         (img_height),
        .irq                (drc_irq),
        .trap               (drc_trap)
    );
    // -- DRC Register Map
    drc_regmap #(
        .DRC_BASE_ADDR      (DRM_BASE_ADDR),
        .S_DATA_W           (S_DATA_W),
        .S_ADDR_W           (S_ADDR_W),
        .MST_ID_W           (MST_ID_W),
        .ATX_LEN_W          (ATX_LEN_W),
        .ATX_SIZE_W         (ATX_SIZE_W),
        .ATX_RESP_W         (ATX_RESP_W),
        .IMG_DIM_MAX        (IMG_DIM_MAX),
        .IMG_DIM_W          (IMG_DIM_W)
    ) drm (
        .aclk               (clk),
        .aresetn            (rst_n),
        .s_awid_i           (s_awid[0]),
        .s_awaddr_i         (s_awaddr[0]),
        .s_awburst_i        (s_awburst[0]),
        .s_awlen_i          (s_awlen[0]),
        .s_awvalid_i        (s_awvalid[0]),
        .s_awready_o        (s_awready[0]),
        .s_wdata_i          (s_wdata[0]),
        .s_wlast_i          (s_wlast[0]),
        .s_wvalid_i         (s_wvalid[0]),
        .s_wready_o         (s_wready[0]),
        .s_bid_o            (s_bid[0]),
        .s_bresp_o          (s_bresp[0]),
        .s_bvalid_o         (s_bvalid[0]),
        .s_bready_i         (s_bready[0]),
        .s_arid_i           (s_arid[0]),
        .s_araddr_i         (s_araddr[0]),
        .s_arburst_i        (s_arburst[0]),
        .s_arlen_i          (s_arlen[0]),
        .s_arvalid_i        (s_arvalid[0]),
        .s_arready_o        (s_arready[0]),
        .s_rid_o            (s_rid[0]),
        .s_rdata_o          (s_rdata[0]),
        .s_rresp_o          (s_rresp[0]),
        .s_rlast_o          (s_rlast[0]),
        .s_rvalid_o         (s_rvalid[0]),
        .s_rready_i         (s_rready[0]),
        .cam_rx_en          (cam_rx_en),
        .cam_pwdn           (cam_pwdn),
        .cam_rx_mode        (cam_rx_mode),
        .cam_rx_start       (cam_rx_start),
        .cam_rx_start_qed   (cam_rx_start_qed),
        .cam_rx_state       (cam_rx_state),
        .cam_rx_len         (cam_rx_len),
        .irq_msk_frm_comp   (irq_msk_frm_comp),
        .irq_msk_frm_err    (irq_msk_frm_err),
        .img_width          (img_width),
        .img_height         (img_height)
    );
    // -- Resizer (Optional) 
    drc_resizer #(
        .PXL_GRAYSCALE      (PXL_GRAYSCALE),
        .FRM_DOWNSCALE      (FRM_DOWNSCALE),
        .FRM_COL_NUM        (FRM_COL_NUM),
        .FRM_ROW_NUM        (FRM_ROW_NUM),
        .DOWNSCALE_TYPE     (DOWNSCALE_TYPE),
        .RGB_PXL_W          (RGB_PXL_W),
        .GS_PXL_W           (GS_PXL_W)
    ) dr (
        .clk                (clk),
        .rst_n              (rst_n),
        .i_pxl_dat          (rgb_pxl_dat),
        .i_pxl_last         (rgb_pxl_last),
        .i_pxl_vld          (rgb_pxl_vld),
        .i_pxl_rdy          (rgb_pxl_rdy),
        .o_pxl_dat          (proc_pxl_dat),
        .o_pxl_last         (proc_pxl_last),
        .o_pxl_vld          (proc_pxl_vld),
        .o_pxl_rdy          (proc_pxl_rdy)
    );
    // -- Memory Aligner
    drc_mem_aligner #(
        .I_PXL_W            (PROC_PXL_W),
        .AXIS_DATA_W        (DMA_DATA_W),
        .AXIS_TID_W         (MST_ID_W)
    ) ma (
        .aclk               (clk),
        .aresetn            (rst_n),
        .i_pxl_dat          (proc_pxl_dat),
        .i_pxl_last         (proc_pxl_last),
        .i_pxl_vld          (proc_pxl_vld),
        .i_pxl_rdy          (proc_pxl_rdy),
        .tid                (int_tid),
        .tdest              (int_tdest),
        .tdata              (int_tdata),
        .tvalid             (int_tvalid),
        .tkeep              (int_tkeep),
        .tstrb              (int_tstrb),
        .tlast              (int_tlast),
        .tready             (int_tready)
    );
    // -- Internal DMA
    axi_dma #(
        .DMA_BASE_ADDR      (DMA_BASE_ADDR),
        .DMA_CHN_NUM        (1),
        .DMA_LENGTH_W       (16),
        .DMA_DESC_DEPTH     (2),
        .DMA_CHN_ARB_W      (3),
        .ROB_EN             (0),
        .DESC_QUEUE_TYPE    (),
        .SRC_IF_TYPE        ("AXIS"),   // Source: AXI-Stream
        .SRC_ADDR_W         (),
        .SRC_TDEST_W        (1),
        .ATX_SRC_DATA_W     (DMA_DATA_W),
        .DST_IF_TYPE        ("AXI4"),   // Destination: AXI4
        .DST_ADDR_W         (DMA_ADDR_W),
        .DST_TDEST_W        (),
        .ATX_DST_DATA_W     (DMA_DATA_W),
        .S_DATA_W           (S_DATA_W),
        .S_ADDR_W           (S_ADDR_W),
        .MST_ID_W           (MST_ID_W),
        .ATX_LEN_W          (ATX_LEN_W),
        .ATX_SIZE_W         (ATX_SIZE_W),
        .ATX_RESP_W         (ATX_RESP_W),
        .ATX_SRC_BYTE_AMT   (),
        .ATX_DST_BYTE_AMT   (),
        .ATX_NUM_OSTD       (),
        .ATX_INTL_DEPTH     (2)
    ) idma (
        .aclk               (clk),
        .aresetn            (rst_n),
        .s_awid_i           (s_awid[1]),
        .s_awaddr_i         (s_awaddr[1]),
        .s_awburst_i        (s_awburst[1]),
        .s_awlen_i          (s_awlen[1]),
        .s_awvalid_i        (s_awvalid[1]),
        .s_awready_o        (s_awready[1]),
        .s_wdata_i          (s_wdata[1]),
        .s_wlast_i          (s_wlast[1]),
        .s_wvalid_i         (s_wvalid[1]),
        .s_wready_o         (s_wready[1]),
        .s_bid_o            (s_bid[1]),
        .s_bresp_o          (s_bresp[1]),
        .s_bvalid_o         (s_bvalid[1]),
        .s_bready_i         (s_bready[1]),
        .s_arid_i           (s_arid[1]),
        .s_araddr_i         (s_araddr[1]),
        .s_arburst_i        (s_arburst[1]),
        .s_arlen_i          (s_arlen[1]),
        .s_arvalid_i        (s_arvalid[1]),
        .s_arready_o        (s_arready[1]),
        .s_rid_o            (s_rid[1]),
        .s_rdata_o          (s_rdata[1]),
        .s_rresp_o          (s_rresp[1]),
        .s_rlast_o          (s_rlast[1]),
        .s_rvalid_o         (s_rvalid[1]),
        .s_rready_i         (s_rready[1]),
        .m_arid_o           (),
        .m_araddr_o         (),
        .m_arlen_o          (),
        .m_arburst_o        (),
        .m_arvalid_o        (),
        .m_arready_i        (),
        .m_rid_i            (),
        .m_rdata_i          (),
        .m_rresp_i          (),
        .m_rlast_i          (),
        .m_rvalid_i         (),
        .m_rready_o         (),
        .s_tid_i            (int_tid),
        .s_tdest_i          (int_tdest),
        .s_tdata_i          (int_tdata),
        .s_tkeep_i          (int_tkeep),
        .s_tstrb_i          (int_tstrb),
        .s_tlast_i          (int_tlast),
        .s_tvalid_i         (int_tvalid),
        .s_tready_o         (int_tready),
        .m_awid_o           (m_awid_o),
        .m_awaddr_o         (m_awaddr_o),
        .m_awlen_o          (m_awlen_o),
        .m_awburst_o        (m_awburst_o),
        .m_awvalid_o        (m_awvalid_o),
        .m_awready_i        (m_awready_i),
        .m_wdata_o          (m_wdata_o),
        .m_wlast_o          (m_wlast_o),
        .m_wvalid_o         (m_wvalid_o),
        .m_wready_i         (m_wready_i),
        .m_bid_i            (m_bid_i),
        .m_bresp_i          (m_bresp_i),
        .m_bvalid_i         (m_bvalid_i),
        .m_bready_o         (m_bready_o),
        .m_tid_o            (),
        .m_tdest_o          (),
        .m_tdata_o          (),
        .m_tkeep_o          (),
        .m_tstrb_o          (),
        .m_tlast_o          (),
        .m_tvalid_o         (),
        .m_tready_i         (),
        .irq                (int_dma_irq),
        .trap               (int_dma_trap)
    ); 
    // -- AXI Dispatch
    axi_interconnect #(
        .MST_AMT            (1),
        .SLV_AMT            (1 + 1),    // DMA_RegMap + DRC_RegMap    
        .OUTSTANDING_AMT    (2),
        .MST_WEIGHT         (),
        .MST_ID_W           (),
        .SLV_ID_W           (),
        .DATA_WIDTH         (S_DATA_W),
        .ADDR_WIDTH         (S_ADDR_W),
        .TRANS_MST_ID_W     (MST_ID_W),
        .TRANS_SLV_ID_W     (),
        .TRANS_BURST_W      (2),
        .TRANS_DATA_LEN_W   (ATX_LEN_W),
        .TRANS_DATA_SIZE_W  (ATX_SIZE_W),
        .TRANS_WR_RESP_W    (ATX_RESP_W),
        .SLV_ID_MSB_IDX     (DMA_SEL_BIT),   // MSB: 28
        .SLV_ID_LSB_IDX     (DMA_SEL_BIT),   // LSB: 28
        .DSP_RDATA_DEPTH    (2)         // == Interleaving depth
    ) ad (
        .ACLK_i             (clk),
        .ARESETn_i          (rst_n),
        .m_AWID_i           (s_awid_i),
        .m_AWADDR_i         (s_awaddr_i),
        .m_AWBURST_i        (s_awburst_i),
        .m_AWLEN_i          (s_awlen_i),
        .m_AWSIZE_i         (),
        .m_AWVALID_i        (s_awvalid_i),
        .m_AWREADY_o        (s_awready_o),
        .m_WDATA_i          (s_wdata_i),
        .m_WLAST_i          (s_wlast_i),
        .m_WVALID_i         (s_wvalid_i),
        .m_WREADY_o         (s_wready_o),
        .m_BID_o            (s_bid_o),
        .m_BRESP_o          (s_bresp_o),
        .m_BVALID_o         (s_bvalid_o),
        .m_BREADY_i         (s_bready_i),
        .m_ARID_i           (s_arid_i),
        .m_ARADDR_i         (s_araddr_i),
        .m_ARBURST_i        (s_arburst_i),
        .m_ARLEN_i          (s_arlen_i),
        .m_ARSIZE_i         (),
        .m_ARVALID_i        (s_arvalid_i),
        .m_ARREADY_o        (s_arready_o),
        .m_RID_o            (s_rid_o),
        .m_RDATA_o          (s_rdata_o),
        .m_RRESP_o          (s_rresp_o),
        .m_RLAST_o          (s_rlast_o),
        .m_RVALID_o         (s_rvalid_o),
        .m_RREADY_i         (s_rready_i),

        .s_AWREADY_i        (flat_s_awready),
        .s_WREADY_i         (flat_s_wready),
        .s_BID_i            (flat_s_bid),
        .s_BRESP_i          (flat_s_bresp),
        .s_BVALID_i         (flat_s_bvalid),
        .s_ARREADY_i        (flat_s_arready),
        .s_RID_i            (flat_s_rid),
        .s_RDATA_i          (flat_s_rdata),
        .s_RRESP_i          (flat_s_rresp),
        .s_RLAST_i          (flat_s_rlast),
        .s_RVALID_i         (flat_s_rvalid),
        .s_AWID_o           (flat_s_awid),
        .s_AWADDR_o         (flat_s_awaddr),
        .s_AWBURST_o        (flat_s_awburst),
        .s_AWLEN_o          (flat_s_awlen),
        .s_AWSIZE_o         (),
        .s_AWVALID_o        (flat_s_awvalid),
        .s_WDATA_o          (flat_s_wdata),
        .s_WLAST_o          (flat_s_wlast),
        .s_WVALID_o         (flat_s_wvalid),
        .s_BREADY_o         (flat_s_bready),
        .s_ARID_o           (flat_s_arid),
        .s_ARADDR_o         (flat_s_araddr),
        .s_ARBURST_o        (flat_s_arburst),
        .s_ARLEN_o          (flat_s_arlen),
        .s_ARSIZE_o         (),
        .s_ARVALID_o        (flat_s_arvalid),
        .s_RREADY_o         (flat_s_rready)
    );
    assign dma_irq  = int_dma_irq;
    assign dma_trap = int_dma_trap;
    // Flatten
    assign {s_awid[1],       s_awid[0]}     = flat_s_awid;
    assign {s_awaddr[1],     s_awaddr[0]}   = flat_s_awaddr;
    assign {s_awburst[1],    s_awburst[0]}  = flat_s_awburst;
    assign {s_awlen[1],      s_awlen[0]}    = flat_s_awlen;
    assign {s_awvalid[1],    s_awvalid[0]}  = flat_s_awvalid;
    assign flat_s_awready                   = {s_awready[1],    s_awready[0]};

    assign {s_wdata[1],      s_wdata[0]}    = flat_s_wdata;
    assign {s_wlast[1],      s_wlast[0]}    = flat_s_wlast;
    assign {s_wvalid[1],     s_wvalid[0]}   = flat_s_wvalid;
    assign flat_s_wready                    = {s_wready[1],     s_wready[0]};
    
    assign flat_s_bid                       = {s_bid[1],        s_bid[0]};
    assign flat_s_bresp                     = {s_bresp[1],      s_bresp[0]};
    assign flat_s_bvalid                    = {s_bvalid[1],     s_bvalid[0]};
    assign {s_bready[1],     s_bready[0]}   = flat_s_bready;

    assign {s_arid[1],       s_arid[0]}     = flat_s_arid;
    assign {s_araddr[1],     s_araddr[0]}   = flat_s_araddr;
    assign {s_arburst[1],    s_arburst[0]}  = flat_s_arburst;
    assign {s_arlen[1],      s_arlen[0]}    = flat_s_arlen;
    assign {s_arvalid[1],    s_arvalid[0]}  = flat_s_arvalid;
    assign flat_s_arready                   = {s_arready[1],    s_arready[0]};

    assign flat_s_rid                       = {s_rid[1],        s_rid[0]};
    assign flat_s_rdata                     = {s_rdata[1],      s_rdata[0]};
    assign flat_s_rresp                     = {s_rresp[1],      s_rresp[0]};
    assign flat_s_rlast                     = {s_rlast[1],      s_rlast[0]};
    assign flat_s_rvalid                    = {s_rvalid[1],     s_rvalid[0]};
    assign {s_rready[1],     s_rready[0]}   = flat_s_rready;
endmodule
