`timescale 1ns / 1ps

`define DUT_CLK_PERIOD  2
`define DVP_CLK_PERIOD  12
`define RST_DLY_START   3
`define RST_DUR         9

// Monitor mode
`define MONITOR_AXI4_SLV_AW
`define MONITOR_AXI4_SLV_W
`define MONITOR_AXI4_SLV_B
// `define MONITOR_AXI4_SLV_AR
// `define MONITOR_AXI4_SLV_R

`define END_TIME        60000000

// DVP Physical characteristic
// -- t_PDV = 5 ns = (5/INTERNAL_CLK_PERIOD)*DUT_CLK_PERIOD = (5/8)*2
`define DVP_PCLK_DLY    1.25

// System
parameter INTERNAL_CLK      = 125_000_000;
// Memory Mapping
parameter DRC_BASE_ADDR     = 32'h8000_0000;    // Memory mapping - BASE
parameter DMA_SEL_BIT       = 28;   // Select bit of DMA -> ADDR[DMA_SEL_BIT]
// DMA AXI interface
parameter DMA_DATA_W        = 256;
parameter DMA_ADDR_W        = 32;
// Master AXI interface
parameter S_DATA_W          = 32;
parameter S_ADDR_W          = 32;
parameter MST_ID_W          = 5;
parameter ATX_LEN_W         = 8;
parameter ATX_SIZE_W        = 3;
parameter ATX_RESP_W        = 2;
// DVP configuration
parameter DVP_DATA_W        = 8;
parameter DVP_FIFO_D        = 32;   // DVP FIFO depth 
// Image 
parameter PXL_GRAYSCALE     = 1;    // Resize (Pixel Grayscale) - 0: DISABLE || 1 : ENABLE 
parameter FRM_DOWNSCALE     = 0;    // Resize (Frame Downscale) - 0: DISABLE || 1 : ENABLE
parameter FRM_COL_NUM       = 640;  // Maximum columns in 1 frame
parameter FRM_ROW_NUM       = 480;  // Maximum rows in 1 frame
parameter DOWNSCALE_TYPE    = "AVR-POOLING";  // Downscale Type - "AVR-POOLING": Average Pooling || "MAX-POOLING": Max pooling

parameter IMG_DIM_MAX       = (FRM_COL_NUM > FRM_ROW_NUM) ? FRM_COL_NUM : FRM_ROW_NUM;
parameter IMG_DIM_W         = $clog2(IMG_DIM_MAX);

/************ AXI Transaction ************/
typedef struct {
    bit                     trans_type; // Write(1) / read(0) transaction
    bit [MST_ID_W-1:0]      axid;
    bit [DMA_ADDR_W-1:0]    axaddr;
    bit [1:0]               axburst;
    bit [ATX_LEN_W-1:0]     axlen;
    bit [ATX_SIZE_W-1:0]    axsize;
} atx_ax_info;
typedef struct {
    bit [DMA_DATA_W-1:0]    wdata [32];
    bit [ATX_LEN_W-1:0]     wlen; // length = wlen + 1
} atx_w_info;
typedef struct {
    bit [MST_ID_W-1:0]      bid;
    bit [ATX_RESP_W-1:0]    bresp;
} atx_b_info;

/****************** DMA ******************/ 
typedef struct {
    bit                     dma_en;
} dma_info;
typedef struct {
    bit [31:0]              chn_id;
    bit                     chn_en;             // Channel Enable
    bit                     chn_2d_xfer;        // 2D mode (flag)
    bit                     chn_cyclic_xfer;    // Cyclic mode (flag)
    bit                     chn_irq_msk_com;    // Interrupt completion mask
    bit                     chn_irq_msk_qed;    // Interrupt queueing mask
    bit [2:0]               chn_arb_rate;       // Channel arbitration rate
    bit [MST_ID_W-1:0]      atx_id;             // AXI Transaction ID
    bit [1:0]               atx_src_burst;      // AXI Transaction Burst type 
    bit [1:0]               atx_dst_burst;      // AXI Transaction Burst type 
    bit [15:0]              atx_wd_per_burst;   // Word per burst 
} channel_info;
typedef struct {
    bit [31:0]              chn_id;
    bit [DMA_ADDR_W-1:0]    src_addr;
    bit [DMA_ADDR_W-1:0]    dst_addr;
    bit [15:0]              xfer_xlen;
    bit [15:0]              xfer_ylen;
    bit [15:0]              src_stride;
    bit [15:0]              dst_stride;
} descriptor_info;

/************ DVP RX Controller *********/ 
typedef struct {
    bit                     cam_rx_en;
    bit                     cam_pwdn;
    bit [1:0]               cam_rx_mode;// 2’b00: SLEEP || 2’b01: SINGLE-SHOT || 2’b10: STREAM
    bit                     cam_rx_start;
    bit                     irq_fr_comp_msk;
    bit                     irq_fr_err_msk;
    bit [IMG_DIM_W-1:0]     img_width;
    bit [IMG_DIM_W-1:0]     img_height;
} drc_info;

// Sequence queue
mailbox #(atx_ax_info)  s_seq_aw_info;
mailbox #(atx_w_info)   s_seq_w_info;

// Driver queue
mailbox #(atx_ax_info)  m_drv_aw_info;
mailbox #(atx_b_info)   m_drv_b_info;
mailbox #(atx_ax_info)  m_drv_ar_info;

module dvp_rx_controller_tb;

    localparam IN_PXL_SIZE      = 16;   // Input pixel size (RGB565)
    localparam PROC_PXL_SIZE    = PXL_GRAYSCALE ? 8 : IN_PXL_SIZE;   // Processed pixel size
    
    localparam IN_FRM_SIZE      = FRM_COL_NUM * FRM_ROW_NUM;// Input frame size
    localparam PROC_FRM_COL_NUM = FRM_DOWNSCALE ? (FRM_COL_NUM/2) : FRM_COL_NUM;
    localparam PROC_FRM_ROW_NUM = FRM_DOWNSCALE ? (FRM_ROW_NUM/2) : FRM_ROW_NUM;
    localparam PROC_FRM_SIZE    = FRM_DOWNSCALE ? (IN_FRM_SIZE/4) : IN_FRM_SIZE; // Processed frame size 

    localparam MEM_WORD_W       = 256;// 256bit-access
    localparam MEM_FRM_SIZE     = PROC_FRM_SIZE * PROC_PXL_SIZE / MEM_WORD_W;
    localparam MEM_FRM_ADDR_W   = $clog2(MEM_FRM_SIZE);

    localparam DMA_BASE_ADDR    = DRC_BASE_ADDR |  (1 << DMA_SEL_BIT);  // DMA Memory Address:    ADDR[DMA_SEL_BIT] = 1
    localparam DRM_BASE_ADDR    = DRC_BASE_ADDR & ~(1 << DMA_SEL_BIT);  // RegMap Memory Address: ADDR[DMA_SEL_BIT] = 0
    
    logic                       clk;
    logic                       rst_n;
    // DVP RX interface
    logic   [DVP_DATA_W-1:0]    dvp_d_i;
    logic                       dvp_href_i;
    logic                       dvp_vsync_i;
    logic                       dvp_hsync_i;
    logic                       dvp_pclk_i;
    logic                      dvp_xclk_o;
    logic                      dvp_pwdn_o;
    // AXI4 interface (configuration)
    // -- AW channel
    logic   [MST_ID_W-1:0]      s_awid_i;
    logic   [S_ADDR_W-1:0]      s_awaddr_i;
    logic   [1:0]               s_awburst_i;
    logic   [ATX_LEN_W-1:0]     s_awlen_i;
    logic                       s_awvalid_i;
    logic                      s_awready_o;
    // -- W channel
    logic   [S_DATA_W-1:0]      s_wdata_i;
    logic                       s_wlast_i;
    logic                       s_wvalid_i;
    logic                      s_wready_o;
    // -- B channel
    logic  [MST_ID_W-1:0]      s_bid_o;
    logic  [ATX_RESP_W-1:0]    s_bresp_o;
    logic                      s_bvalid_o;
    logic                       s_bready_i;
    // -- AR channel
    logic   [MST_ID_W-1:0]      s_arid_i;
    logic   [S_ADDR_W-1:0]      s_araddr_i;
    logic   [1:0]               s_arburst_i;
    logic   [ATX_LEN_W-1:0]     s_arlen_i;
    logic                       s_arvalid_i;
    logic                      s_arready_o;
    // -- R channel
    logic  [MST_ID_W-1:0]      s_rid_o;
    logic  [S_DATA_W-1:0]      s_rdata_o;
    logic  [ATX_RESP_W-1:0]    s_rresp_o;
    logic                      s_rlast_o;
    logic                      s_rvalid_o;
    logic                       s_rready_i;
    // AXI4 interface (pixels streaming)
    // -- AW channel         
    logic  [MST_ID_W-1:0]      m_awid_o;
    logic  [DMA_ADDR_W-1:0]    m_awaddr_o;
    logic  [ATX_LEN_W-1:0]     m_awlen_o;
    logic  [1:0]               m_awburst_o;
    logic                      m_awvalid_o;
    logic                       m_awready_i;
    // -- W channel          
    logic  [DMA_DATA_W-1:0]    m_wdata_o;
    logic                      m_wlast_o;
    logic                      m_wvalid_o;
    logic                       m_wready_i;
    // -- B channel
    logic   [MST_ID_W-1:0]      m_bid_i;
    logic   [ATX_RESP_W-1:0]    m_bresp_i;
    logic                       m_bvalid_i;
    logic                      m_bready_o;
    // Interrupt and Trap
    logic                      drc_irq; // Caused by frame completion
    logic                      drc_trap;// Caused by pixel misalignment
    logic                      dma_irq; // Caused by frame transaction completion
    logic                      dma_trap;// Caused by wrong address mapping
    
    
    reg [IN_PXL_SIZE-1:0]   input_img   [0:IN_FRM_SIZE-1];
    reg [MEM_WORD_W-1:0]    output_img  [0:MEM_FRM_SIZE-1];

    int pclk_cnt    = 0;
    int dvp_st      = 0;
    int tx_cnt      = 0;
    int wdata_cnt   = 0;
    dvp_rx_controller #(
        .INTERNAL_CLK      (INTERNAL_CLK),
        .DRC_BASE_ADDR     (DRC_BASE_ADDR),
        .DMA_SEL_BIT       (DMA_SEL_BIT),
        .DMA_DATA_W        (DMA_DATA_W),
        .DMA_ADDR_W        (DMA_ADDR_W),
        .S_DATA_W          (S_DATA_W),
        .S_ADDR_W          (S_ADDR_W),
        .MST_ID_W          (MST_ID_W),
        .ATX_LEN_W         (ATX_LEN_W),
        .ATX_SIZE_W        (ATX_SIZE_W),
        .ATX_RESP_W        (ATX_RESP_W),
        .DVP_DATA_W        (DVP_DATA_W),
        .DVP_FIFO_D        (DVP_FIFO_D),
        .PXL_GRAYSCALE     (PXL_GRAYSCALE),
        .FRM_DOWNSCALE     (FRM_DOWNSCALE),
        .FRM_COL_NUM       (FRM_COL_NUM),
        .FRM_ROW_NUM       (FRM_ROW_NUM),
        .DOWNSCALE_TYPE    (DOWNSCALE_TYPE)
    ) dut (
        .*
    );
    
    initial begin
        clk     <= 0;
        rst_n   <= 1;
        
        // DVP interface
        dvp_d_i     <= 0;    
        dvp_href_i  <= 0;
        dvp_vsync_i <= 0;
        dvp_hsync_i <= 0;
        dvp_pclk_i  <= 0;
        // AXI4 Pixel TX interface
        m_awready_i <= 1;
        m_wready_i  <= 1;
        m_bvalid_i  <= 0;
        // AXI4 Configuration interface
        s_awvalid_i <= 0;
        s_wvalid_i  <= 0;
        s_bready_i  <= 1;
        s_arvalid_i <= 0; 
        s_rready_i  <= 1;
        
        #(`RST_DLY_START)   rst_n <= 0;
        #(`RST_DUR)         rst_n <= 1;
    end
    
    initial begin
        forever #(`DUT_CLK_PERIOD/2) clk <= ~clk;
    end

    initial begin
        $readmemh("./env/dvp_mem_data.txt", input_img);
    end
    
    // initial begin
    //     #(`RST_DLY_START + `RST_DUR + 1);
    //     wait(dvp_pwdn_o == 1'b0); #0.1;
    //     forever #(`DVP_CLK_PERIOD/2) dvp_pclk_i <= ~dvp_pclk_i;
    // end

    always @(dvp_xclk_o) begin
        #1; dvp_pclk_i <= dvp_xclk_o;
    end
    
    initial begin : SOFTWARE_SEQUENCE
        // SOFTWARE_SEQUENCE: uses to configure DMA & DRC via High-Level-Abtraction task
        dma_info        dma_config;
        channel_info    chn_config;
        descriptor_info desc_config;
        drc_info        drc_config;
        s_seq_aw_info   = new();
        s_seq_w_info    = new();

        /************************************************************************/
        /****************** ADD YOUR CUSTOM IP CONTROL HERE *********************/ 
        /************************************************************************/
        // Configure DMA
        dma_config.dma_en = 1'b1;
        config_dma(dma_config);

        // Configure DMA-Channel
        chn_config.chn_id           = 'd00;
        chn_config.chn_en           = 1'b1; // Enable channel 0
        chn_config.chn_2d_xfer      = 1'b1; // On
        chn_config.chn_cyclic_xfer  = 1'b0; // Off
        chn_config.chn_irq_msk_com  = 1'b1; // Enable
        chn_config.chn_irq_msk_qed  = 1'b0; // Disable
        chn_config.chn_arb_rate     = 'h03;
        chn_config.atx_id           = 'h02;
        chn_config.atx_src_burst    = 2'b00; // FIX burst 
        chn_config.atx_dst_burst    = 2'b01; // INCR burst
        chn_config.atx_wd_per_burst = 'd15;  // 16 AXI transfers per burst
        config_chn(chn_config);
        
        // Push 1 Descriptor to DMA-Channel
        desc_config.chn_id          = 'd00;
        desc_config.src_addr        = 32'h1000_0000;
        desc_config.dst_addr        = 32'h2000_0000;
        desc_config.xfer_xlen       = PROC_FRM_COL_NUM * PROC_PXL_SIZE / MEM_WORD_W - 1; 
        desc_config.xfer_ylen       = PROC_FRM_ROW_NUM - 1; // Row Length = FRM_ROW_NUM
        desc_config.src_stride      = PROC_FRM_COL_NUM * PROC_PXL_SIZE / MEM_WORD_W;
        desc_config.dst_stride      = PROC_FRM_COL_NUM * PROC_PXL_SIZE / MEM_WORD_W;
        config_desc(desc_config);

        // Configure DVP RX Controller
        drc_config.cam_rx_en        = 1'b1;
        drc_config.cam_pwdn         = 1'b0;
        drc_config.cam_rx_mode      = 2'b10;// 2’b00: SLEEP || 2’b01: SINGLE-SHOT || 2’b10: STREAM
        drc_config.cam_rx_start     = 1'b1;
        drc_config.irq_fr_comp_msk  = 1'b1;
        drc_config.irq_fr_err_msk   = 1'b1;
        drc_config.img_width        = FRM_COL_NUM;  // Image width from DVP interface
        drc_config.img_height       = FRM_ROW_NUM;  // Image height from DVP interface
        config_drc(drc_config);

        #18121676;
        config_desc(desc_config);
        config_desc(desc_config);
    end
    
    /* ------------ Driver ------------ */
    initial begin
        #(`RST_DLY_START + `RST_DUR + 1);
        fork
            dvp_driver();
            slave_driver();
            master_driver();
        join_none
    end 
    /* ------------ Driver ------------ */


    /*------------ Monitor ------------*/
    initial begin   : DMA_SLV_MONITOR
        #(`RST_DLY_START + `RST_DUR + 1);
        fork
            drc_slave_monitor();
            mem_monitor();
        join_none
    end
    /*------------ Monitor ------------*/

    initial begin
        #(`END_TIME) $finish;
    end



    // ===============================================================================
    // =============================== Task Definitions ==============================
    // ===============================================================================

    task automatic dvp_driver();
        // Important note: 
        //      - Data and Control signal in DVP are changed in FALLING edge of PCLK
        //          + T_clk_delay   = 5ns
        //          + T_setup       = 15ns
        //          + T_hold        = 8ns
        //      -> Data will be stable befor RISING edge 
        //      -> (Delay time to sample data at RISING edge) < 8ns
        localparam DVP_IDLE_ST          = 0;
        localparam DVP_SOF_ST           = 1;
        localparam DVP_PRE_TXN_ST       = 2;
        localparam DVP_PRE_HSYNC_FALL_ST= 3;
        localparam DVP_HSYNC_FALL_ST    = 4;
        localparam DVP_PRE_TX_ST        = 5;
        localparam DVP_TX_ST            = 6;
        localparam DVP_POST_TXN         = 7;
        localparam DVP_EOF_ST           = 8;
        int stall_cnt                   = 0;
        while(1'b1) begin
            stall_cnt = $urandom_range(10, 20);
            repeat(stall_cnt) begin
                pclk_cl;  
            end
            while (1'b1) begin
                case(dvp_st)
                    DVP_IDLE_ST: begin
                        dvp_st      = DVP_SOF_ST;
                        pclk_cnt    = 3*784*2;
                        dvp_vsync_i <= 1'b1;
                    end
                    DVP_SOF_ST: begin
                    if(pclk_cnt == 0) begin
                        dvp_st      = DVP_PRE_TXN_ST;
                        pclk_cnt    = 17*784*2 - 80*2 - 40*2 - 19*2;
                        dvp_vsync_i <= 1'b0;
                    end
                    end
                    DVP_PRE_TXN_ST: begin
                    if(pclk_cnt == 0) begin
                        dvp_st      = DVP_PRE_HSYNC_FALL_ST;
                        pclk_cnt    = 19*2;
                        dvp_hsync_i <= 1'b1;
                    end
                    end
                    DVP_PRE_HSYNC_FALL_ST: begin
                    if(pclk_cnt == 0) begin
                        dvp_st      = DVP_HSYNC_FALL_ST;
                        pclk_cnt    = 80*2;
                        dvp_hsync_i <= 1'b0;
                    end
                    end
                    DVP_HSYNC_FALL_ST: begin
                        if(pclk_cnt == 0) begin
                            dvp_st      = DVP_PRE_TX_ST;
                            pclk_cnt    = 40*2;
                            dvp_hsync_i <= 1'b1;
                        end
                    end
                    DVP_PRE_TX_ST: begin
                        if(pclk_cnt == 0) begin
                            if(tx_cnt == (FRM_COL_NUM*FRM_ROW_NUM*2)) begin
                                dvp_st      = DVP_POST_TXN;
                                pclk_cnt    = 10*784*2 - 80*2 - 40*2 - 19*2;
                                tx_cnt      = 0;
                            end
                            else begin
                                dvp_st      = DVP_TX_ST;
                                dvp_href_i  <= 1'b1;
                                // dvp_d_i     <= tx_cnt%32;
                                dvp_d_i     <= (tx_cnt%2 == 0) ? input_img[tx_cnt/2][15:8] : input_img[tx_cnt/2][7:0];
                                
                            end
                        end
                    end
                    DVP_TX_ST: begin
                        tx_cnt = tx_cnt + 1;
                        dvp_d_i <= (tx_cnt%2 == 0) ? input_img[tx_cnt/2][15:8] : input_img[tx_cnt/2][7:0];
                        if(tx_cnt%(FRM_COL_NUM*2) == 0) begin
                            dvp_st      = DVP_PRE_HSYNC_FALL_ST;
                            pclk_cnt    = 19*2;
                            dvp_hsync_i <= 1'b1;
                            dvp_href_i  <= 1'b0;
                        end
                    end
                    DVP_POST_TXN: begin
                        if(pclk_cnt == 0) begin
                            dvp_st      = DVP_EOF_ST;
                            pclk_cnt    = 3*784*2;
                            dvp_vsync_i <= 1'b1;
                        end
                    end
                    DVP_EOF_ST: begin
                        if(pclk_cnt == 0) begin
                            dvp_st      = DVP_IDLE_ST;
                            pclk_cnt    = 0;
                            dvp_vsync_i <= 1'b0;
                            break;
                        end
                    end
                endcase
                pclk_cl;
                pclk_cnt = pclk_cnt - 1;
            end
        end
    endtask
    task automatic slave_driver;
        m_drv_aw_info   = new();
        m_drv_b_info    = new();
        m_drv_ar_info   = new();
        fork
            begin   : AW_CHN
                atx_ax_info aw_temp;
                forever begin
                    m_awready_i = 1'b1;
                    m_aw_receive (
                        .awid   (aw_temp.axid),
                        .awaddr (aw_temp.axaddr),
                        .awburst(aw_temp.axburst),
                        .awlen  (aw_temp.axlen)
                        // .awsize (aw_temp.axsize)
                    );
                    // Store AW info 
                    m_drv_aw_info.put(aw_temp);
                    // Handshake occurs
                    aclk_cl;
                end
            end
            begin   : W_CHN
                atx_ax_info aw_temp;
                atx_w_info  w_temp;
                atx_b_info  b_temp;
                bit         wlast_temp;
                forever begin
                    if(m_drv_aw_info.try_get(aw_temp)) begin
                        for(int i = 0; i <= aw_temp.axlen; i = i + 1) begin
                            // Assert WREADY
                            m_wready_i = 1'b1;
                            m_w_receive (
                                .wdata(w_temp.wdata[i]),
                                .wlast(wlast_temp)
                            );
                            // WLAST predictor
                            if(wlast_temp == (i == aw_temp.axlen)) begin
                            
                            end
                            else begin
                                $display("[FAIL]: Destination - Wrong sample WLAST = %0d at WDATA = %8h (idx: %0d, AWLEN: %2d)", wlast_temp, w_temp.wdata[i], i, aw_temp.axlen);
                                $stop;
                            end
                            // Store pixel group to Memory
                            output_img[(aw_temp.axaddr[MEM_FRM_ADDR_W-1:0] + i)] <= w_temp.wdata[i];

                            // Handshake occurs 
                            aclk_cl;
                        end
                        // Generate B transfer
                        b_temp.bid      = aw_temp.axid;
                        b_temp.bresp    = 2'b00;
                        m_drv_b_info.put(b_temp);
                    end
                    else begin
                        // Wait 1 cycle
                        aclk_cl;
                        m_wready_i = 1'b0;
                    end
                end
            end
            begin   : B_CHN
                atx_b_info  b_temp;
                forever begin
                    if(m_drv_b_info.try_get(b_temp)) begin
                        m_b_transfer (
                            .bid(b_temp.bid),
                            .bresp(b_temp.bresp)
                        );
                        // $display("[INFO]: Destination - The transaction with ID-%0h has been completed", b_temp.bid);
                    end
                    else begin
                        // Wait 1 cycle
                        aclk_cl;
                        m_bvalid_i = 1'b0;
                    end
                end
            end
        join_none
    endtask
    task automatic master_driver();
        fork 
            begin   : AW_DRV
                atx_ax_info aw_temp;
                forever begin
                    if(s_seq_aw_info.try_get(aw_temp)) begin
                        s_aw_transfer(.s_awid(aw_temp.axid), .s_awaddr(aw_temp.axaddr), .s_awburst(2'b01), .s_awlen(aw_temp.axlen));
                    end
                    else begin
                        aclk_cl;    // Penalty 1 cycle
                        s_awvalid_i <= 1'b0;
                    end
                end
            end
            begin   : W_DRV
                atx_w_info w_temp;
                int w_cnt;
                forever begin
                    if(s_seq_w_info.try_get(w_temp)) begin
                        for(w_cnt = 0; w_cnt <= w_temp.wlen; w_cnt++) begin
                            s_w_transfer(.s_wdata(w_temp.wdata[w_cnt]), .s_wlast(w_temp.wlen == w_cnt));
                        end
                    end
                    else begin
                        aclk_cl;    // Penalty 1 cycle
                        s_wvalid_i <= 1'b0;
                    end
                end
            end
            begin   : AR_DRV
                // 1st: TRANSFER_ID
                s_ar_transfer(.s_arid(5'h00), .s_araddr(32'h8000_2001), .s_arburst(2'b01), .s_arlen(8'd00));
                // 2nd: TRANSFER_ID
                s_ar_transfer(.s_arid(5'h00), .s_araddr(32'h8000_2001), .s_arburst(2'b01), .s_arlen(8'd00));
                aclk_cl;
                s_arvalid_i <= 1'b0;
            end
        join_none
    endtask
    
    task automatic drc_slave_monitor();
        fork 
            `ifdef MONITOR_AXI4_SLV_AW
                begin   : AW_chn
                    while(1'b1) begin
                        wait(s_awready_o & s_awvalid_i); #0.1;  // AW hanshaking
                        $display("\n---------- DMA Slave: AW channel ----------");
                        $display("AWID:     0x%8h", s_awid_i);
                        $display("AWADDR:   0x%8h", s_awaddr_i);
                        $display("AWLEN:    0x%8h", s_awlen_i);
                        $display("--------------------------------------------");
                        aclk_cl;
                    end
                end
            `endif
            `ifdef MONITOR_AXI4_SLV_W
                begin   : W_chn
                    while(1'b1) begin
                        wait(s_wready_o & s_wvalid_i); #0.1;  // W hanshaking
                        $display("\n\t\t\t\t\t\t---------- DMA Slave: W channel ----------");
                        $display("\t\t\t\t\t\tWDATA:    0x%8h", s_wdata_i);
                        $display("\t\t\t\t\t\tWLAST:    0x%8h", s_wlast_i);
                        $display("\t\t\t\t\t\t--------------------------------------------");
                        aclk_cl;
                    end
                end
            `endif
            `ifdef MONITOR_AXI4_SLV_B
                begin   : B_chn
                    while(1'b1) begin
                        wait(s_bready_i & s_bvalid_o); #0.1;  // B hanshaking
                        $display("\n\t\t\t\t\t\t\t\t\t\t\t\t---------- DMA Slave: B channel ----------");
                        $display("\t\t\t\t\t\t\t\t\t\t\t\tBID:      0x%8h", s_bid_o);
                        $display("\t\t\t\t\t\t\t\t\t\t\t\tBRESP:    0x%8h", s_bresp_o);
                        $display("\t\t\t\t\t\t\t\t\t\t\t\t--------------------------------------------");
                        aclk_cl;
                    end
                end
            `endif
            `ifdef MONITOR_AXI4_SLV_AR
                begin   : AR_chn
                    while(1'b1) begin
                        wait(s_arready_o & s_arvalid_i); #0.1;  // AR hanshaking
                        $display("\n---------- DMA Slave: AR channel ----------");
                        $display("ARID:     0x%8h", s_arid_i);
                        $display("ARADDR:   0x%8h", s_araddr_i);
                        $display("ARLEN:    0x%8h", s_arlen_i);
                        $display("--------------------------------------------");
                        aclk_cl;
                    end
                end
            `endif
            `ifdef MONITOR_AXI4_SLV_R
                begin   : R_chn
                    while(1'b1) begin
                        wait(s_rready_i & s_rvalid_o); #0.1;  // R hanshaking
                        $display("\n\t\t\t\t\t\t\t\t\t\t\t\t---------- DMA Slave: R channel ----------");
                        $display("\t\t\t\t\t\t\t\t\t\t\t\tRID:      0x%8h", s_rid_o);
                        $display("\t\t\t\t\t\t\t\t\t\t\t\tRDATA:    0x%8h", s_rdata_o);
                        $display("\t\t\t\t\t\t\t\t\t\t\t\tRRESP:    0x%8h", s_rresp_o);
                        $display("\t\t\t\t\t\t\t\t\t\t\t\tRLAST:    0x%8h", s_rlast_o);
                        $display("\t\t\t\t\t\t\t\t\t\t\t\t--------------------------------------------");
                        aclk_cl;
                    end
                end
            `endif
            begin end
        join_none
    endtask

    task automatic mem_monitor();
        int fd0;
        #(`RST_DLY_START + `RST_DUR + 1); // Wait for reset ending
        while(1'b1) begin
            wait(dma_irq == 1'b1); #0.1;    // 1 frame is stored in Memory completely
            aclk_cl;
            fd0 = $fopen("./env/axi_mem_format.txt", "w");
            if (PXL_GRAYSCALE) begin
                $fwrite(fd0, "Image Size:\t\t%0d x %0d\nPixel Format:\tGRAYSCALE", PROC_FRM_COL_NUM, PROC_FRM_ROW_NUM);
            end 
            else begin
                $fwrite(fd0, "Image Size:\t\t%0d x %0d\nPixel Format:\tRGB565", PROC_FRM_COL_NUM, PROC_FRM_ROW_NUM);
            end
            $fclose(fd0);
            $writememh("./env/axi_mem_data.txt", output_img);
        end
    endtask

    task automatic s_aw_transfer(
        input [MST_ID_W-1:0]    s_awid,
        input [S_ADDR_W-1:0]    s_awaddr,
        input [1:0]             s_awburst,
        input [ATX_LEN_W-1:0]   s_awlen
    );
        aclk_cl;
        s_awid_i            <= s_awid;
        s_awaddr_i          <= s_awaddr;
        s_awburst_i         <= s_awburst;
        s_awlen_i           <= s_awlen;
        s_awvalid_i         <= 1'b1;
        // Handshake occur
        wait(s_awready_o == 1'b1); #0.1;
    endtask
    
    task automatic s_w_transfer (
        input [S_DATA_W-1:0]    s_wdata,
        input                   s_wlast
    );
        aclk_cl;
        s_wdata_i           <= s_wdata;
        s_wlast_i           <= s_wlast;
        s_wvalid_i          <= 1'b1;
        // Handshake occur
        wait(s_wready_o == 1'b1); #0.1;
    endtask
    
    task automatic s_ar_transfer(
        input [MST_ID_W-1:0]    s_arid,
        input [S_ADDR_W-1:0]    s_araddr,
        input [1:0]             s_arburst,
        input [ATX_LEN_W-1:0]   s_arlen
    );
        aclk_cl;
        s_arid_i            <= s_arid;
        s_araddr_i          <= s_araddr;
        s_arburst_i         <= s_arburst;
        s_arlen_i           <= s_arlen;
        s_arvalid_i         <= 1'b1;
        // Handshake occur
        wait(s_arready_o == 1'b1); #0.1;
    endtask
    
    task automatic m_aw_receive(
        output  [MST_ID_W-1:0]      awid,
        output  [DMA_ADDR_W-1:0]    awaddr,
        output  [1:0]               awburst,
        output  [ATX_LEN_W-1:0]     awlen
        // output      [ATX_SIZE_W-1:0]    awsize
    );
        // Wait for BVALID
        wait(m_awvalid_o == 1'b1); #0.1;
        awid    = m_awid_o;
        awaddr  = m_awaddr_o;
        awburst = m_awburst_o;
        awlen   = m_awlen_o; 
        // awsize  = m_awsize_o; 
    endtask
    task automatic m_w_receive (
        output  [DMA_DATA_W-1:0]    wdata,
        output                      wlast
    );
        wait(m_wvalid_o == 1'b1); #0.1;
        wdata   = m_wdata_o;
        wlast   = m_wlast_o;
    endtask
    task automatic m_b_transfer (
        input [MST_ID_W-1:0]    bid,
        input [ATX_RESP_W-1:0]  bresp
    );
        aclk_cl;
        m_bid_i     <= bid;
        m_bresp_i   <= bresp;
        m_bvalid_i  <= 1'b1;
        // Wait for handshaking
        wait(m_bready_o == 1'b1); #0.1;
    endtask

    task automatic aclk_cl;
        @(posedge clk);
        #0.05; 
    endtask
    task automatic aclk_cls(
        input [31:0] n
    );
        repeat(n) aclk_cl;
    endtask
    
    task automatic pclk_cl;
        @(negedge dvp_xclk_o);
        #(`DVP_PCLK_DLY); 
    endtask
    

    task automatic config_dma (dma_info info);
        atx_ax_info aw_temp;
        atx_w_info  w_temp;

        // DMA Enable register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h00;
        aw_temp.axid    = 5'h00;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = {30'h00, info.dma_en};
        w_temp.wlen     = 'h00;

        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
    endtask

    task automatic config_chn (channel_info info);
        atx_ax_info aw_temp;
        atx_w_info  w_temp;

        // Channel Enable register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h01 + (info.chn_id<<4);
        aw_temp.axid    = 5'h01;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = {30'h00, info.chn_en};
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
        
        // Channel Flag register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h02 + (info.chn_id<<4);
        aw_temp.axid    = 5'h02;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = {29'h00, info.chn_cyclic_xfer, info.chn_2d_xfer};
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
        
        // Channel Interrupt Mask register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h03 + (info.chn_id<<4);
        aw_temp.axid    = 5'h03;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = {29'h00, info.chn_irq_msk_qed, info.chn_irq_msk_com};
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
        
        // Channel AXI ID register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h05 + (info.chn_id<<4);
        aw_temp.axid    = 5'h04;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.atx_id;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
        
        // Channel AXI Source Burst register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h06 + (info.chn_id<<4);
        aw_temp.axid    = 5'h05;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.atx_src_burst;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Channel AXI Destination Burst register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h07 + (info.chn_id<<4);
        aw_temp.axid    = 5'h06;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.atx_dst_burst;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Channel AXI Words Per Burst register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h08 + (info.chn_id<<4);
        aw_temp.axid    = 5'h07;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.atx_wd_per_burst;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
    endtask

    task automatic config_desc (descriptor_info info);
        atx_ax_info aw_temp;
        atx_w_info  w_temp;

        // Descriptor - Source Address register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h09 + (info.chn_id<<4);
        aw_temp.axid    = 5'h08;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.src_addr;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Descriptor - Destination Address register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h0A + (info.chn_id<<4);
        aw_temp.axid    = 5'h09;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.dst_addr;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Descriptor - Transfer X Length register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h0B + (info.chn_id<<4);
        aw_temp.axid    = 5'h0A;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.xfer_xlen;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Descriptor - Transfer Y Length register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h0C + (info.chn_id<<4);
        aw_temp.axid    = 5'h0B;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.xfer_ylen;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Descriptor - Source Stride register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h0D + (info.chn_id<<4);
        aw_temp.axid    = 5'h0C;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.src_stride;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Descriptor - Destination Stride register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h0E + (info.chn_id<<4);
        aw_temp.axid    = 5'h0D;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.dst_stride;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // Descriptor - Submit register
        aw_temp.axaddr  = DMA_BASE_ADDR + 'h1000 + (info.chn_id<<4);
        aw_temp.axid    = 5'h0E;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = 32'h01;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);
    endtask

    task automatic config_drc(drc_info info);
        atx_ax_info aw_temp;
        atx_w_info  w_temp;
        // CAM_RX_EN
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h00; // BASE + OFFSET(CAM_RX_EN)
        aw_temp.axid    = 5'h00;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.cam_rx_en;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // CAM_PWDN
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h01; // BASE + OFFSET(CAM_PWDN)
        aw_temp.axid    = 5'h01;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.cam_pwdn;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // CAM_RX_MODE
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h02; // BASE + OFFSET(CAM_RX_MODE)
        aw_temp.axid    = 5'h02;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.cam_rx_mode;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // IRQ_MASK
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h03; // BASE + OFFSET(IRQ_MASK)
        aw_temp.axid    = 5'h04;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = {info.irq_fr_err_msk, info.irq_fr_comp_msk};
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // IMG_WIDTH
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h04; // BASE + OFFSET(IMG_WIDTH)
        aw_temp.axid    = 5'h06;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.img_width;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // IMG_HEIGHT
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h05; // BASE + OFFSET(IMG_HEIGHT)
        aw_temp.axid    = 5'h07;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.img_height;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

        // CAM_RX_START
        aw_temp.axaddr  = DRM_BASE_ADDR + 'h10; // BASE + OFFSET(CAM_RX_START)
        aw_temp.axid    = 5'h03;
        aw_temp.axburst = 2'b00;
        aw_temp.axlen   = 'h00;
        w_temp.wdata[0] = info.cam_rx_start;
        w_temp.wlen     = 'h00;
        s_seq_aw_info.put(aw_temp);
        s_seq_w_info.put(w_temp);

    endtask
endmodule
