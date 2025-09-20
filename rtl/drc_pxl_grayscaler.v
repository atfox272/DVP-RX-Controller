// RGB565 -> GRAY 
module drc_pxl_grayscaler
#(
    parameter RGB_PXL_W = 16,
    parameter R_DAT_W   = 5,
    parameter G_DAT_W   = 6,
    parameter B_DAT_W   = 5,
    parameter GS_PXL_W  = 8
)
(
    // -- RGB
    input   [RGB_PXL_W-1:0] rgb_pxl_i,
    input                   rgb_pxl_last_i, // Last pixel of the frame
    input                   rgb_pxl_vld_i,
    output                  rgb_pxl_rdy_o,
    // -- Gray
    output  [GS_PXL_W-1:0]  gs_pxl_o,
    output                  gs_pxl_last_o,  // Last pixel of the frame
    output                  gs_pxl_vld_o,
    input                   gs_pxl_rdy_i
);
    // Localparam
    localparam R_MSB_IDX    = RGB_PXL_W-1;
    localparam G_MSB_IDX    = R_MSB_IDX-R_DAT_W;
    localparam B_MSB_IDX    = G_MSB_IDX-G_DAT_W;
    localparam STD_R_DAT_W  = 8;
    localparam STD_G_DAT_W  = 8;
    localparam STD_B_DAT_W  = 8;
    // Internal declaration
    wire    [STD_R_DAT_W-1:0]   r_data;
    wire    [STD_G_DAT_W-1:0]   g_data;
    wire    [STD_B_DAT_W-1:0]   b_data;
    // Combination logic
    assign gs_pxl_last_o    = rgb_pxl_last_i;
    assign gs_pxl_vld_o     = rgb_pxl_vld_i;
    assign rgb_pxl_rdy_o    = gs_pxl_rdy_i;
    assign r_data           = (STD_R_DAT_W)'(rgb_pxl_i[R_MSB_IDX-:R_DAT_W]);
    assign g_data           = (STD_G_DAT_W)'(rgb_pxl_i[G_MSB_IDX-:G_DAT_W]);
    assign b_data           = (STD_B_DAT_W)'(rgb_pxl_i[B_MSB_IDX-:B_DAT_W]);
    assign gs_pxl_o         = (r_data>>2) + (r_data>>5) + (g_data>>1) + (g_data>>4) + (b_data>>4) + (b_data>>5);
endmodule
