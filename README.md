# DVP RX Controller

[*Document of the DVP RX Controller*](https://www.notion.so/DVP-RX-Controller-1581fdef70268057babac1ec0d40004b?pvs=21)

[*HDL source of DVP RX Controller*](https://github.com/atfox272/DVP-RX-Controller)

# Introduction

DVP RX Controller is a parallel camera interface controller that supports communicating with **Digital Video Port (DVP) protocol**. The controller provides 2 AXI interfaces to stream the pixels and configure the controller. The user can configure an internal Direct-memory-access (DMA) to set streaming pixel out mode. Moreover, the controller provides some features: hardware image resizer, pixel-misalignment automatic correction, etc.

## Feature

- Support **DVP protocol**
- Support **AXI4** Slave interface to configure the controller
- Support **AXI4** Master interface to stream images
- Support **configurable DMA** to stream images
    - 2D transfer mode
    - Cyclic transfer mode
- Support **interrupt** outputs for Image capturing and Image streaming
- Support **pixel-misalignment automatic correction**
- Support a **configurable image resizer**
    - Image Downscaler (ratio 4:1)
    - Pixel Grayscale (RGB to Gray)

## More detail
Read the [*Document of the DVP RX Controller*](https://www.notion.so/DVP-RX-Controller-1581fdef70268057babac1ec0d40004b?pvs=21) for more detail

# Getting Started
## Prepare
```
git clone https://github.com/atfox272/DVP-RX-Controller.git
cd ./DVP-RX-Controller
git submodule update --init --recursive
```
## RTL file
All RTL files are listed in the [rtl.f](https://github.com/atfox272/DVP-RX-Controller/blob/v1.1/sim/rtl.f) file.

## Simulation
More detail in the section **Verification** in [*document of the DVP RX Controller*](https://www.notion.so/DVP-RX-Controller-1581fdef70268057babac1ec0d40004b?pvs=21). In Simulation section, I only guide you how to run simulation.
```
cd ./sim
make
```
