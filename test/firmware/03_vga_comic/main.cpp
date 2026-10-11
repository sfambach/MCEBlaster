//-*- C++ -*-
//
// Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
// License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
// Free for noncommercial use; commercial use only with the author's consent.
//
// VGA test picture for the MCE Blaster, step 3: a 64-colour comic picture in
// 800x600 @ 56 Hz. The picture has 400x300 pixels, every pixel is shown 2x2:
// the PIO holds each byte for two pixel clocks and every picture line is sent
// twice. This tests all 64 colours of the DAC instead of only black and white.
//

#include "Comic.h"
#include "VGAOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>

// Same pins as the MCE Blaster firmware (Common.h): GPIO 14-19 RRGGBB,
// GPIO 20 HSync, GPIO 21 VSync.
static constexpr uint32_t VGA_FIRST_GPIO = 14;

static constexpr uint8_t HBit = 1u << 6;
static constexpr uint8_t VBit = 1u << 7;

// 252 MHz / 7 = 36 MHz pixel clock, as in step 2.
static constexpr uint32_t SysClkKHz = 252000;
static constexpr uint32_t ClkDiv = 7;

// 800x600 @ 56 Hz (TimingsVGA.def), horizontal values halved because the PIO
// outputs every byte for two pixel clocks. HSync and VSync are positive.
static constexpr uint32_t HVisible = ComicW, HFrontPorch = 12, HSync = 36,
                          HBackPorch = 64;
static constexpr uint32_t VVisible = 2 * ComicH, VFrontPorch = 1, VSync = 2,
                          VBackPorch = 22;
static constexpr uint32_t HTotal = HVisible + HFrontPorch + HSync + HBackPorch;
static constexpr uint32_t VTotal = VVisible + VFrontPorch + VSync + VBackPorch;
static_assert(HTotal % 4 == 0, "DMA sends 32-bit words");

static uint8_t PicLines[ComicH][HTotal] __attribute__((aligned(4)));
static uint8_t BlankLine[HTotal] __attribute__((aligned(4)));
static uint8_t VSyncLine[HTotal] __attribute__((aligned(4)));
static const uint8_t *Frame[VTotal];

static PIO Pio = pio0;
static uint SM;
static int DataChan;
static int CtrlChan;

/// Fills \p Line with black pixels and the sync levels for this line.
static void initLine(uint8_t *Line, bool InVSync) {
  uint8_t V = InVSync ? VBit : 0;
  std::memset(Line, V, HTotal);
  std::memset(Line + HVisible + HFrontPorch, V | HBit, HSync);
}

/// Unpacks the RLE picture into the line buffers and builds the line table.
static void buildFrame() {
  initLine(BlankLine, false);
  initLine(VSyncLine, true);
  const uint8_t *Src = ComicRLE;
  for (uint32_t Y = 0; Y != ComicH; ++Y) {
    uint8_t *Line = PicLines[Y];
    initLine(Line, false);
    for (uint32_t X = 0; X != ComicW;) {
      uint8_t Cnt = *Src++;
      uint8_t Color = *Src++ & 0x3f;
      std::memset(Line + X, Color, Cnt);
      X += Cnt;
    }
  }
  for (uint32_t Y = 0; Y != VTotal; ++Y) {
    uint32_t SyncStart = VVisible + VFrontPorch;
    if (Y < VVisible)
      Frame[Y] = PicLines[Y / 2];
    else if (Y >= SyncStart && Y < SyncStart + VSync)
      Frame[Y] = VSyncLine;
    else
      Frame[Y] = BlankLine;
  }
}

// The control channel feeds the next line address to the data channel, so the
// hardware runs on its own. This IRQ only rewinds the line table once per
// frame and has a whole scanline of time to do so.
static void ctrlChanIRQ() {
  dma_hw->ints0 = 1u << CtrlChan;
  if (dma_hw->ch[CtrlChan].read_addr == (uintptr_t)&Frame[VTotal])
    dma_channel_set_read_addr(CtrlChan, &Frame[0], /*trigger=*/false);
}

static void startVGA() {
  SM = pio_claim_unused_sm(Pio, true);
  uint Offset = pio_add_program(Pio, &VGAOut_program);
  VGAOutPioConfig(Pio, SM, Offset, VGA_FIRST_GPIO, (float)ClkDiv);

  DataChan = dma_claim_unused_channel(true);
  CtrlChan = dma_claim_unused_channel(true);

  dma_channel_config DataConf = dma_channel_get_default_config(DataChan);
  channel_config_set_transfer_data_size(&DataConf, DMA_SIZE_32);
  channel_config_set_read_increment(&DataConf, true);
  channel_config_set_write_increment(&DataConf, false);
  channel_config_set_dreq(&DataConf, pio_get_dreq(Pio, SM, /*is_tx=*/true));
  channel_config_set_chain_to(&DataConf, CtrlChan);
  dma_channel_configure(DataChan, &DataConf, &Pio->txf[SM], nullptr,
                        HTotal / 4, /*trigger=*/false);

  dma_channel_config CtrlConf = dma_channel_get_default_config(CtrlChan);
  channel_config_set_transfer_data_size(&CtrlConf, DMA_SIZE_32);
  channel_config_set_read_increment(&CtrlConf, true);
  channel_config_set_write_increment(&CtrlConf, false);
  dma_channel_configure(CtrlChan, &CtrlConf,
                        &dma_hw->ch[DataChan].al3_read_addr_trig, &Frame[0], 1,
                        /*trigger=*/false);

  dma_channel_set_irq0_enabled(CtrlChan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, ctrlChanIRQ);
  irq_set_enabled(DMA_IRQ_0, true);

  pio_sm_set_enabled(Pio, SM, true);
  dma_channel_start(CtrlChan);
}

int main() {
  set_sys_clock_khz(SysClkKHz, true);
  stdio_init_all();
  buildFrame();
  startVGA();

#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  float PixClk = SysClkKHz * 1000.0f / ClkDiv;
  float HHz = PixClk / (2 * HTotal);
  while (true) {
    std::printf("VGA comic: 800x600 %.2f Hz, H %.3f kHz, pixel clock %.3f "
                "MHz, 64 colours\n",
                HHz / VTotal, HHz / 1000.0f, PixClk / 1000000.0f);
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
    sleep_ms(2000);
  }
}
