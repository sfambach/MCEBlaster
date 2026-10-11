//-*- C++ -*-
//
// Static VGA test picture for the MCE Blaster: a white frame along the edge of
// the visible area and the VGA resolution, line and frame frequency as text.
// No TTL input is used, this only tests the VGA side (Pico, DAC, cable,
// monitor).
//

#include "Bitmaps.h"
#include "VGAOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>

#ifndef VGA_MODE
#define VGA_MODE 1
#endif

// Same pins as the MCE Blaster firmware (Common.h): GPIO 14-19 RRGGBB,
// GPIO 20 HSync, GPIO 21 VSync.
static constexpr uint32_t VGA_FIRST_GPIO = 14;

static constexpr uint8_t Black = 0b000000;
static constexpr uint8_t White = 0b111111;
static constexpr uint8_t HBit = 1u << 6;
static constexpr uint8_t VBit = 1u << 7;

// 252 MHz divides into both pixel clocks: 25.2 MHz (/10) and 36 MHz (/7).
static constexpr uint32_t SysClkKHz = 252000;

enum class Polarity { Neg, Pos };

struct VGATiming {
  const char *Name;
  uint32_t HVisible, HFrontPorch, HSync, HBackPorch;
  uint32_t VVisible, VFrontPorch, VSync, VBackPorch;
  Polarity HPol, VPol;
  uint32_t ClkDiv;
  uint32_t hTotal() const { return HVisible + HFrontPorch + HSync + HBackPorch; }
  uint32_t vTotal() const { return VVisible + VFrontPorch + VSync + VBackPorch; }
  float pixelClkHz() const { return SysClkKHz * 1000.0f / ClkDiv; }
  float hHz() const { return pixelClkHz() / hTotal(); }
  float vHz() const { return hHz() / vTotal(); }
};

// The three modes the MCE Blaster outputs (TimingsVGA.def), without the
// MCE Blaster's extra border pixels.
static constexpr VGATiming Timings[] = {
    {"640X400", 640, 16, 96, 48, 400, 12, 2, 35, Polarity::Neg, Polarity::Pos,
     10},
    {"640X480", 640, 16, 96, 48, 480, 10, 2, 33, Polarity::Neg, Polarity::Neg,
     10},
    {"800X600", 800, 24, 72, 128, 600, 1, 2, 22, Polarity::Pos, Polarity::Pos,
     7},
};
static constexpr const VGATiming &T = Timings[VGA_MODE];

static constexpr uint32_t MaxHTotal = 1024;
static constexpr uint32_t MaxVTotal = 625;
static constexpr uint32_t Border = 2;
static constexpr uint32_t Zoom = 2;
static constexpr uint32_t NumTxtLines = 7;
static constexpr uint32_t TxtLineGap = 8;

// One buffer per distinct scanline; the frame is a table of line pointers.
static uint8_t BlankLine[MaxHTotal] __attribute__((aligned(4)));
static uint8_t VSyncLine[MaxHTotal] __attribute__((aligned(4)));
static uint8_t FullBorderLine[MaxHTotal] __attribute__((aligned(4)));
static uint8_t SideBorderLine[MaxHTotal] __attribute__((aligned(4)));
static uint8_t TxtLines[NumTxtLines][BMapHeight][MaxHTotal]
    __attribute__((aligned(4)));
static const uint8_t *Frame[MaxVTotal];

static int DataChan;
static int CtrlChan;

static const uint8_t *getChar(char C) {
  // clang-format off
  switch (C) {
  case '0': return Char_0; case '1': return Char_1; case '2': return Char_2;
  case '3': return Char_3; case '4': return Char_4; case '5': return Char_5;
  case '6': return Char_6; case '7': return Char_7; case '8': return Char_8;
  case '9': return Char_9;
  case 'A': return Char_A; case 'B': return Char_B; case 'C': return Char_C;
  case 'D': return Char_D; case 'E': return Char_E; case 'F': return Char_F;
  case 'G': return Char_G; case 'H': return Char_H; case 'I': return Char_I;
  case 'J': return Char_J; case 'K': return Char_K; case 'L': return Char_L;
  case 'M': return Char_M; case 'N': return Char_N; case 'O': return Char_O;
  case 'P': return Char_P; case 'Q': return Char_Q; case 'R': return Char_R;
  case 'S': return Char_S; case 'T': return Char_T; case 'U': return Char_U;
  case 'V': return Char_V; case 'W': return Char_W; case 'X': return Char_X;
  case 'Y': return Char_Y; case 'Z': return Char_Z;
  case '.': return Char_PERIOD; case ':': return Char_COLON;
  case '@': return Char_AT; case '-': return Char_MINUS;
  default: return Char_SPACE;
  }
  // clang-format on
}

/// Fills \p Line with black pixels and the sync levels for this line.
static void initLine(uint8_t *Line, bool InVSync) {
  uint8_t HIdle = T.HPol == Polarity::Pos ? 0 : HBit;
  uint8_t HActive = HIdle ^ HBit;
  bool VHigh = (T.VPol == Polarity::Pos) == InVSync;
  uint8_t V = VHigh ? VBit : 0;
  std::memset(Line, Black | HIdle | V, T.hTotal());
  uint32_t SyncStart = T.HVisible + T.HFrontPorch;
  std::memset(Line + SyncStart, Black | HActive | V, T.HSync);
}

static void setPixel(uint8_t *Line, uint32_t X, uint8_t Color) {
  Line[X] = (Line[X] & (HBit | VBit)) | Color;
}

/// Draws font row \p Row of \p Txt, centered, into \p Line.
static void drawTxtRow(uint8_t *Line, const char *Txt, uint32_t Row) {
  uint32_t Width = std::strlen(Txt) * BMapWidth * Zoom;
  uint32_t X = (T.HVisible - Width) / 2;
  for (const char *C = Txt; *C; ++C) {
    uint8_t Bits = getChar(*C)[Row];
    for (uint32_t Col = 0; Col != BMapWidth; ++Col) {
      // Bit 7 is the leftmost pixel.
      bool On = Bits & (1u << (BMapWidth - 1 - Col));
      for (uint32_t Z = 0; Z != Zoom; ++Z)
        setPixel(Line, X++, On ? White : Black);
    }
  }
}

static void addSideBorder(uint8_t *Line) {
  for (uint32_t X = 0; X != Border; ++X) {
    setPixel(Line, X, White);
    setPixel(Line, T.HVisible - 1 - X, White);
  }
}

static void buildFrame() {
  char Txt[NumTxtLines][40];
  std::snprintf(Txt[0], sizeof(Txt[0]), "MCE BLASTER VGA TEST");
  std::snprintf(Txt[1], sizeof(Txt[1]), "%s @ %.2f HZ", T.Name, T.vHz());
  std::snprintf(Txt[2], sizeof(Txt[2]), "H: %.3f KHZ  V: %.2f HZ",
                T.hHz() / 1000.0f, T.vHz());
  std::snprintf(Txt[3], sizeof(Txt[3]), "PIXELCLOCK: %.3f MHZ",
                T.pixelClkHz() / 1000000.0f);
  Txt[4][0] = '\0';
  std::snprintf(Txt[5], sizeof(Txt[5]), "THANKS TO SCRAP COMPUTING");
  std::snprintf(Txt[6], sizeof(Txt[6]), "GREETINGS STEFAM");

  initLine(BlankLine, false);
  initLine(VSyncLine, true);
  initLine(FullBorderLine, false);
  for (uint32_t X = 0; X != T.HVisible; ++X)
    setPixel(FullBorderLine, X, White);
  initLine(SideBorderLine, false);
  addSideBorder(SideBorderLine);
  for (uint32_t L = 0; L != NumTxtLines; ++L)
    for (uint32_t Row = 0; Row != BMapHeight; ++Row) {
      initLine(TxtLines[L][Row], false);
      addSideBorder(TxtLines[L][Row]);
      drawTxtRow(TxtLines[L][Row], Txt[L], Row);
    }

  uint32_t TxtRowHeight = BMapHeight * Zoom + TxtLineGap;
  uint32_t TxtTop = (T.VVisible - NumTxtLines * TxtRowHeight) / 2;
  for (uint32_t Y = 0; Y != T.vTotal(); ++Y) {
    if (Y >= T.VVisible) {
      uint32_t SyncStart = T.VVisible + T.VFrontPorch;
      bool InVSync = Y >= SyncStart && Y < SyncStart + T.VSync;
      Frame[Y] = InVSync ? VSyncLine : BlankLine;
    } else if (Y < Border || Y >= T.VVisible - Border) {
      Frame[Y] = FullBorderLine;
    } else {
      Frame[Y] = SideBorderLine;
      if (Y >= TxtTop) {
        uint32_t L = (Y - TxtTop) / TxtRowHeight;
        uint32_t RowPx = (Y - TxtTop) % TxtRowHeight;
        if (L < NumTxtLines && RowPx < BMapHeight * Zoom)
          Frame[Y] = TxtLines[L][RowPx / Zoom];
      }
    }
  }
  for (uint32_t L = 0; L != NumTxtLines; ++L)
    std::printf("%s\n", Txt[L]);
}

// The control channel feeds the next line address to the data channel, so the
// hardware runs on its own. This IRQ only rewinds the line table once per
// frame and has a whole scanline of time to do so.
static void ctrlChanIRQ() {
  dma_hw->ints0 = 1u << CtrlChan;
  if (dma_hw->ch[CtrlChan].read_addr == (uintptr_t)&Frame[T.vTotal()])
    dma_channel_set_read_addr(CtrlChan, &Frame[0], /*trigger=*/false);
}

static void startVGA() {
  PIO Pio = pio0;
  uint SM = pio_claim_unused_sm(Pio, true);
  uint Offset = pio_add_program(Pio, &VGAOut_program);
  VGAOutPioConfig(Pio, SM, Offset, VGA_FIRST_GPIO, (float)T.ClkDiv);

  DataChan = dma_claim_unused_channel(true);
  CtrlChan = dma_claim_unused_channel(true);

  dma_channel_config DataConf = dma_channel_get_default_config(DataChan);
  channel_config_set_transfer_data_size(&DataConf, DMA_SIZE_32);
  channel_config_set_read_increment(&DataConf, true);
  channel_config_set_write_increment(&DataConf, false);
  channel_config_set_dreq(&DataConf, pio_get_dreq(Pio, SM, /*is_tx=*/true));
  channel_config_set_chain_to(&DataConf, CtrlChan);
  dma_channel_configure(DataChan, &DataConf, &Pio->txf[SM], nullptr,
                        T.hTotal() / 4, /*trigger=*/false);

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
  while (true) {
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
    std::printf("VGA test: %s %.2f Hz, H %.3f kHz, pixel clock %.3f MHz\n",
                T.Name, T.vHz(), T.hHz() / 1000.0f,
                T.pixelClkHz() / 1000000.0f);
    sleep_ms(2000);
  }
}
