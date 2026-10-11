//-*- C++ -*-
//
// Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
// License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
// Free for noncommercial use; commercial use only with the author's consent.
//
// Step 4: TTL input analyzer for the MCE Blaster. Once per second it measures
// the MDA/CGA/EGA input (HSync and VSync frequency, polarity and pulse width,
// lines per frame, which data pins toggle) and shows the results on a
// 640x480 VGA screen and over USB serial.
//

#include "Font8x8.h"
#include "VGAOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// Same pins as the MCE Blaster firmware (Common.h).
static constexpr uint32_t VGA_FIRST_GPIO = 14;
static constexpr uint32_t EGA_RGB_GPIO = 0;  // 0-5
static constexpr uint32_t TTL_VSYNC_GPIO = 6;
static constexpr uint32_t TTL_HSYNC_GPIO = 7;
static constexpr uint32_t CGA_RGB_GPIO = 8;  // 8-13
static constexpr uint32_t MDA_VI_GPIO = 26;  // 26-27

static constexpr uint8_t Black = 0b000000;
static constexpr uint8_t White = 0b111111;
static constexpr uint8_t Yellow = 0b111100;
static constexpr uint8_t HBit = 1u << 6;
static constexpr uint8_t VBit = 1u << 7;

// 640x480@60, 25.2 MHz pixel clock (252 MHz / 10), HSync and VSync negative.
static constexpr uint32_t SysClkKHz = 252000;
static constexpr uint32_t ClkDiv = 10;
static constexpr uint32_t HVisible = 640, HFrontPorch = 16, HSync = 96,
                          HBackPorch = 48;
static constexpr uint32_t VVisible = 480, VFrontPorch = 10, VSync = 2,
                          VBackPorch = 33;
static constexpr uint32_t HTotal = HVisible + HFrontPorch + HSync + HBackPorch;
static constexpr uint32_t VTotal = VVisible + VFrontPorch + VSync + VBackPorch;

static constexpr uint32_t Border = 2;
static constexpr uint32_t Zoom = 2;
static constexpr uint32_t NumTxtLines = 16;
static constexpr uint32_t TxtLineGap = 6;
static constexpr uint32_t MaxChars = HVisible / (FontWidth * Zoom);

// One buffer per distinct scanline; the frame is a table of line pointers.
static uint8_t BlankLine[HTotal] __attribute__((aligned(4)));
static uint8_t VSyncLine[HTotal] __attribute__((aligned(4)));
static uint8_t FullBorderLine[HTotal] __attribute__((aligned(4)));
static uint8_t SideBorderLine[HTotal] __attribute__((aligned(4)));
static uint8_t TxtLines[NumTxtLines][FontHeight][HTotal]
    __attribute__((aligned(4)));
static const uint8_t *Frame[VTotal];

static PIO Pio = pio0;
static uint SM;
static int DataChan;
static int CtrlChan;

/// Fills \p Line with black pixels and the sync levels for this line.
static void initLine(uint8_t *Line, bool InVSync) {
  uint8_t V = InVSync ? 0 : VBit;
  std::memset(Line, Black | HBit | V, HTotal);
  std::memset(Line + HVisible + HFrontPorch, Black | V, HSync);
}

static void setPixel(uint8_t *Line, uint32_t X, uint8_t Color) {
  Line[X] = (Line[X] & (HBit | VBit)) | Color;
}

static void addSideBorder(uint8_t *Line) {
  for (uint32_t X = 0; X != Border; ++X) {
    setPixel(Line, X, White);
    setPixel(Line, HVisible - 1 - X, White);
  }
}

/// Draws \p Txt left aligned (or centered) into text line \p L. The line is
/// drawn into a scratch buffer first and then copied, so the running output
/// never shows a half drawn line.
static void drawTxtLine(uint32_t L, const char *Txt, uint8_t Color,
                        bool Center) {
  static uint8_t Scratch[HTotal] __attribute__((aligned(4)));
  uint32_t Len = std::strlen(Txt);
  if (Len > MaxChars - 1)
    Len = MaxChars - 1;
  uint32_t Width = Len * FontWidth * Zoom;
  uint32_t X0 = Center ? (HVisible - Width) / 2 : FontWidth * Zoom / 2;
  for (uint32_t Row = 0; Row != FontHeight; ++Row) {
    initLine(Scratch, false);
    addSideBorder(Scratch);
    uint32_t X = X0;
    for (uint32_t I = 0; I != Len; ++I) {
      uint8_t Bits = Font8x8[Txt[I] & 0x7f][Row];
      for (uint32_t Col = 0; Col != FontWidth; ++Col) {
        // Bit 0 is the leftmost pixel.
        bool On = Bits & (1u << Col);
        for (uint32_t Z = 0; Z != Zoom; ++Z)
          setPixel(Scratch, X++, On ? Color : Black);
      }
    }
    std::memcpy(TxtLines[L][Row], Scratch, HTotal);
  }
}

static void buildFrame() {
  initLine(BlankLine, false);
  initLine(VSyncLine, true);
  initLine(FullBorderLine, false);
  for (uint32_t X = 0; X != HVisible; ++X)
    setPixel(FullBorderLine, X, White);
  initLine(SideBorderLine, false);
  addSideBorder(SideBorderLine);
  for (uint32_t L = 0; L != NumTxtLines; ++L)
    drawTxtLine(L, "", White, false);

  uint32_t TxtRowHeight = FontHeight * Zoom + TxtLineGap;
  uint32_t TxtTop = (VVisible - NumTxtLines * TxtRowHeight) / 2;
  for (uint32_t Y = 0; Y != VTotal; ++Y) {
    if (Y >= VVisible) {
      uint32_t SyncStart = VVisible + VFrontPorch;
      bool InVSync = Y >= SyncStart && Y < SyncStart + VSync;
      Frame[Y] = InVSync ? VSyncLine : BlankLine;
    } else if (Y < Border || Y >= VVisible - Border) {
      Frame[Y] = FullBorderLine;
    } else {
      Frame[Y] = SideBorderLine;
      if (Y >= TxtTop) {
        uint32_t L = (Y - TxtTop) / TxtRowHeight;
        uint32_t RowPx = (Y - TxtTop) % TxtRowHeight;
        if (L < NumTxtLines && RowPx < FontHeight * Zoom)
          Frame[Y] = TxtLines[L][RowPx / Zoom];
      }
    }
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

// ---------------------------------------------------------------------------
// TTL input measurement

static constexpr uint32_t InputMask =
    0x3fu << EGA_RGB_GPIO | 1u << TTL_VSYNC_GPIO | 1u << TTL_HSYNC_GPIO |
    0x3fu << CGA_RGB_GPIO | 0x3u << MDA_VI_GPIO;

static volatile uint32_t VSyncEdges = 0;

static void vsyncIRQ(uint GPIO, uint32_t Events) {
  if (GPIO == TTL_VSYNC_GPIO && (Events & GPIO_IRQ_EDGE_RISE))
    ++VSyncEdges;
}

struct SyncResult {
  float Hz = 0;
  float HighFraction = 0; // share of time the pin is high, 0..1
  bool positive() const { return HighFraction < 0.5f; }
  float pulseUs() const {
    float Active = positive() ? HighFraction : 1.0f - HighFraction;
    return Hz > 0 ? Active / Hz * 1e6f : 0;
  }
};

struct Measurement {
  SyncResult H, V;
  uint32_t SeenHigh = 0; // pins that were high at least once
  uint32_t SeenLow = 0;  // pins that were low at least once
};

static void initInputs() {
  // Pull-ups as in the MCE Blaster firmware: unconnected pins read high.
  for (uint32_t Pin = 0; Pin != 32; ++Pin)
    if (InputMask & (1u << Pin)) {
      gpio_init(Pin);
      gpio_set_dir(Pin, GPIO_IN);
      gpio_pull_up(Pin);
    }
  // HSync is a PWM B input (GPIO 7 = slice 3 B), so the PWM counts its rising
  // edges in hardware. VSync is slow enough for an edge interrupt.
  gpio_set_function(TTL_HSYNC_GPIO, GPIO_FUNC_PWM);
  uint Slice = pwm_gpio_to_slice_num(TTL_HSYNC_GPIO);
  pwm_config Conf = pwm_get_default_config();
  pwm_config_set_clkdiv_mode(&Conf, PWM_DIV_B_RISING);
  pwm_config_set_clkdiv(&Conf, 1.0f);
  pwm_init(Slice, &Conf, false);
  gpio_set_irq_enabled_with_callback(TTL_VSYNC_GPIO, GPIO_IRQ_EDGE_RISE, true,
                                     vsyncIRQ);
}

/// Samples all input pins for \p Ms milliseconds and counts sync edges.
static Measurement measure(uint32_t Ms) {
  uint Slice = pwm_gpio_to_slice_num(TTL_HSYNC_GPIO);
  Measurement M;
  uint32_t Samples = 0, HHigh = 0, VHigh = 0;
  uint32_t Or = 0, And = 0xffffffff;

  pwm_set_counter(Slice, 0);
  VSyncEdges = 0;
  uint64_t Start = time_us_64();
  pwm_set_enabled(Slice, true);
  uint64_t End = Start + Ms * 1000ull;
  uint64_t Now;
  // The PWM counter is 16 bits; 65535 edges in 1 s is far above any TTL mode.
  do {
    for (uint32_t I = 0; I != 256; ++I) {
      // The PWM function of GPIO 7 does not stop gpio_get_all() reading it.
      uint32_t All = gpio_get_all();
      Or |= All;
      And &= All;
      HHigh += (All >> TTL_HSYNC_GPIO) & 1;
      VHigh += (All >> TTL_VSYNC_GPIO) & 1;
    }
    Samples += 256;
    Now = time_us_64();
  } while (Now < End);
  pwm_set_enabled(Slice, false);
  float Seconds = (Now - Start) / 1e6f;

  M.H.Hz = pwm_get_counter(Slice) / Seconds;
  M.V.Hz = VSyncEdges / Seconds;
  M.H.HighFraction = (float)HHigh / Samples;
  M.V.HighFraction = (float)VHigh / Samples;
  M.SeenHigh = Or & InputMask;
  M.SeenLow = ~And & InputMask;
  return M;
}

struct TTLMode {
  const char *Name;
  float HHz, VHz;
};
// From the MCE Blaster's TimingsTTL.def. CGA and EGA 200 lines share one
// timing; the data pins that toggle tell them apart.
static constexpr TTLMode TTLModes[] = {
    {"MDA 720x350 50 Hz", 18430, 50},
    {"CGA/EGA 640x200 60 Hz", 15700, 60},
    {"EGA 640x350 60 Hz", 21850, 60},
};

static const char *guessMode(const Measurement &M) {
  if (M.H.Hz < 1000 || M.V.Hz < 10)
    return nullptr;
  for (const TTLMode &Mode : TTLModes)
    if (std::fabs(M.H.Hz - Mode.HHz) < Mode.HHz * 0.03f &&
        std::fabs(M.V.Hz - Mode.VHz) < 3)
      return Mode.Name;
  return "unknown";
}

/// One character per pin: '*' toggles, '+' always high, '-' always low.
static void pinStates(const Measurement &M, uint32_t First, uint32_t Num,
                      char *Out) {
  for (uint32_t I = 0; I != Num; ++I) {
    uint32_t Bit = 1u << (First + I);
    bool High = M.SeenHigh & Bit, Low = M.SeenLow & Bit;
    *Out++ = High && Low ? '*' : High ? '+' : '-';
    *Out++ = ' ';
  }
  Out[-1] = '\0';
}

static void show(const Measurement &M) {
  char Txt[NumTxtLines][MaxChars + 1] = {};
  const char *Mode = guessMode(M);
  std::snprintf(Txt[0], sizeof(Txt[0]), "MCE BLASTER TTL INPUT ANALYZER");
  if (M.H.Hz >= 1000)
    std::snprintf(Txt[2], sizeof(Txt[2]), "HSync %7.3f kHz %s %5.2f us",
                  M.H.Hz / 1000.0f, M.H.positive() ? "pos" : "neg",
                  M.H.pulseUs());
  else
    std::snprintf(Txt[2], sizeof(Txt[2]), "HSync none (stuck %s)",
                  M.H.HighFraction > 0.5f ? "high" : "low");
  if (M.V.Hz >= 10)
    std::snprintf(Txt[3], sizeof(Txt[3]), "VSync %7.2f Hz  %s %5.0f us",
                  M.V.Hz, M.V.positive() ? "pos" : "neg", M.V.pulseUs());
  else
    std::snprintf(Txt[3], sizeof(Txt[3]), "VSync none (stuck %s)",
                  M.V.HighFraction > 0.5f ? "high" : "low");
  if (Mode)
    std::snprintf(Txt[4], sizeof(Txt[4]), "Lines per frame: %.1f",
                  M.H.Hz / M.V.Hz);
  std::snprintf(Txt[5], sizeof(Txt[5]), "Mode: %s",
                Mode ? Mode : "NO SIGNAL");

  char States[16];
  std::snprintf(Txt[7], sizeof(Txt[7]), "Pins: * toggles + high - low");
  pinStates(M, EGA_RGB_GPIO, 6, States);
  std::snprintf(Txt[8], sizeof(Txt[8]), "EGA GPIO  0-5:  %s", States);
  pinStates(M, CGA_RGB_GPIO, 6, States);
  std::snprintf(Txt[9], sizeof(Txt[9]), "CGA GPIO  8-13: %s", States);
  pinStates(M, MDA_VI_GPIO, 2, States);
  std::snprintf(Txt[10], sizeof(Txt[10]), "MDA GPIO 26-27: %s", States);
  pinStates(M, TTL_VSYNC_GPIO, 2, States);
  std::snprintf(Txt[11], sizeof(Txt[11]), "V/H GPIO  6-7:  %s", States);
  std::snprintf(Txt[14], sizeof(Txt[14]), "Greetings SteFam");

  for (uint32_t L = 0; L != NumTxtLines; ++L) {
    bool Title = L == 0 || L == 5;
    bool Center = L == 0 || L >= 13;
    drawTxtLine(L, Txt[L], Title ? Yellow : White, Center);
  }
  for (uint32_t L = 2; L != 12; ++L)
    if (Txt[L][0])
      std::printf("%s\n", Txt[L]);
  std::printf("\n");
}

int main() {
  set_sys_clock_khz(SysClkKHz, true);
  stdio_init_all();
  buildFrame();
  startVGA();
  initInputs();

#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  while (true) {
    show(measure(1000));
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
  }
}
