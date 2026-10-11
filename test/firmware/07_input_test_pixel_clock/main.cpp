//-*- C++ -*-
//
// Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
// License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
// Free for noncommercial use; commercial use only with the author's consent.
//
// Step 7: input test with the measured pixel clock. As step 5, but core 0
// also measures the pixel clock (step 6), and core 1 samples the input once
// per input pixel at that clock instead of 320 times per line. The middle
// 640 pixels are shown at full width on a 640x480 VGA screen.
//

#include "FastSample.pio.h"
#include "Font8x8.h"
#include "InputCapture.pio.h"
#include "VGAOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

// Same pins as the MCE Blaster firmware (Common.h).
static constexpr uint32_t VGA_FIRST_GPIO = 14;
static constexpr uint32_t EGA_RGB_GPIO = 0;  // 0-5
static constexpr uint32_t TTL_VSYNC_GPIO = 6;
static constexpr uint32_t TTL_HSYNC_GPIO = 7;
static constexpr uint32_t CGA_RGB_GPIO = 8;  // 8-13
static constexpr uint32_t MDA_VI_GPIO = 26;  // 26 V, 27 I
static constexpr uint32_t PIXEL_CLOCK_BTN_GPIO = 22;
static constexpr uint32_t AUTO_ADJUST_BTN_GPIO = 28;

static constexpr uint8_t Black = 0b000000;
static constexpr uint8_t Gray = 0b101010;
static constexpr uint8_t White = 0b111111;
static constexpr uint8_t Yellow = 0b111100;
static constexpr uint8_t HBit = 1u << 6;
static constexpr uint8_t VBit = 1u << 7;

// 640x480@60, 25.2 MHz pixel clock (252 MHz / 10), HSync and VSync negative.
// Full width, but only 240 line buffers: every line is sent twice.
static constexpr uint32_t SysClkKHz = 252000;
static constexpr uint32_t ClkDiv = 10;
static constexpr uint32_t HVisible = 640, HFrontPorch = 16, HSync = 96,
                          HBackPorch = 48;
static constexpr uint32_t VVisible = 480, VFrontPorch = 10, VSync = 2,
                          VBackPorch = 33;
static constexpr uint32_t HTotal = HVisible + HFrontPorch + HSync + HBackPorch;
static constexpr uint32_t VTotal = VVisible + VFrontPorch + VSync + VBackPorch;
static constexpr uint32_t Rows = VVisible / 2;
static_assert(HTotal % 4 == 0, "DMA sends 4 bytes per transfer");

// Visible pixels carry the idle sync levels.
static constexpr uint8_t Idle = HBit | VBit;

static uint8_t BlankLine[HTotal] __attribute__((aligned(4)));
static uint8_t VSyncLine[HTotal] __attribute__((aligned(4)));
static uint8_t OutLines[Rows][HTotal] __attribute__((aligned(4)));
static const uint8_t *Frame[VTotal];

static PIO VGAPio = pio0;
static uint VGASM;
static int DataChan;
static int CtrlChan;

static void initLine(uint8_t *Line, bool InVSync) {
  uint8_t V = InVSync ? 0 : VBit;
  std::memset(Line, Black | HBit | V, HTotal);
  std::memset(Line + HVisible + HFrontPorch, Black | V, HSync);
}

static void buildFrame() {
  initLine(BlankLine, false);
  initLine(VSyncLine, true);
  for (uint32_t R = 0; R != Rows; ++R)
    initLine(OutLines[R], false);
  for (uint32_t Y = 0; Y != VTotal; ++Y) {
    uint32_t SyncStart = VVisible + VFrontPorch;
    if (Y < VVisible)
      Frame[Y] = OutLines[Y / 2];
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
  VGASM = pio_claim_unused_sm(VGAPio, true);
  uint Offset = pio_add_program(VGAPio, &VGAOut_program);
  VGAOutPioConfig(VGAPio, VGASM, Offset, VGA_FIRST_GPIO, (float)ClkDiv);

  DataChan = dma_claim_unused_channel(true);
  CtrlChan = dma_claim_unused_channel(true);

  dma_channel_config DataConf = dma_channel_get_default_config(DataChan);
  channel_config_set_transfer_data_size(&DataConf, DMA_SIZE_32);
  channel_config_set_read_increment(&DataConf, true);
  channel_config_set_write_increment(&DataConf, false);
  channel_config_set_dreq(&DataConf,
                          pio_get_dreq(VGAPio, VGASM, /*is_tx=*/true));
  channel_config_set_chain_to(&DataConf, CtrlChan);
  dma_channel_configure(DataChan, &DataConf, &VGAPio->txf[VGASM], nullptr,
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

  pio_sm_set_enabled(VGAPio, VGASM, true);
  dma_channel_start(CtrlChan);
}

// ---------------------------------------------------------------------------
// Text overlay: a block of lines, centered on the screen, every line left
// aligned with the title.

static constexpr uint32_t NumTxtLines = 17;
static constexpr uint32_t MaxChars = 36;
static constexpr uint32_t TxtRowHeight = FontHeight + 2;

struct Overlay {
  char Lines[NumTxtLines][MaxChars + 1] = {};
  uint32_t Width = 0; // longest line in characters
};
// Core 0 writes the overlay and then flips Shown; core 1 only reads Shown.
static Overlay Overlays[2];
static volatile uint32_t Shown = 0;

// The PIXEL CLOCK button hides and shows the text.
static volatile bool OverlayOn = true;

// The text is drawn into two bitmaps once per update (glyph pixels and their
// black outline, one bit per font pixel), so core 1 can put it on top of each
// captured line within the line time.
static constexpr uint32_t BlockW = MaxChars * FontWidth + 2; // font pixels
static constexpr uint32_t MaskBytes = (BlockW + 7) / 8;
static constexpr uint32_t BlockH = NumTxtLines * TxtRowHeight + 2;
static uint8_t TextMask[BlockH][MaskBytes];
static uint8_t EdgeMask[BlockH][MaskBytes];
static uint32_t MaskX = 0, MaskY = 0; // screen position of the block
static uint32_t MaskW = 0;            // used width in font pixels

static bool textPixel(const Overlay &O, int32_t Y, int32_t X) {
  if (Y < 0 || X < 0)
    return false;
  uint32_t L = Y / TxtRowHeight, FontRow = Y % TxtRowHeight;
  if (L >= NumTxtLines || FontRow >= FontHeight)
    return false;
  uint32_t Char = X / FontWidth;
  if (Char >= std::strlen(O.Lines[L]))
    return false;
  return Font8x8[O.Lines[L][Char] & 0x7f][FontRow] & (1u << (X % FontWidth));
}

static void buildMasks(const Overlay &O) {
  std::memset(TextMask, 0, sizeof(TextMask));
  std::memset(EdgeMask, 0, sizeof(EdgeMask));
  MaskW = O.Width * FontWidth + 2;
  // Two screen pixels per font pixel horizontally, rows are shown twice.
  MaskX = (HVisible - 2 * MaskW) / 2;
  MaskY = (Rows - BlockH) / 2;
  for (int32_t Y = 0; Y != (int32_t)BlockH; ++Y)
    for (int32_t X = 0; X != (int32_t)MaskW; ++X) {
      // Mask coordinates have a one pixel margin for the outline.
      bool Text = textPixel(O, Y - 1, X - 1);
      bool Edge = false;
      for (int32_t DY = -1; DY <= 1 && !Text && !Edge; ++DY)
        for (int32_t DX = -1; DX <= 1 && !Edge; ++DX)
          Edge = textPixel(O, Y - 1 + DY, X - 1 + DX);
      if (Text)
        TextMask[Y][X / 8] |= 1u << (X % 8);
      else if (Edge)
        EdgeMask[Y][X / 8] |= 1u << (X % 8);
    }
}

/// Puts the text on top of output row \p Row.
static void __not_in_flash_func(applyMasks)(uint32_t Row, uint8_t *Out) {
  if (!OverlayOn || Row < MaskY || Row >= MaskY + BlockH)
    return;
  uint32_t Y = Row - MaskY;
  // The title line is yellow.
  uint8_t Color = ((Y < TxtRowHeight + 1) ? Yellow : White) | Idle;
  const uint8_t *T = TextMask[Y], *E = EdgeMask[Y];
  uint8_t *P = Out + MaskX;
  for (uint32_t B = 0; B != MaskBytes; ++B, P += 16) {
    uint32_t Any = T[B] | E[B];
    if (!Any)
      continue;
    for (uint32_t I = 0; I != 8; ++I) {
      if (!(Any & (1u << I)))
        continue;
      uint8_t C = (T[B] & (1u << I)) ? Color : (Black | Idle);
      P[2 * I] = P[2 * I + 1] = C;
    }
  }
}

// ---------------------------------------------------------------------------
// Hang diagnostics: each core writes where it is into a watchdog scratch
// register, which survives the watchdog reset. After such a reset the screen
// and the USB console show the last position of both cores.

enum Stage : uint32_t {
  StageBoot = 1,
  StageMeasure,
  StagePixelClock,
  StageOverlay,
  StagePrint,
  StageCapWait = 11,
  StageCapLines,
  StageCapIdle,
  StageCapSetup,
};
static void stage(uint32_t Core, Stage S) { watchdog_hw->scratch[Core] = S; }
static bool WatchdogReset = false;
static uint32_t ResetStage[2] = {};

// ---------------------------------------------------------------------------
// Picture capture on core 1, one sample per input pixel

enum class Source : uint32_t { None, EGA, CGA, MDA };

struct CaptureSetup {
  Source Src = Source::None;
  bool PosHSync = true;
  bool PosVSync = true;
  uint32_t LinesPerFrame = 0;
  float PixelHz = 0;
  // Per line: skip SkipPx pixels after HSync, then take Samples pixels, of
  // which VisiblePx after the margin are scaled to the 640 output pixels.
  uint32_t SkipPx = 0, Samples = 0, VisiblePx = 0;
};
// Written by core 0, read by core 1. A torn read only costs one bad frame.
static volatile uint32_t SetupSeq = 0;
static CaptureSetup Setup;
// AUTO ADJUST shifts the sampling point by half an input pixel.
static volatile uint32_t Phase = 0;

// Pixels sampled before and after the visible part, so a slightly wrong
// back porch does not cut the picture.
static constexpr uint32_t MarginPx = 8;
static constexpr uint32_t MaxSamples = 768;
// Input pixel for each output pixel. 720 to 640 drops every ninth pixel,
// which in VGA text is the empty ninth column of a character cell.
static uint16_t XMap[HVisible];

enum class CaptureFail : uint32_t { None, NoVSync, VSyncEnd, Line };
static volatile uint32_t FramesOk = 0;
static volatile uint32_t FramesFailed = 0;
static volatile CaptureFail LastFail = CaptureFail::None;
static volatile uint32_t LastFailLine = 0;

// One DMA channel writes the whole frame into a ring buffer. The DMA write
// ring wraps the address in hardware, so a capture that runs ahead of core 1
// can only overwrite old samples, never other memory.
static constexpr uint32_t RingBits = 13; // 8 KB, ten lines
static constexpr uint32_t RingBytes = 1u << RingBits;
static uint8_t Ring[RingBytes] __attribute__((aligned(RingBytes)));
static uint8_t ColorLUT[256];

static PIO CapPio = pio1;
static uint CapSM;
static int CapChan;
static uint PosOffset, NegOffset;

static void buildLUT(Source Src) {
  for (uint32_t I = 0; I != 256; ++I) {
    uint8_t C = Black;
    switch (Src) {
    case Source::EGA:
    case Source::CGA:
      // The MCE Blaster copies these 6 bits unchanged to the VGA DAC.
      C = I & 0x3f;
      break;
    case Source::MDA: {
      bool Video = I & 0b01, Intensity = I & 0b10;
      C = Video ? (Intensity ? White : Gray) : Black;
      break;
    }
    case Source::None:
      break;
    }
    ColorLUT[I] = C | Idle;
  }
}

static uint32_t firstGPIO(Source Src) {
  switch (Src) {
  case Source::EGA:
    return EGA_RGB_GPIO;
  case Source::MDA:
    return MDA_VI_GPIO;
  default:
    return CGA_RGB_GPIO;
  }
}

static void configCapture(const CaptureSetup &S) {
  pio_sm_set_enabled(CapPio, CapSM, false);
  // Two SM cycles per sample: one sample per input pixel.
  float Div = SysClkKHz * 1000.0f / (2 * S.PixelHz);
  if (Div < 1)
    Div = 1;
  uint Offset = S.PosHSync ? PosOffset : NegOffset;
  pio_sm_config Conf = CaptureConfig(Offset, S.PosHSync, firstGPIO(S.Src), Div);
  pio_sm_init(CapPio, CapSM, Offset, &Conf);
}

static bool __not_in_flash_func(waitPin)(uint32_t GPIO, bool Level,
                                         uint64_t Deadline) {
  while (gpio_get(GPIO) != Level)
    if (time_us_64() > Deadline)
      return false;
  return true;
}

static void __not_in_flash_func(stopCapture)() {
  pio_sm_set_enabled(CapPio, CapSM, false);
  dma_channel_abort(CapChan);
}

/// Captures one frame straight into the output lines. \returns false
/// without a signal.
static bool __not_in_flash_func(captureFrame)(const CaptureSetup &S) {
  uint64_t Deadline = time_us_64() + 100000;
  stage(1, StageCapWait);
  if (!waitPin(TTL_VSYNC_GPIO, !S.PosVSync, Deadline) ||
      !waitPin(TTL_VSYNC_GPIO, S.PosVSync, Deadline)) {
    LastFail = CaptureFail::NoVSync;
    return false;
  }
  uint Offset = S.PosHSync ? PosOffset : NegOffset;
  pio_sm_clear_fifos(CapPio, CapSM);
  pio_sm_restart(CapPio, CapSM);
  pio_sm_exec(CapPio, CapSM, pio_encode_jmp(Offset));
  // First the phase delay, then the sample count (see InputCapture.pio).
  // The delay loop takes one SM cycle, half a pixel, per count.
  pio_sm_put(CapPio, CapSM, 2 * S.SkipPx + Phase);
  pio_sm_put(CapPio, CapSM, S.Samples - 1);
  uint32_t WordsPerLine = S.Samples / 4;
  uint32_t Words = S.LinesPerFrame * WordsPerLine;
  dma_channel_set_write_addr(CapChan, Ring, false);
  dma_channel_set_trans_count(CapChan, Words, false);
  if (!waitPin(TTL_VSYNC_GPIO, !S.PosVSync, Deadline)) {
    LastFail = CaptureFail::VSyncEnd;
    return false;
  }
  stage(1, StageCapLines);
  dma_channel_start(CapChan);
  pio_sm_set_enabled(CapPio, CapSM, true);

  uint32_t LastRow = ~0u;
  for (uint32_t Line = 0; Line != S.LinesPerFrame; ++Line) {
    uint32_t Needed = (Line + 1) * WordsPerLine;
    uint64_t LineDeadline = time_us_64() + 1000;
    while (Words - dma_channel_hw_addr(CapChan)->transfer_count < Needed)
      if (time_us_64() > LineDeadline) {
        stopCapture();
        LastFail = CaptureFail::Line;
        LastFailLine = Line;
        return false;
      }

    uint32_t Row = Line * Rows / S.LinesPerFrame;
    if (Row == LastRow)
      continue;
    LastRow = Row;
    uint8_t *Out = OutLines[Row];
    uint32_t Start = Line * S.Samples;
    for (uint32_t X = 0; X != HVisible; ++X)
      Out[X] = ColorLUT[Ring[(Start + XMap[X]) & (RingBytes - 1)]];
    applyMasks(Row, Out);
  }
  stopCapture();
  return true;
}

// Core 1 gets its own stack in main RAM. Its default stack lies directly
// below core 0's, so a deep printf on core 0 could overwrite it.
static uint32_t Core1Stack[1024];

static void core1Main() {
  CapSM = pio_claim_unused_sm(CapPio, true);
  PosOffset = pio_add_program(CapPio, &CapturePosHSync_program);
  NegOffset = pio_add_program(CapPio, &CaptureNegHSync_program);
  CapChan = dma_claim_unused_channel(true);
  dma_channel_config Conf = dma_channel_get_default_config(CapChan);
  channel_config_set_transfer_data_size(&Conf, DMA_SIZE_32);
  channel_config_set_read_increment(&Conf, false);
  channel_config_set_write_increment(&Conf, true);
  channel_config_set_ring(&Conf, /*write=*/true, RingBits);
  channel_config_set_high_priority(&Conf, true);
  channel_config_set_dreq(&Conf, pio_get_dreq(CapPio, CapSM, /*is_tx=*/false));
  dma_channel_configure(CapChan, &Conf, Ring, &CapPio->rxf[CapSM], 0,
                        /*trigger=*/false);

  uint32_t SeenSeq = ~0u;
  uint32_t SeenShown = ~0u;
  CaptureSetup S;
  while (true) {
    if (SeenShown != Shown) {
      SeenShown = Shown;
      buildMasks(Overlays[SeenShown]);
    }
    if (SeenSeq != SetupSeq) {
      SeenSeq = SetupSeq;
      stage(1, StageCapSetup);
      __dmb();
      S = Setup;
      buildLUT(S.Src);
      for (uint32_t X = 0; X != HVisible; ++X)
        XMap[X] = MarginPx + X * S.VisiblePx / HVisible;
      if (S.Src != Source::None)
        configCapture(S);
    }
    bool HavePicture = S.Src != Source::None && captureFrame(S);
    if (S.Src != Source::None) {
      if (HavePicture)
        FramesOk = FramesOk + 1;
      else
        FramesFailed = FramesFailed + 1;
    }
    if (!HavePicture) {
      stage(1, StageCapIdle);
      for (uint32_t R = 0; R != Rows; ++R) {
        std::memset(OutLines[R], Black | Idle, HVisible);
        applyMasks(R, OutLines[R]);
      }
      sleep_ms(20);
    }
  }
}

// ---------------------------------------------------------------------------
// TTL input measurement on core 0 (as in step 4)

static constexpr uint32_t InputMask =
    0x3fu << EGA_RGB_GPIO | 1u << TTL_VSYNC_GPIO | 1u << TTL_HSYNC_GPIO |
    0x3fu << CGA_RGB_GPIO | 0x3u << MDA_VI_GPIO;

static volatile uint32_t VSyncEdges = 0;
// Times of the first and last edge: counting edges in 1 s alone gives only
// whole hertz, so 70 Hz flips between 70 and 71.
static volatile uint64_t FirstVSyncUs = 0, LastVSyncUs = 0;

static void gpioIRQ(uint GPIO, uint32_t Events) {
  if (GPIO == TTL_VSYNC_GPIO && (Events & GPIO_IRQ_EDGE_RISE)) {
    LastVSyncUs = time_us_64();
    if (VSyncEdges++ == 0)
      FirstVSyncUs = LastVSyncUs;
  } else if (GPIO == PIXEL_CLOCK_BTN_GPIO || GPIO == AUTO_ADJUST_BTN_GPIO) {
    // Debounce: ignore presses within 200 ms of the last one.
    static uint64_t LastPress = 0;
    uint64_t Now = time_us_64();
    if (Now - LastPress > 200000) {
      if (GPIO == PIXEL_CLOCK_BTN_GPIO)
        OverlayOn = !OverlayOn;
      else
        Phase = Phase ^ 1;
    }
    LastPress = Now;
  }
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
  bool toggles(uint32_t First, uint32_t Num) const {
    uint32_t Mask = ((1u << Num) - 1) << First;
    return SeenHigh & SeenLow & Mask;
  }
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
                                     gpioIRQ);
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
  uint32_t Edges = VSyncEdges;
  M.V.Hz = Edges >= 2 ? (Edges - 1) * 1e6f / (LastVSyncUs - FirstVSyncUs)
                      : Edges / Seconds;
  M.H.HighFraction = (float)HHigh / Samples;
  M.V.HighFraction = (float)VHigh / Samples;
  M.SeenHigh = Or & InputMask;
  M.SeenLow = ~And & InputMask;
  return M;
}

struct TTLMode {
  const char *Name;
  float HHz, VHz;
  float PixelHz; // the card's usual pixel clock
  // Pixels from the end of HSync to the picture, and picture width; 0 when
  // not known, then most of the line is shown.
  uint32_t BackPorchPx, VisiblePx;
};
// MDA/CGA/EGA from the MCE Blaster's TimingsTTL.def. CGA and EGA 200 lines
// share one timing. The VGA modes are not supported by the MCE Blaster; they
// are listed because VGA cards with a 9-pin TTL output send them.
static constexpr TTLMode TTLModes[] = {
    // Porches from the standard CRTC values (MDA, CGA) and VESA timings.
    {"MDA 720x350 50Hz", 18430, 50, 16257000, 9, 720},
    {"CGA/EGA 640x200 60Hz", 15700, 60, 14318181.0f, 112, 640},
    {"EGA 640x350 60Hz", 21850, 60, 16257000, 0, 0},
    {"VGA 720x400 70Hz", 31469, 70, 28322000, 54, 720},
    {"VGA 640x480 60Hz", 31469, 60, 25175000, 48, 640},
    // Measured on a Trident TVGA 9-pin output in its non-VGA switch setting;
    // the pixel clock is unknown, so it is measured only.
    {"TVGA 200 lines 117Hz", 31055, 117.6f, 0, 0, 0},
};

static const char *guessMode(const Measurement &M) {
  if (M.H.Hz < 1000 || M.V.Hz < 10)
    return nullptr;
  for (const TTLMode &Mode : TTLModes)
    if (std::fabs(M.H.Hz - Mode.HHz) < Mode.HHz * 0.03f &&
        std::fabs(M.V.Hz - Mode.VHz) < 3.0f)
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

static Source pickSource(const Measurement &M, const char *Mode) {
  if (!Mode)
    return Source::None;
  if (M.toggles(CGA_RGB_GPIO, 6))
    return Source::CGA;
  if (M.toggles(EGA_RGB_GPIO, 6))
    return Source::EGA;
  if (M.toggles(MDA_VI_GPIO, 2))
    return Source::MDA;
  return Source::CGA;
}

static const char *sourceName(Source Src) {
  switch (Src) {
  case Source::EGA:
    return "EGA pins 0-5";
  case Source::CGA:
    return "CGA pins 8-13";
  case Source::MDA:
    return "MDA pins 26-27";
  case Source::None:
    break;
  }
  return "none";
}

// ---------------------------------------------------------------------------
// Pixel clock from the run lengths of the colour pins

static constexpr uint32_t SampleClkDiv = 2;
static constexpr float SampleHz = SysClkKHz * 1000.0f / SampleClkDiv;
static constexpr uint32_t BurstWords = 2048; // 8192 samples, 65 us
static constexpr uint32_t NumBursts = 32;
static constexpr uint32_t MaxRun = 256; // longer runs are blanking
static uint32_t Burst[BurstWords];
static uint32_t RunCount[MaxRun];
// Sample positions of the rising edges of each pin in one burst.
// RAM is tight here (240 output lines of 800 bytes), hence 6 pins only.
static constexpr uint32_t MaxEdges = 1024;
static uint16_t Edges[6][MaxEdges];
static uint32_t NumEdges[6];

static PIO SamplePio = pio1;
static uint SampleSM;
static uint SampleOffset;
static int SampleChan;

struct PixelClock {
  uint32_t Runs = 0;
  float PixelSamples = 0; // pixel time in samples
  float FitPx = 0;        // rms deviation of the runs, in pixels
  uint32_t PixelsPerLine = 0;
  float Hz = 0;
};

static void initFastSample() {
  SampleSM = pio_claim_unused_sm(SamplePio, true);
  SampleOffset = pio_add_program(SamplePio, &FastSample_program);
  SampleChan = dma_claim_unused_channel(true);
}

/// Samples \p FirstGPIO.. for one burst into Burst[].
static void captureBurst(uint32_t FirstGPIO) {
  FastSamplePioInit(SamplePio, SampleSM, SampleOffset, FirstGPIO);
  pio_sm_set_clkdiv_int_frac(SamplePio, SampleSM, SampleClkDiv, 0);
  dma_channel_config Conf = dma_channel_get_default_config(SampleChan);
  channel_config_set_transfer_data_size(&Conf, DMA_SIZE_32);
  channel_config_set_read_increment(&Conf, false);
  channel_config_set_write_increment(&Conf, true);
  channel_config_set_high_priority(&Conf, true);
  channel_config_set_dreq(&Conf,
                          pio_get_dreq(SamplePio, SampleSM, /*is_tx=*/false));
  dma_channel_configure(SampleChan, &Conf, Burst, &SamplePio->rxf[SampleSM],
                        BurstWords, /*trigger=*/true);
  pio_sm_set_enabled(SamplePio, SampleSM, true);
  dma_channel_wait_for_finish_blocking(SampleChan);
  pio_sm_set_enabled(SamplePio, SampleSM, false);
}

/// Adds the run lengths of the pins in \p Mask of Burst[] to RunCount.
static void countRuns(uint8_t Mask) {
  const uint8_t *Samples = (const uint8_t *)Burst;
  uint8_t Prev = Samples[0] & Mask;
  uint32_t Run = 0;
  bool First = true; // the first run started before the burst
  for (uint32_t I = 1; I != BurstWords * 4; ++I) {
    uint8_t Cur = Samples[I] & Mask;
    ++Run;
    if (Cur == Prev)
      continue;
    if (!First && Run < MaxRun)
      ++RunCount[Run];
    First = false;
    Run = 0;
    Prev = Cur;
  }
}

/// Collects the rising edges of every pin in \p Mask of Burst[].
static void findEdges(uint8_t Mask) {
  const uint8_t *Samples = (const uint8_t *)Burst;
  std::memset(NumEdges, 0, sizeof(NumEdges));
  for (uint32_t I = 1; I != BurstWords * 4; ++I) {
    uint8_t Rising = Samples[I] & ~Samples[I - 1] & Mask;
    for (uint32_t B = 0; Rising && B != 6; ++B, Rising >>= 1)
      if ((Rising & 1) && NumEdges[B] != MaxEdges)
        Edges[B][NumEdges[B]++] = I;
  }
}

/// One refinement pass over the edges of one burst: every pair of rising
/// edges of the same pin about \p MinPx..2*\p MinPx pixels apart is a whole
/// number of pixels apart. Rising edges of the same pin only, so a slower
/// rise than fall does not bias the result. \returns the new pixel time.
static float refine(float PixelSamples, float MinPx, float &SumD,
                    float &SumK) {
  SumD = SumK = 0;
  float WMin = MinPx * PixelSamples, WMax = 2 * WMin;
  for (uint32_t B = 0; B != 6; ++B) {
    const uint16_t *E = Edges[B];
    uint32_t J = 0;
    for (uint32_t I = 0; I != NumEdges[B]; ++I) {
      while (J != NumEdges[B] && E[J] - E[I] < WMin)
        ++J;
      if (J == NumEdges[B])
        break;
      float D = E[J] - E[I];
      if (D > WMax)
        continue;
      float K = std::round(D / PixelSamples);
      if (std::fabs(D - K * PixelSamples) > 0.3f * PixelSamples)
        continue;
      SumD += D;
      SumK += K;
    }
  }
  return SumK > 0 ? SumD / SumK : PixelSamples;
}

static PixelClock measurePixelClock(uint32_t FirstGPIO, uint8_t Mask,
                                    float HHz) {
  PixelClock P;
  std::memset(RunCount, 0, sizeof(RunCount));
  for (uint32_t B = 0; B != NumBursts; ++B) {
    captureBurst(FirstGPIO);
    countRuns(Mask);
    // Not a multiple of the line time, so the bursts land on other lines.
    sleep_us(3137);
  }
  for (uint32_t R = 1; R != MaxRun; ++R)
    P.Runs += RunCount[R];
  if (P.Runs < 50)
    return P;

  // One pixel: the shortest run length that occurs often.
  uint32_t Peak = 0;
  for (uint32_t R = 2; R + 1 < MaxRun && !Peak; ++R)
    if (RunCount[R] * 50 >= P.Runs && RunCount[R] >= RunCount[R - 1] &&
        RunCount[R] >= RunCount[R + 1])
      Peak = R;
  if (!Peak)
    return P;
  float Sum = 0, Cnt = 0;
  for (uint32_t R = Peak - 1; R <= Peak + 1; ++R) {
    Sum += (float)R * RunCount[R];
    Cnt += RunCount[R];
  }
  float First = Sum / Cnt;

  // All runs are whole multiples of the pixel time; averaging over all of
  // them makes the estimate much finer than one sample.
  float SumR = 0, SumK = 0;
  for (uint32_t R = 2; R != MaxRun; ++R) {
    uint32_t K = (uint32_t)std::lround(R / First);
    if (K == 0 || K > 16 || std::fabs(R / (float)K - First) > First * 0.25f)
      continue;
    SumR += (float)R * RunCount[R];
    SumK += (float)K * RunCount[R];
  }
  if (SumK == 0)
    return P;
  P.PixelSamples = SumR / SumK;

  // Long distances: a whole number of pixels between two edges hundreds of
  // pixels apart pins the pixel time down to a small fraction of a sample.
  // Each pass widens the distance as far as the previous estimate allows.
  float TotalD = 0, TotalK = 0;
  for (uint32_t B = 0; B != NumBursts; ++B) {
    captureBurst(FirstGPIO);
    findEdges(Mask);
    float Est = P.PixelSamples, D = 0, K = 0;
    for (float MinPx : {8.0f, 40.0f, 200.0f})
      Est = refine(Est, MinPx, D, K);
    TotalD += D;
    TotalK += K;
    sleep_us(3137);
  }
  if (TotalK > 0)
    P.PixelSamples = TotalD / TotalK;
  float Dev = 0, DevCnt = 0;
  for (uint32_t R = 2; R != MaxRun; ++R) {
    uint32_t K = (uint32_t)std::lround(R / P.PixelSamples);
    if (K == 0 || K > 16)
      continue;
    float D = (R - K * P.PixelSamples) / P.PixelSamples;
    Dev += D * D * RunCount[R];
    DevCnt += RunCount[R];
  }
  P.FitPx = DevCnt > 0 ? std::sqrt(Dev / DevCnt) : 0;

  float Hz = SampleHz / P.PixelSamples;
  if (HHz > 1000) {
    P.PixelsPerLine = (uint32_t)std::lround(Hz / HHz);
    P.Hz = P.PixelsPerLine * HHz;
  } else {
    P.Hz = Hz;
  }
  return P;
}

struct KnownClock {
  float Hz;
  const char *Name;
};
static constexpr KnownClock KnownClocks[] = {
    {14318181.0f, "CGA"},
    {16257000, "EGA/MDA"},
    {25175000, "VGA 640"},
    {28322000, "VGA 720"},
    {28636363.0f, "8x NTSC"},
};

static PixelClock LastClock;

// The clock the capture runs at: the measurement snapped to the nearest known
// clock, or the mode's usual clock while there is no measurement. Snapping
// keeps a small measuring error from smearing the end of each line.
static constexpr float SnapTolerance = 0.015f; // relative
struct CaptureClock {
  float Hz = 0;
  uint32_t PixelsPerLine = 0;
  const char *Name = nullptr; // known clock, nullptr if measured only
  bool FromMode = false;      // no measurement, the mode's usual clock
};

static CaptureClock captureClock(const char *Mode, float HHz,
                                 const PixelClock &Measured) {
  CaptureClock C;
  float Best = SnapTolerance;
  for (const KnownClock &K : KnownClocks) {
    float Off = std::fabs(Measured.Hz - K.Hz) / K.Hz;
    if (Measured.Hz > 0 && Off < Best) {
      Best = Off;
      C.Hz = K.Hz;
      C.Name = K.Name;
    }
  }
  if (Measured.Hz > 0 && !C.Name) {
    C.Hz = Measured.Hz;
  } else if (Measured.Hz == 0) {
    for (const TTLMode &M : TTLModes)
      if (M.Name == Mode)
        C.Hz = M.PixelHz;
    for (const KnownClock &K : KnownClocks)
      if (K.Hz == C.Hz)
        C.Name = K.Name;
    C.FromMode = C.Hz > 0;
  }
  if (C.Hz == 0 || HHz < 1000)
    return C;
  // A whole number of pixels per line, at the line rate actually measured.
  C.PixelsPerLine = (uint32_t)std::lround(C.Hz / HHz);
  C.Hz = C.PixelsPerLine * HHz;
  return C;
}

// The detected input is latched and held until a measurement leaves it by
// more than these thresholds for LatchMisses seconds in a row, so a single
// bad measurement neither drops nor changes the mode.
static constexpr float LatchHTolerance = 0.02f; // relative
static constexpr float LatchVToleranceHz = 2.0f;
static constexpr uint32_t LatchMisses = 3;

struct LatchedInput {
  bool Valid = false;
  const char *Mode = nullptr;
  float HHz = 0, VHz = 0;
  bool PosHSync = true, PosVSync = true;
  Source Src = Source::None;
};
static LatchedInput Latch;
static uint32_t Misses = 0;

static bool matchesLatch(const Measurement &M) {
  return M.H.Hz >= 1000 && M.V.Hz >= 10 &&
         std::fabs(M.H.Hz - Latch.HHz) < Latch.HHz * LatchHTolerance &&
         std::fabs(M.V.Hz - Latch.VHz) < LatchVToleranceHz &&
         M.H.positive() == Latch.PosHSync && M.V.positive() == Latch.PosVSync;
}

static bool sourceToggles(const Measurement &M, Source Src) {
  switch (Src) {
  case Source::EGA:
    return M.toggles(EGA_RGB_GPIO, 6);
  case Source::CGA:
    return M.toggles(CGA_RGB_GPIO, 6);
  case Source::MDA:
    return M.toggles(MDA_VI_GPIO, 2);
  case Source::None:
    break;
  }
  return false;
}

static void updateLatch(const Measurement &M) {
  const char *Mode = guessMode(M);
  if (Latch.Valid && matchesLatch(M)) {
    Misses = 0;
    // A blank screen toggles no colour pin, so the source picked then is a
    // guess; switch once another source shows a picture.
    Source Src = pickSource(M, Mode);
    if (!sourceToggles(M, Latch.Src) && sourceToggles(M, Src)) {
      Latch.Src = Src;
      LastClock = PixelClock();
    }
    return;
  }
  if (Latch.Valid && ++Misses < LatchMisses)
    return;
  Misses = 0;
  LastClock = PixelClock();
  Latch = LatchedInput();
  if (!Mode)
    return;
  Latch.Valid = true;
  Latch.Mode = Mode;
  Latch.HHz = M.H.Hz;
  Latch.VHz = M.V.Hz;
  Latch.PosHSync = M.H.positive();
  Latch.PosVSync = M.V.positive();
  Latch.Src = pickSource(M, Mode);
}

static void update(const Measurement &M) {
  updateLatch(M);

  // The pixel clock briefly disturbs the capture, so it is measured once per
  // latched input and retried only while it has no result.
  if (Latch.Valid && LastClock.Hz == 0) {
    stage(0, StagePixelClock);
    uint8_t Mask = Latch.Src == Source::MDA ? 0x3 : 0x3f;
    PixelClock P = measurePixelClock(firstGPIO(Latch.Src), Mask, Latch.HHz);
    if (P.Hz > 0)
      LastClock = P;
  }

  CaptureClock Clock = captureClock(Latch.Mode, Latch.HHz, LastClock);
  CaptureSetup New;
  New.Src = Latch.Valid && Clock.Hz > 0 ? Latch.Src : Source::None;
  New.PosHSync = Latch.PosHSync;
  New.PosVSync = Latch.PosVSync;
  New.PixelHz = Clock.Hz;
  New.LinesPerFrame =
      Latch.Valid ? (uint32_t)std::lround(Latch.HHz / Latch.VHz) : 0;
  const TTLMode *Mode = nullptr;
  for (const TTLMode &T : TTLModes)
    if (T.Name == Latch.Mode)
      Mode = &T;
  if (Mode && Mode->VisiblePx) {
    New.SkipPx = Mode->BackPorchPx > MarginPx ? Mode->BackPorchPx - MarginPx : 0;
    New.VisiblePx = Mode->VisiblePx;
  } else {
    // Unknown timing: four fifths of the line, which stays clear of the next
    // HSync in every mode.
    New.VisiblePx = Clock.PixelsPerLine * 4 / 5 - 2 * MarginPx;
  }
  New.Samples = (New.VisiblePx + 2 * MarginPx + 3) / 4 * 4;
  if (New.Samples > MaxSamples) {
    New.Samples = MaxSamples;
    New.VisiblePx = MaxSamples - 2 * MarginPx;
  }
  if (New.Samples < 4 * MarginPx)
    New.Src = Source::None;
  bool Changed = New.Src != Setup.Src || New.PosHSync != Setup.PosHSync ||
                 New.PosVSync != Setup.PosVSync ||
                 New.LinesPerFrame != Setup.LinesPerFrame ||
                 New.PixelHz != Setup.PixelHz || New.SkipPx != Setup.SkipPx ||
                 New.Samples != Setup.Samples ||
                 New.VisiblePx != Setup.VisiblePx;
  if (Changed) {
    Setup = New;
    __dmb();
    SetupSeq = SetupSeq + 1;
  }

  stage(0, StageOverlay);
  Overlay &O = Overlays[!Shown];
  // Cleared in place: a temporary Overlay would cost core 0 600 bytes of its
  // small stack.
  std::memset(&O, 0, sizeof(O));
  auto Line = [&O](uint32_t L, const char *Fmt, auto... Args) {
    std::snprintf(O.Lines[L], sizeof(O.Lines[L]), Fmt, Args...);
  };
  Line(0, "MCE BLASTER - INPUT TEST");
  if (M.H.Hz >= 1000)
    Line(2, "HSync %7.3f kHz %s %5.2f us", M.H.Hz / 1000.0f,
         M.H.positive() ? "pos" : "neg", M.H.pulseUs());
  else
    Line(2, "HSync none (stuck %s)", M.H.HighFraction > 0.5f ? "high" : "low");
  if (M.V.Hz >= 10)
    Line(3, "VSync %7.2f Hz  %s %5.0f us", M.V.Hz,
         M.V.positive() ? "pos" : "neg", M.V.pulseUs());
  else
    Line(3, "VSync none (stuck %s)", M.V.HighFraction > 0.5f ? "high" : "low");
  if (M.H.Hz >= 1000 && M.V.Hz >= 10)
    Line(4, "Lines per frame: %.1f", M.H.Hz / M.V.Hz);
  if (!Latch.Valid)
    Line(5, "Mode: NO SIGNAL");
  else if (Misses)
    Line(5, "Mode: %s (held %lu/%lu)", Latch.Mode, Misses, LatchMisses);
  else
    Line(5, "Mode: %s", Latch.Mode);
  if (Clock.Hz > 0)
    Line(6, "Pixel clock: %.3f MHz, %lu/line", Clock.Hz / 1e6f,
         Clock.PixelsPerLine);
  if (LastClock.Hz > 0)
    Line(7, "Measured %.3f MHz%s%s", LastClock.Hz / 1e6f,
         Clock.Name ? ", snap " : "", Clock.Name ? Clock.Name : "");
  else if (Latch.Valid)
    Line(7, "Measured: no detail, %s", Clock.FromMode ? "mode clock" : "none");
  Line(8, "Picture: %s, phase %lu", sourceName(Latch.Src), (uint32_t)Phase);

  char States[16];
  pinStates(M, EGA_RGB_GPIO, 6, States);
  Line(10, "EGA GPIO  0-5:  %s", States);
  pinStates(M, CGA_RGB_GPIO, 6, States);
  Line(11, "CGA GPIO  8-13: %s", States);
  pinStates(M, MDA_VI_GPIO, 2, States);
  Line(12, "MDA GPIO 26-27: %s", States);
  uint32_t Ok = FramesOk, Failed = FramesFailed;
  FramesOk = 0;
  FramesFailed = 0;
  static const char *FailNames[] = {"-", "no VSync", "VSync end", "line"};
  if (Failed)
    Line(13, "Frames: %lu ok %lu bad (%s %lu)", Ok, Failed,
         FailNames[(uint32_t)LastFail], LastFailLine);
  else
    Line(13, "Frames: %lu ok", Ok);
  if (WatchdogReset)
    Line(14, "Hang reset: core0 %lu core1 %lu", ResetStage[0], ResetStage[1]);
  Line(15, "PIXEL CLOCK: hide text");
  Line(16, "AUTO ADJUST: sampling phase");

  for (uint32_t L = 0; L != NumTxtLines; ++L) {
    uint32_t Len = std::strlen(O.Lines[L]);
    if (Len > O.Width)
      O.Width = Len;
  }
  Shown = !Shown;

  stage(0, StagePrint);
  for (uint32_t L = 2; L != 15; ++L)
    if (O.Lines[L][0])
      std::printf("%s\n", O.Lines[L]);
  std::printf("\n");
}

int main() {
  if (watchdog_caused_reboot()) {
    WatchdogReset = true;
    ResetStage[0] = watchdog_hw->scratch[0];
    ResetStage[1] = watchdog_hw->scratch[1];
  }
  stage(0, StageBoot);
  stage(1, StageBoot);
  set_sys_clock_khz(SysClkKHz, true);
  stdio_init_all();
  buildFrame();
  startVGA();
  initInputs();
  initFastSample();
  multicore_launch_core1_with_stack(core1Main, Core1Stack, sizeof(Core1Stack));

#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  for (uint32_t Btn : {PIXEL_CLOCK_BTN_GPIO, AUTO_ADJUST_BTN_GPIO}) {
    gpio_init(Btn);
    gpio_set_dir(Btn, GPIO_IN);
    gpio_pull_up(Btn);
    gpio_set_irq_enabled(Btn, GPIO_IRQ_EDGE_FALL, true);
  }
  // Resets the Pico if core 0 hangs for 4 s, so no BOOTSEL press is needed.
  watchdog_enable(4000, /*pause_on_debug=*/true);
  while (true) {
    stage(0, StageMeasure);
    watchdog_update();
    Measurement M = measure(1000);
    watchdog_update();
    update(M);
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
  }
}
