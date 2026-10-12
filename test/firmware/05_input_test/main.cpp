//-*- C++ -*-
//
// Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
// License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
// Free for noncommercial use; commercial use only with the author's consent.
//
// Step 5: input test for the MCE Blaster. Core 0 measures the TTL input once
// per second as in step 4. Core 1 captures the input picture frame by frame
// and shows it in the background of a 640x480 VGA screen, with the
// measurement as a centered text block on top.
//

#include "Font8x8.h"
#include "InputCapture.pio.h"
#include "VGAOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
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
static constexpr uint32_t MDA_VI_GPIO = 26;  // 26 I, 27 V (DE9 pins 6, 7)
static constexpr uint32_t PIXEL_CLOCK_BTN_GPIO = 22;

static constexpr uint8_t Black = 0b000000;
static constexpr uint8_t Gray = 0b101010;
static constexpr uint8_t White = 0b111111;
static constexpr uint8_t Yellow = 0b111100;
static constexpr uint8_t HBit = 1u << 6;
static constexpr uint8_t VBit = 1u << 7;

// 640x480@60, HSync and VSync negative. Every byte lasts two pixels, so the
// PIO runs at 12.6 MHz (252 MHz / 20 per pixel pair) and all horizontal
// values are halved: the picture is 320x240, every line is sent twice.
static constexpr uint32_t SysClkKHz = 252000;
static constexpr uint32_t ClkDiv = 10;
static constexpr uint32_t HVisible = 320, HFrontPorch = 8, HSync = 48,
                          HBackPorch = 24;
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

static constexpr uint32_t NumTxtLines = 15;
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

/// Draws the overlay on top of a background row. The text has a black
/// outline instead of a box, so the input picture stays visible around it.
static void __not_in_flash_func(drawOverlayRow)(const Overlay &O, uint32_t Row,
                                                uint8_t *Out) {
  if (!OverlayOn || O.Width == 0)
    return;
  uint32_t BlockW = O.Width * FontWidth;
  uint32_t BlockH = NumTxtLines * TxtRowHeight;
  int32_t BlockX = (HVisible - BlockW) / 2;
  int32_t BlockY = (Rows - BlockH) / 2;
  int32_t Y = (int32_t)Row - BlockY;
  if (Y < -1 || Y > (int32_t)BlockH)
    return;
  // Pixels of this row and of the rows above and below, for the outline.
  auto RowBits = [&O](int32_t Y, uint32_t Col0) -> uint32_t {
    if (Y < 0)
      return 0;
    uint32_t L = Y / TxtRowHeight, FontRow = Y % TxtRowHeight;
    if (L >= NumTxtLines || FontRow >= FontHeight)
      return 0;
    // Bits Col0-1 .. Col0+8 of the line, bit 1 = column Col0.
    uint32_t Bits = 0;
    for (int32_t Col = (int32_t)Col0 - 1; Col <= (int32_t)Col0 + 8; ++Col) {
      if (Col < 0)
        continue;
      uint32_t Char = Col / FontWidth, Bit = Col % FontWidth;
      const char *Txt = O.Lines[L];
      if (Char >= std::strlen(Txt))
        continue;
      if (Font8x8[Txt[Char] & 0x7f][FontRow] & (1u << Bit))
        Bits |= 1u << (Col - (int32_t)Col0 + 1);
    }
    return Bits;
  };
  uint32_t L = Y >= 0 ? Y / TxtRowHeight : 0;
  uint8_t Color = (L == 0 ? Yellow : White) | Idle;
  for (uint32_t Col0 = 0; Col0 < BlockW; Col0 += 8) {
    uint32_t Mid = RowBits(Y, Col0);
    uint32_t Around = RowBits(Y - 1, Col0) | RowBits(Y + 1, Col0) | Mid |
                      Mid << 1 | Mid >> 1;
    for (uint32_t I = 0; I != 8; ++I) {
      int32_t X = BlockX + Col0 + I;
      if (X < 0 || X >= (int32_t)HVisible)
        continue;
      uint32_t Bit = 1u << (I + 1);
      if (Mid & Bit)
        Out[X] = Color;
      else if (Around & Bit)
        Out[X] = Black | Idle;
    }
  }
}

// ---------------------------------------------------------------------------
// Picture capture on core 1

enum class Source : uint32_t { None, EGA, CGA, MDA };

struct CaptureSetup {
  Source Src = Source::None;
  bool PosHSync = true;
  bool PosVSync = true;
  uint32_t LinesPerFrame = 0;
  float HHz = 0;
};
// Written by core 0, read by core 1. A torn read only costs one bad frame.
static volatile uint32_t SetupSeq = 0;
static CaptureSetup Setup;

static constexpr uint32_t SamplesPerLine = HVisible;
// Samples cover this share of a line, starting at the end of HSync.
static constexpr float SampledShare = 0.8f;

// Capture statistics, written by core 1, read and reset by core 0.
enum class CaptureFail : uint32_t { None, NoVSync, VSyncEnd, Line };
static volatile uint32_t FramesOk = 0;
static volatile uint32_t FramesFailed = 0;
static volatile CaptureFail LastFail = CaptureFail::None;
static volatile uint32_t LastFailLine = 0;

static uint8_t Captured[Rows][SamplesPerLine] __attribute__((aligned(4)));
static uint8_t DiscardLine[SamplesPerLine] __attribute__((aligned(4)));
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
      bool Intensity = I & 0b01, Video = I & 0b10;
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

static void startCapture(const CaptureSetup &S) {
  pio_sm_set_enabled(CapPio, CapSM, false);
  float SampleSec = SampledShare / S.HHz / SamplesPerLine;
  float Div = SysClkKHz * 1000.0f * SampleSec / 2;
  if (Div < 1)
    Div = 1;
  uint Offset = S.PosHSync ? PosOffset : NegOffset;
  pio_sm_config Conf = CaptureConfig(Offset, S.PosHSync, firstGPIO(S.Src), Div);
  pio_sm_init(CapPio, CapSM, Offset, &Conf);
  pio_sm_put(CapPio, CapSM, SamplesPerLine - 1);
}

static bool __not_in_flash_func(waitPin)(uint32_t GPIO, bool Level,
                                         uint64_t Deadline) {
  while (gpio_get(GPIO) != Level)
    if (time_us_64() > Deadline)
      return false;
  return true;
}

/// Captures one frame into Captured[]. \returns false without a signal.
static bool __not_in_flash_func(captureFrame)(const CaptureSetup &S) {
  uint64_t Deadline = time_us_64() + 100000;
  // Wait for the VSync pulse, restart the capture, start at its end.
  if (!waitPin(TTL_VSYNC_GPIO, !S.PosVSync, Deadline) ||
      !waitPin(TTL_VSYNC_GPIO, S.PosVSync, Deadline)) {
    LastFail = CaptureFail::NoVSync;
    return false;
  }
  pio_sm_set_enabled(CapPio, CapSM, false);
  pio_sm_clear_fifos(CapPio, CapSM);
  pio_sm_restart(CapPio, CapSM);
  pio_sm_exec(CapPio, CapSM,
              pio_encode_jmp(S.PosHSync ? PosOffset : NegOffset));
  pio_sm_put(CapPio, CapSM, SamplesPerLine - 1);
  if (!waitPin(TTL_VSYNC_GPIO, !S.PosVSync, Deadline)) {
    LastFail = CaptureFail::VSyncEnd;
    return false;
  }
  pio_sm_set_enabled(CapPio, CapSM, true);

  uint32_t LastRow = ~0u;
  for (uint32_t Line = 0; Line != S.LinesPerFrame; ++Line) {
    uint32_t Row = Line * Rows / S.LinesPerFrame;
    uint8_t *Dst = Row != LastRow ? Captured[Row] : DiscardLine;
    LastRow = Row;
    dma_channel_set_write_addr(CapChan, Dst, false);
    dma_channel_set_trans_count(CapChan, SamplesPerLine / 4, true);
    uint64_t LineDeadline = time_us_64() + 1000;
    while (dma_channel_is_busy(CapChan))
      if (time_us_64() > LineDeadline) {
        dma_channel_abort(CapChan);
        pio_sm_set_enabled(CapPio, CapSM, false);
        LastFail = CaptureFail::Line;
        LastFailLine = Line;
        return false;
      }
  }
  pio_sm_set_enabled(CapPio, CapSM, false);
  return true;
}

static void __not_in_flash_func(showFrame)(bool HavePicture) {
  const Overlay &O = Overlays[Shown];
  for (uint32_t R = 0; R != Rows; ++R) {
    uint8_t *Out = OutLines[R];
    if (HavePicture)
      for (uint32_t X = 0; X != HVisible; ++X)
        Out[X] = ColorLUT[Captured[R][X]];
    else
      std::memset(Out, Black | Idle, HVisible);
    drawOverlayRow(O, R, Out);
  }
}

static void core1Main() {
  CapSM = pio_claim_unused_sm(CapPio, true);
  PosOffset = pio_add_program(CapPio, &CapturePosHSync_program);
  NegOffset = pio_add_program(CapPio, &CaptureNegHSync_program);
  CapChan = dma_claim_unused_channel(true);
  dma_channel_config Conf = dma_channel_get_default_config(CapChan);
  channel_config_set_transfer_data_size(&Conf, DMA_SIZE_32);
  channel_config_set_read_increment(&Conf, false);
  channel_config_set_write_increment(&Conf, true);
  channel_config_set_dreq(&Conf, pio_get_dreq(CapPio, CapSM, /*is_tx=*/false));
  dma_channel_configure(CapChan, &Conf, Captured, &CapPio->rxf[CapSM],
                        SamplesPerLine / 4, /*trigger=*/false);

  uint32_t SeenSeq = ~0u;
  CaptureSetup S;
  while (true) {
    if (SeenSeq != SetupSeq) {
      SeenSeq = SetupSeq;
      __dmb();
      S = Setup;
      buildLUT(S.Src);
      if (S.Src != Source::None)
        startCapture(S);
    }
    bool HavePicture = S.Src != Source::None && captureFrame(S);
    if (S.Src != Source::None) {
      if (HavePicture)
        FramesOk = FramesOk + 1;
      else
        FramesFailed = FramesFailed + 1;
    }
    showFrame(HavePicture);
    if (!HavePicture)
      sleep_ms(20);
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
  } else if (GPIO == PIXEL_CLOCK_BTN_GPIO) {
    // Debounce: ignore presses within 200 ms of the last one.
    static uint64_t LastPress = 0;
    uint64_t Now = time_us_64();
    if (Now - LastPress > 200000)
      OverlayOn = !OverlayOn;
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
};
// MDA/CGA/EGA from the MCE Blaster's TimingsTTL.def. CGA and EGA 200 lines
// share one timing. The VGA modes are not supported by the MCE Blaster; they
// are listed because VGA cards with a 9-pin TTL output send them.
static constexpr TTLMode TTLModes[] = {
    {"MDA 720x350 50Hz", 18430, 50},
    {"CGA/EGA 640x200 60Hz", 15700, 60},
    {"EGA 640x350 60Hz", 21850, 60},
    {"VGA 720x400 70Hz", 31469, 70},
    {"VGA 640x480 60Hz", 31469, 60},
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

static void update(const Measurement &M) {
  const char *Mode = guessMode(M);
  Source Src = pickSource(M, Mode);

  // Restart the capture only when the input really changed.
  CaptureSetup New;
  New.Src = Src;
  New.PosHSync = M.H.positive();
  New.PosVSync = M.V.positive();
  New.HHz = M.H.Hz;
  New.LinesPerFrame = Mode ? (uint32_t)std::lround(M.H.Hz / M.V.Hz) : 0;
  bool Changed = New.Src != Setup.Src || New.PosHSync != Setup.PosHSync ||
                 New.PosVSync != Setup.PosVSync ||
                 New.LinesPerFrame != Setup.LinesPerFrame ||
                 std::fabs(New.HHz - Setup.HHz) > Setup.HHz * 0.002f;
  if (Changed) {
    Setup = New;
    __dmb();
    SetupSeq = SetupSeq + 1;
  }

  Overlay &O = Overlays[!Shown];
  O = Overlay();
  auto Line = [&O](uint32_t L, const char *Fmt, auto... Args) {
    std::snprintf(O.Lines[L], sizeof(O.Lines[L]), Fmt, Args...);
  };
  Line(0, "MCE BLASTER - INPUT TEST");
  Line(14, "PIXEL CLOCK button: hide text");
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
  if (Mode)
    Line(4, "Lines per frame: %.1f", M.H.Hz / M.V.Hz);
  Line(5, "Mode: %s", Mode ? Mode : "NO SIGNAL");
  Line(6, "Picture: %s", sourceName(Src));
  uint32_t Ok = FramesOk, Failed = FramesFailed;
  FramesOk = 0;
  FramesFailed = 0;
  static const char *FailNames[] = {"-", "no VSync", "VSync end", "line"};
  if (Failed)
    Line(13, "Frames: %lu ok %lu bad (%s %lu)", Ok, Failed,
         FailNames[(uint32_t)LastFail], LastFailLine);
  else
    Line(13, "Frames: %lu ok", Ok);

  char States[16];
  Line(8, "Pins: * toggles + high - low");
  pinStates(M, EGA_RGB_GPIO, 6, States);
  Line(9, "EGA GPIO  0-5:  %s", States);
  pinStates(M, CGA_RGB_GPIO, 6, States);
  Line(10, "CGA GPIO  8-13: %s", States);
  pinStates(M, MDA_VI_GPIO, 2, States);
  Line(11, "MDA GPIO 26-27: %s", States);
  pinStates(M, TTL_VSYNC_GPIO, 2, States);
  Line(12, "V/H GPIO  6-7:  %s", States);

  for (uint32_t L = 0; L != NumTxtLines; ++L) {
    uint32_t Len = std::strlen(O.Lines[L]);
    if (Len > O.Width)
      O.Width = Len;
  }
  Shown = !Shown;

  for (uint32_t L = 2; L != NumTxtLines; ++L)
    if (O.Lines[L][0])
      std::printf("%s\n", O.Lines[L]);
  std::printf("\n");
}

int main() {
  set_sys_clock_khz(SysClkKHz, true);
  stdio_init_all();
  buildFrame();
  startVGA();
  initInputs();
  multicore_launch_core1(core1Main);

#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  gpio_init(PIXEL_CLOCK_BTN_GPIO);
  gpio_set_dir(PIXEL_CLOCK_BTN_GPIO, GPIO_IN);
  gpio_pull_up(PIXEL_CLOCK_BTN_GPIO);
  gpio_set_irq_enabled(PIXEL_CLOCK_BTN_GPIO, GPIO_IRQ_EDGE_FALL, true);
  while (true) {
    update(measure(1000));
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
  }
}
