//-*- C++ -*-
//
// Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
// License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
// Free for noncommercial use; commercial use only with the author's consent.
//
// Step 9: frame buffer. As step 7, but every input pixel and every input
// line is stored once, 16 colours, in a frame buffer, and shown 1:1 in the
// middle of a 640x480 VGA screen; 200-line modes are doubled to 400. Input
// and output run at their own rates.
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
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "tusb.h"
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
static constexpr uint32_t MDA_VI_GPIO = 26;  // 26 I, 27 V (DE9 pins 6, 7)
static constexpr uint32_t PIXEL_CLOCK_BTN_GPIO = 22;
static constexpr uint32_t AUTO_ADJUST_BTN_GPIO = 28;

static constexpr uint8_t Black = 0b000000;
static constexpr uint8_t Gray = 0b101010;
static constexpr uint8_t White = 0b111111;
static constexpr uint8_t Yellow = 0b111100;
static constexpr uint8_t HBit = 1u << 6;
static constexpr uint8_t VBit = 1u << 7;

// 640x480@60 timing, but 720 pixels per line at 28 MHz (252 MHz / 9), so
// a 720-pixel input line fits 1:1. Line 31.53 kHz, frame 60.06 Hz, HSync and
// VSync negative.
static constexpr uint32_t SysClkKHz = 252000;
static constexpr uint32_t ClkDiv = 9;
static constexpr uint32_t HVisible = 720, HFrontPorch = 16, HSync = 104,
                          HBackPorch = 48;
static constexpr uint32_t VVisible = 480, VFrontPorch = 10, VSync = 2,
                          VBackPorch = 33;
static constexpr uint32_t HTotal = HVisible + HFrontPorch + HSync + HBackPorch;
static constexpr uint32_t VTotal = VVisible + VFrontPorch + VSync + VBackPorch;
// The text is laid out in rows of two screen lines.
static constexpr uint32_t Rows = VVisible / 2;
static_assert(HTotal % 4 == 0, "DMA sends 4 bytes per transfer");

// Visible pixels carry the idle sync levels.
static constexpr uint8_t Idle = HBit | VBit;

// The frame buffer: one nibble per input pixel, a colour index. Core 1
// writes it at the input's rate, the output reads it at 60 Hz.
static constexpr uint32_t FbWidth = HVisible, FbLines = VVisible;
static uint8_t Fb[FbLines][FbWidth / 2] __attribute__((aligned(4)));
// Where the picture is: its size in the frame buffer, and where it goes on
// the screen. Written by core 1, read by the output.
static volatile uint32_t PicWidth = 0, PicLines = 0;
static volatile uint32_t OutLeft = 0, OutTop = 0, OutShift = 0;
// Two frame buffer pixels to two output bytes.
static uint16_t PairLUT[256];

static uint8_t BlankLine[HTotal] __attribute__((aligned(4)));
static uint8_t VSyncLine[HTotal] __attribute__((aligned(4)));
// Visible lines rotate through four buffers. The line IRQ fills the buffer
// for three lines ahead, so it may come two lines late without the output
// catching up with it.
static constexpr uint32_t NumWork = 4, FillAhead = 3;
static uint8_t Work[NumWork][HTotal] __attribute__((aligned(4)));
// Lines the output started sending before the IRQ had finished them, the
// first and last such line, and the longest IRQ in microseconds.
static volatile uint32_t LateLines = 0, LateFirst = 0, LateLast = 0;
static volatile uint32_t IrqMaxUs = 0;
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
  for (uint32_t I = 0; I != NumWork; ++I)
    initLine(Work[I], false);
  for (uint32_t Y = 0; Y != VTotal; ++Y) {
    uint32_t SyncStart = VVisible + VFrontPorch;
    if (Y < VVisible)
      Frame[Y] = Work[Y % NumWork];
    else if (Y >= SyncStart && Y < SyncStart + VSync)
      Frame[Y] = VSyncLine;
    else
      Frame[Y] = BlankLine;
  }
}

static void applyMasks(uint32_t Row, uint8_t *Out);

/// Fills the visible part of screen line \p Y from the frame buffer. Runs in
/// an IRQ, so it must not turn its loops into memset calls, which run from
/// flash.
__attribute__((optimize("no-tree-loop-distribute-patterns")))
static void __not_in_flash_func(fillLine)(uint32_t Y, uint8_t *Out) {
  constexpr uint32_t BlackWord = (Black | Idle) * 0x01010101u;
  uint32_t *Words = reinterpret_cast<uint32_t *>(Out);
  uint32_t Rel = Y - OutTop;
  uint32_t Width = PicWidth;
  if (Y < OutTop || Rel >= (PicLines << OutShift) || !Width) {
    for (uint32_t W = 0; W != HVisible / 4; ++W)
      Words[W] = BlackWord;
  } else {
    uint32_t Left = OutLeft;
    for (uint32_t X = 0; X < Left; X += 2)
      reinterpret_cast<uint16_t *>(Out)[X / 2] = (uint16_t)BlackWord;
    // Two frame buffer bytes, four pixels, per 32-bit store. Left and Width
    // are multiples of 4 and 2.
    const uint8_t *Src = Fb[Rel >> OutShift];
    uint32_t *Dst = reinterpret_cast<uint32_t *>(Out + Left);
    for (uint32_t I = 0; I != Width / 4; ++I)
      Dst[I] = PairLUT[Src[2 * I]] | (uint32_t)PairLUT[Src[2 * I + 1]] << 16;
    for (uint32_t X = Left + Width; X < HVisible; X += 2)
      reinterpret_cast<uint16_t *>(Out)[X / 2] = (uint16_t)BlackWord;
  }
  applyMasks(Y / 2, Out);
}

// The control channel feeds the next line address to the data channel. Its
// IRQ fires once per line and converts the next visible line from the frame
// buffer, with a whole scanline of time to do so.
static void __not_in_flash_func(ctrlChanIRQ)() {
  dma_hw->ints0 = 1u << CtrlChan;
  // The data channel has just started on line Next - 1; fill the buffer for
  // line Next + FillAhead - 1.
  uint32_t Next = (const uint8_t **)dma_hw->ch[CtrlChan].read_addr - Frame;
  if (Next == VTotal) {
    dma_channel_set_read_addr(CtrlChan, &Frame[0], /*trigger=*/false);
    Next = 0;
  }
  uint32_t Y = Next + FillAhead - 1;
  if (Y >= VTotal)
    Y -= VTotal;
  if (Y < VVisible) {
    uint32_t Start = timer_hw->timerawl;
    uint8_t *Buf = Work[Y % NumWork];
    fillLine(Y, Buf);
    uintptr_t Sending = dma_hw->ch[DataChan].read_addr;
    if (Sending > (uintptr_t)Buf && Sending <= (uintptr_t)(Buf + HTotal)) {
      if (!LateLines)
        LateFirst = Y;
      LateLast = Y;
      LateLines = LateLines + 1;
    }
    uint32_t Took = timer_hw->timerawl - Start;
    if (Took > IrqMaxUs)
      IrqMaxUs = Took;
  }
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
  irq_set_priority(DMA_IRQ_0, 0);
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
// black outline, one bit per font pixel), so the output can put it on top of
// each line within the line time. Two sets: core 1 builds the hidden one
// while the output reads the shown one, so the text never flickers.
static constexpr uint32_t BlockW = MaxChars * FontWidth + 2; // font pixels
static constexpr uint32_t MaskBytes = (BlockW + 7) / 8;
static constexpr uint32_t BlockH = NumTxtLines * TxtRowHeight + 2;
struct Masks {
  uint8_t Text[BlockH][MaskBytes];
  uint8_t Edge[BlockH][MaskBytes];
  uint32_t X = 0; // screen position of the block
};
static Masks MaskSets[2];
static volatile uint32_t ShownMasks = 0;
static constexpr uint32_t MaskY = (Rows - BlockH) / 2;

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
  Masks &M = MaskSets[!ShownMasks];
  std::memset(M.Text, 0, sizeof(M.Text));
  std::memset(M.Edge, 0, sizeof(M.Edge));
  uint32_t Width = O.Width * FontWidth + 2; // used width in font pixels
  // Two screen pixels per font pixel horizontally, rows are shown twice.
  M.X = (HVisible - 2 * Width) / 2;
  for (int32_t Y = 0; Y != (int32_t)BlockH; ++Y)
    for (int32_t X = 0; X != (int32_t)Width; ++X) {
      // Mask coordinates have a one pixel margin for the outline.
      bool Text = textPixel(O, Y - 1, X - 1);
      bool Edge = false;
      for (int32_t DY = -1; DY <= 1 && !Text && !Edge; ++DY)
        for (int32_t DX = -1; DX <= 1 && !Edge; ++DX)
          Edge = textPixel(O, Y - 1 + DY, X - 1 + DX);
      if (Text)
        M.Text[Y][X / 8] |= 1u << (X % 8);
      else if (Edge)
        M.Edge[Y][X / 8] |= 1u << (X % 8);
    }
  __dmb();
  ShownMasks = !ShownMasks;
}

/// Puts the text on top of output row \p Row.
static void __not_in_flash_func(applyMasks)(uint32_t Row, uint8_t *Out) {
  if (!OverlayOn || Row < MaskY || Row >= MaskY + BlockH)
    return;
  const Masks &M = MaskSets[ShownMasks];
  uint32_t Y = Row - MaskY;
  // The title line is yellow.
  uint8_t Color = ((Y < TxtRowHeight + 1) ? Yellow : White) | Idle;
  // Two screen pixels per font pixel: one 16-bit store each. M.X is even.
  const uint16_t Glyph = Color * 0x0101u, Outline = (Black | Idle) * 0x0101u;
  const uint8_t *T = M.Text[Y], *E = M.Edge[Y];
  uint16_t *P = reinterpret_cast<uint16_t *>(Out + M.X);
  for (uint32_t B = 0; B != MaskBytes; ++B, P += 8) {
    uint32_t Tb = T[B], Any = Tb | E[B];
    if (!Any)
      continue;
    for (uint32_t I = 0; I != 8; ++I, Tb >>= 1, Any >>= 1)
      if (Any & 1)
        P[I] = (Tb & 1) ? Glyph : Outline;
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
  uint8_t ColorMask = 0x3f; // sample bits that carry colour
  // Per line: skip SkipPx pixels after HSync, then take Samples pixels.
  uint32_t SkipPx = 0, Samples = 0;
  // The picture as the mode table expects it, in samples and in lines after
  // VSync; zero width or height when not known.
  uint32_t Left = 0, Width = 0, Top = 0, Height = 0;
};
// Written by core 0, read by core 1. A torn read only costs one bad frame.
static volatile uint32_t SetupSeq = 0;
static CaptureSetup Setup;
// AUTO ADJUST shifts the sampling point by half an input pixel.
static volatile uint32_t Phase = 0;

// Pixels sampled before and after the expected picture, so the picture
// window can grow when a card starts its picture earlier or ends it later.
static constexpr uint32_t MarginPx = 32;
static constexpr uint32_t MaxSamples = 768;

// The picture window on core 1: starts at the mode table's values; for an
// unknown mode it grows to every non-black sample and line seen.
// Detection runs every DetectEvery frames; those frames are not stored.
static constexpr uint32_t DetectEvery = 8;
static constexpr uint32_t MaxEarlyPx = 18;
static volatile uint32_t WinLeft = 0, WinRight = 0, WinTop = 0, WinBottom = 0;

/// Places the window's picture on the screen: 1:1, doubled when it has at
/// most half the screen lines, clipped to the frame buffer.
static void placePicture() {
  uint32_t Width = WinRight - WinLeft, Lines = WinBottom - WinTop;
  Width = (Width < FbWidth ? Width : FbWidth) & ~3u;
  Lines = Lines < FbLines ? Lines : FbLines;
  uint32_t Shift = Lines <= FbLines / 2 ? 1 : 0;
  PicWidth = 0; // the output shows black while the values change
  __dmb();
  OutShift = Shift;
  OutTop = (VVisible - (Lines << Shift)) / 2;
  OutLeft = ((HVisible - Width) / 2) & ~3u;
  PicLines = Lines;
  __dmb();
  PicWidth = Width;
}

static void resetWindow(const CaptureSetup &S) {
  bool Known = S.Width && S.Height;
  WinLeft = Known ? S.Left : 0;
  WinRight = Known ? S.Left + S.Width : S.Samples;
  WinTop = Known ? S.Top : 0;
  WinBottom = Known ? S.Top + S.Height : S.LinesPerFrame;
  placePicture();
}

enum class CaptureFail : uint32_t { None, NoVSync, VSyncEnd, Line, Late };
static volatile uint32_t FramesOk = 0;
static volatile uint32_t FramesFailed = 0;
static volatile CaptureFail LastFail = CaptureFail::None;
static volatile uint32_t LastFailLine = 0;
// Most lines core 1 fell behind the capture since the last overlay update.
// In capture words; the line loop does no division, which would run from
// flash.
static volatile uint32_t MaxLag = 0;
// Frames in which the capture SM had to wait for the DMA, which stretches
// the sampling of a line.
static volatile uint32_t StalledFrames = 0;

// One DMA channel writes the whole frame into a ring buffer. The DMA write
// ring wraps the address in hardware, so a capture that runs ahead of core 1
// can only overwrite old samples, never other memory.
static constexpr uint32_t RingBits = 13; // 8 KB, ten lines
static constexpr uint32_t RingBytes = 1u << RingBits;
static uint8_t Ring[RingBytes] __attribute__((aligned(RingBytes)));
// Sample to colour index.
static uint8_t ColorLUT[256];

static PIO CapPio = pio1;
static uint CapSM;
static int CapChan;
static uint PosOffset, NegOffset;

/// One TTL colour component on the 2-bit VGA DAC.
static uint8_t level(bool On, bool Intense) {
  return (On ? 2 : 0) | (Intense ? 1 : 0);
}

/// Colour indices: bits 0-2 blue, green, red, bit 3 intensity. EGA's 64
/// colours are folded onto these 16: a secondary colour line counts as
/// intensity. MDA uses 0 black, 1 normal, 3 bright.
static void buildLUT(Source Src) {
  uint8_t Palette[16];
  for (uint32_t I = 0; I != 16; ++I) {
    bool B = I & 1, G = I & 2, R = I & 4, In = I & 8;
    // Colour 6 without intensity is brown, as on a CGA monitor.
    bool Brown = R && G && !B && !In;
    Palette[I] = level(R, In) << 4 | (Brown ? 1 : level(G, In)) << 2 |
                 level(B, In);
  }
  if (Src == Source::MDA) {
    Palette[0] = Palette[2] = Black;
    Palette[1] = Gray;
    Palette[3] = White;
  }
  for (uint32_t I = 0; I != 256; ++I) {
    uint8_t C = 0;
    switch (Src) {
    case Source::EGA:
      // GPIO 0-5: secondary blue, blue, secondary green, green, secondary
      // red, red.
      C = (I >> 1 & 1) | (I >> 3 & 1) << 1 | (I >> 5 & 1) << 2 |
          ((I & 0b010101) ? 8 : 0);
      break;
    case Source::CGA:
      // GPIO 8-13: intensity, blue, intensity, green, intensity, red.
      C = (I >> 1 & 1) | (I >> 3 & 1) << 1 | (I >> 5 & 1) << 2 |
          ((I & 1) ? 8 : 0);
      break;
    case Source::MDA: {
      bool Intensity = I & 0b01, Video = I & 0b10;
      C = Video ? (Intensity ? 3 : 1) : 0;
      break;
    }
    case Source::None:
      break;
    }
    ColorLUT[I] = C;
    PairLUT[I] = (uint16_t)((Palette[I & 15] | Idle) |
                            (Palette[I >> 4] | Idle) << 8);
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
static bool WindowFound = false;

/// Captures one frame straight into the output lines, or with \p Detect only
/// looks for the picture edges. \returns false without a signal.
static bool __no_inline_not_in_flash_func(captureFrame)(const CaptureSetup &S,
                                              bool Detect) {
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
  // Captured words beyond the start of a line at which the capture starts
  // to overwrite it.
  uint32_t RingWords = RingBytes / S.Samples * WordsPerLine;
  uint32_t Words = S.LinesPerFrame * WordsPerLine;
  dma_channel_set_write_addr(CapChan, Ring, false);
  dma_channel_set_trans_count(CapChan, Words, false);
  if (!waitPin(TTL_VSYNC_GPIO, !S.PosVSync, Deadline)) {
    LastFail = CaptureFail::VSyncEnd;
    return false;
  }
  stage(1, StageCapLines);
  dma_channel_start(CapChan);
  CapPio->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + CapSM);
  pio_sm_set_enabled(CapPio, CapSM, true);

  const bool Known = S.Width && S.Height;
  uint32_t FoundLeft = ~0u, FoundRight = 0, FoundTop = ~0u, FoundBottom = 0;
  constexpr uint32_t MaxLeftPx = 64, MinEarlyLines = 4;
  uint8_t EarlyLines[MaxLeftPx] = {};
  const uint32_t Mask = RingBytes - 1;
  const uint32_t Top = WinTop, Left = WinLeft;
  const uint32_t Width = PicWidth, Lines = PicLines;
  for (uint32_t Line = 0; Line != S.LinesPerFrame; ++Line) {
    uint32_t Needed = (Line + 1) * WordsPerLine;
    // The raw timer register: time_us_64() runs from flash.
    uint32_t LineStart = timer_hw->timerawl;
    while (Words - dma_channel_hw_addr(CapChan)->transfer_count < Needed)
      if (timer_hw->timerawl - LineStart > 1000) {
        stopCapture();
        LastFail = CaptureFail::Line;
        LastFailLine = Line;
        return false;
      }
    uint32_t Start = Line * S.Samples;

    if (Detect) {
      // A known mode only looks left of its window, which is fast enough
      // for 31.5 kHz lines; a full scan of empty lines falls behind there.
      uint32_t L = 0, R = Known ? Left : S.Samples;
      while (L != R && !(Ring[(Start + L) & Mask] & S.ColorMask))
        ++L;
      if (Known)
        R = L == R ? L : L + 1;
      while (R != L && !(Ring[(Start + R - 1) & Mask] & S.ColorMask))
        --R;
      if (Known) {
        // Only picture lines count, and a column only when several lines
        // start there: a single stray sample moved the window a character
        // too far.
        if (L != R && L < MaxLeftPx && Line >= Top && Line - Top < Lines &&
            EarlyLines[L] != 255)
          ++EarlyLines[L];
      } else if (L != R) {
        FoundLeft = L < FoundLeft ? L : FoundLeft;
        FoundRight = R > FoundRight ? R : FoundRight;
        FoundTop = Line < FoundTop ? Line : FoundTop;
        FoundBottom = Line + 1;
      }
      // A line read after the capture overwrote it would widen the window
      // with garbage: drop the whole detection frame.
      uint32_t Behind = Words - dma_channel_hw_addr(CapChan)->transfer_count -
                        Line * WordsPerLine;
      if (Behind >= RingWords) {
        stopCapture();
        LastFail = CaptureFail::Late;
        LastFailLine = Line;
        return false;
      }
      continue;
    }

    if (Line < Top || Line - Top >= Lines)
      continue;
    // Two samples per frame buffer byte. Read the line through a plain
    // pointer; only a line that wraps around the ring end needs the mask on
    // every sample.
    uint8_t *Out = Fb[Line - Top];
    uint32_t First = Start + Left;
    if ((First & Mask) + Width > RingBytes) {
      for (uint32_t X = 0; X != Width; X += 2)
        Out[X / 2] = ColorLUT[Ring[(First + X) & Mask]] |
                     ColorLUT[Ring[(First + X + 1) & Mask]] << 4;
    } else {
      const uint8_t *Src = Ring + (First & Mask);
      for (uint32_t X = 0; X != Width; X += 4) {
        Out[X / 2] = ColorLUT[Src[X]] | ColorLUT[Src[X + 1]] << 4;
        Out[X / 2 + 1] = ColorLUT[Src[X + 2]] | ColorLUT[Src[X + 3]] << 4;
      }
    }
    // The capture overwrites a line RingLines lines later. If it got there
    // while this line was read, the line would come out shifted: stop and
    // keep the rest of the last frame.
    uint32_t Behind = Words - dma_channel_hw_addr(CapChan)->transfer_count -
                      Line * WordsPerLine;
    if (Behind > MaxLag)
      MaxLag = Behind;
    if (Behind >= RingWords) {
      stopCapture();
      LastFail = CaptureFail::Late;
      LastFailLine = Line;
      return false;
    }
  }
  if (Detect && Known)
    for (uint32_t X = 0; X != MaxLeftPx && FoundTop == ~0u; ++X)
      if (EarlyLines[X] >= MinEarlyLines) {
        FoundLeft = X;
        FoundTop = 0;
      }
  // A known mode keeps the table's width and lines, but some cards start
  // their picture a character earlier than the standard timing (a Trident
  // TVGA on 720x400 by 9 pixels): the window moves left to the first
  // picture pixel, within MaxEarlyPx.
  if (Detect && Known && FoundTop != ~0u && FoundLeft < WinLeft &&
      FoundLeft + MaxEarlyPx >= S.Left) {
    // Whole characters, 9 pixels wide in 720-pixel modes, else 8: the first
    // lit pixel is often a glyph's second or third column.
    uint32_t CharPx = S.Width == 720 ? 9 : 8;
    uint32_t Shift = (S.Left - FoundLeft + CharPx - 1) / CharPx * CharPx;
    if (Shift <= S.Left) {
      WinLeft = S.Left - Shift;
      WinRight = WinLeft + S.Width;
      placePicture();
    }
  }
  if (Detect && !Known && FoundTop != ~0u) {
    bool First = !WindowFound;
    WindowFound = true;
    WinLeft = First || FoundLeft < WinLeft ? FoundLeft : WinLeft;
    WinRight = First || FoundRight > WinRight ? FoundRight : WinRight;
    WinTop = First || FoundTop < WinTop ? FoundTop : WinTop;
    WinBottom = First || FoundBottom > WinBottom ? FoundBottom : WinBottom;
    placePicture();
  }
  if (CapPio->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + CapSM)))
    StalledFrames = StalledFrames + 1;
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
      WindowFound = false;
      resetWindow(S);
      if (S.Src != Source::None)
        configCapture(S);
    }
    static uint32_t FrameCount = 0;
    // Every DetectEvery-th frame looks for the picture's edges instead of
    // being stored; a known mode only lets its window start earlier.
    bool Detect = ++FrameCount % DetectEvery == 0;
    bool HavePicture = S.Src != Source::None && captureFrame(S, Detect);
    if (S.Src != Source::None) {
      if (HavePicture)
        FramesOk = FramesOk + 1;
      else
        FramesFailed = FramesFailed + 1;
    }
    // A late frame keeps the last picture; only a lost signal blanks it.
    if (!HavePicture && LastFail != CaptureFail::Late) {
      stage(1, StageCapIdle);
      std::memset(Fb, 0, sizeof(Fb));
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
  // Pixels from the end of HSync to the picture and picture width, lines
  // from the end of VSync to the picture and picture height; 0 when not
  // known, then the window comes from the picture alone.
  uint32_t BackPorchPx, VisiblePx, VBackPorch, VVisible;
};
// MDA/CGA/EGA from the MCE Blaster's TimingsTTL.def. CGA and EGA 200 lines
// share one timing. The VGA modes are not supported by the MCE Blaster; they
// are listed because VGA cards with a 9-pin TTL output send them.
static constexpr TTLMode TTLModes[] = {
    // Porches from the standard CRTC values (MDA, CGA) and VESA timings.
    {"MDA 720x350 50Hz", 18430, 50, 16257000, 9, 720, 4, 350},
    {"CGA/EGA 640x200 60Hz", 15700, 60, 14318181.0f, 112, 640, 22, 200},
    {"EGA 640x350 60Hz", 21850, 60, 16257000, 0, 0, 0, 0},
    {"VGA 720x400 70Hz", 31469, 70, 28322000, 54, 720, 35, 400},
    {"VGA 640x480 60Hz", 31469, 60, 25175000, 48, 640, 33, 480},
    // A Trident TVGA8800 in EGA emulation: VGA's 720x400 at 28.322 MHz with
    // 873 instead of 900 pixels per line. Where the 27 pixels are taken from
    // is not known. Its 4.13 us HSync leaves 748 samples after the pulse,
    // so the picture can start 28 pixels in at most; the window may still
    // move left by itself.
    {"VGA 720x400 72Hz TVGA", 32442, 72.25f, 28322000, 28, 720, 35, 400},
    // The TVGA8800 in MDA emulation: MDA's CRTC geometry (882 pixels, 135
    // of them HSync) at the VGA clock of 28.322 MHz instead of 16.257 MHz.
    // The card repeats the first character of each row in the 9 pixels
    // before it, so the window starts one character later than MDA's.
    {"MDA 720x350 86Hz TVGA", 32110, 86.3f, 28322000, 18, 720, 4, 350},
    // Measured on a Trident TVGA 9-pin output in its non-VGA switch setting;
    // the pixel clock is unknown, so it is measured only.
    {"TVGA 200 lines 117Hz", 31055, 117.6f, 0, 0, 0, 0, 0},
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
  // An MDA card's video and intensity also reach the CGA/EGA inputs (DE9
  // pins 7 and 6 are EGA's secondary blue and green), which shows MDA as a
  // blue picture. In MDA timing the MDA pins win.
  if (std::strncmp(Mode, "MDA", 3) == 0 && M.toggles(MDA_VI_GPIO, 2))
    return Source::MDA;
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

static constexpr float MaxModeDeviation = 0.05f;

static CaptureClock captureClock(const char *Mode, float HHz,
                                 const PixelClock &Measured) {
  CaptureClock C;
  float ModeHz = 0;
  for (const TTLMode &M : TTLModes)
    if (M.Name == Mode)
      ModeHz = M.PixelHz;
  // Thin text has few edges, and the measurement can then be off by 10 %
  // (31.2 instead of 28.3 MHz on a 720x400 input). Cards differ from the
  // mode's usual clock by a few percent at most, so a measurement further
  // off is dropped for the mode's clock.
  PixelClock Kept = Measured;
  if (ModeHz > 0 && std::fabs(Kept.Hz - ModeHz) / ModeHz > MaxModeDeviation)
    Kept.Hz = 0;
  // VGA timings come with their standard clock: a sparse screen measured
  // 28.825 MHz on 720x400, which snapped to NTSC's 28.636 instead of 28.322.
  if (Mode && std::strncmp(Mode, "VGA", 3) == 0 && ModeHz > 0)
    Kept.Hz = 0;
  const PixelClock &Use = Kept;
  // The mode's usual clock wins whenever the measurement is near it: the
  // measurement scatters by a few tenths of a percent, and 28.322 and
  // 28.636 MHz are only 1.1 % apart, so the nearest known clock alone
  // picked the wrong one on a 720x400 input.
  if (Use.Hz > 0 && ModeHz > 0 &&
      std::fabs(Use.Hz - ModeHz) / ModeHz < SnapTolerance) {
    C.Hz = ModeHz;
    for (const KnownClock &K : KnownClocks)
      if (K.Hz == ModeHz)
        C.Name = K.Name;
    C.PixelsPerLine = (uint32_t)std::lround(C.Hz / HHz);
    C.Hz = C.PixelsPerLine * HHz;
    return C;
  }
  float Best = SnapTolerance;
  for (const KnownClock &K : KnownClocks) {
    float Off = std::fabs(Use.Hz - K.Hz) / K.Hz;
    if (Use.Hz > 0 && Off < Best) {
      Best = Off;
      C.Hz = K.Hz;
      C.Name = K.Name;
    }
  }
  if (Use.Hz > 0 && !C.Name) {
    C.Hz = Use.Hz;
  } else if (Use.Hz == 0) {
    C.Hz = ModeHz;
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
  float HSyncUs = 0;
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
  Latch.HSyncUs = M.H.pulseUs();
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
  New.ColorMask = Latch.Src == Source::MDA ? 0x3 : 0x3f;
  // Sample from the end of HSync to shortly before the next one.
  uint32_t SyncPx = (uint32_t)(Latch.HSyncUs * Clock.Hz / 1e6f);
  uint32_t Free = Clock.PixelsPerLine > SyncPx + 8
                      ? Clock.PixelsPerLine - SyncPx - 8
                      : 0;
  if (Mode && Mode->VisiblePx) {
    New.SkipPx = Mode->BackPorchPx > MarginPx ? Mode->BackPorchPx - MarginPx : 0;
    New.Left = Mode->BackPorchPx - New.SkipPx;
    New.Width = Mode->VisiblePx;
    New.Top = Mode->VBackPorch;
    New.Height = Mode->VVisible;
  }
  uint32_t Samples = Free > New.SkipPx ? Free - New.SkipPx : 0;
  if (Samples > MaxSamples)
    Samples = MaxSamples;
  New.Samples = Samples / 4 * 4;
  // A known mode keeps its window even when a slightly long HSync pulse
  // leaves fewer samples: content-based detection on a nearly empty screen
  // finds only a few characters and shows them as a narrow strip.
  if (New.Width && New.Left + New.Width > New.Samples)
    New.Width = New.Samples > New.Left + 64 ? New.Samples - New.Left : 0;
  if (New.Height && New.Top + New.Height > New.LinesPerFrame)
    New.Height =
        New.LinesPerFrame > New.Top + 64 ? New.LinesPerFrame - New.Top : 0;
  if (!New.Width || !New.Height)
    New.Width = New.Height = 0;
  if (New.Samples < 64)
    New.Src = Source::None;
  bool Changed = New.Src != Setup.Src || New.PosHSync != Setup.PosHSync ||
                 New.PosVSync != Setup.PosVSync ||
                 New.LinesPerFrame != Setup.LinesPerFrame ||
                 New.PixelHz != Setup.PixelHz || New.SkipPx != Setup.SkipPx ||
                 New.Samples != Setup.Samples || New.Left != Setup.Left ||
                 New.Width != Setup.Width || New.Top != Setup.Top ||
                 New.Height != Setup.Height;
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
  // The build time tells which firmware is on the Pico.
  Line(0, "MCE BLASTER FRAME BUFFER  %.5s", __TIME__);
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
  if (LastClock.Hz > 0 && Clock.FromMode)
    Line(7, "Measured %.3f MHz, ignored", LastClock.Hz / 1e6f);
  else if (LastClock.Hz > 0)
    Line(7, "Measured %.3f MHz%s%s", LastClock.Hz / 1e6f,
         Clock.Name ? ", snap " : "", Clock.Name ? Clock.Name : "");
  else if (Latch.Valid)
    Line(7, "Measured: no detail, %s", Clock.FromMode ? "mode clock" : "none");
  Line(8, "Picture: %s, phase %lu", sourceName(Latch.Src), (uint32_t)Phase);
  if (New.Src != Source::None)
    Line(9, "Window: x %lu-%lu  y %lu-%lu", New.SkipPx + WinLeft,
         New.SkipPx + WinRight, (uint32_t)WinTop, (uint32_t)WinBottom);

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
  static const char *FailNames[] = {"-", "no VSync", "VSync end", "line",
                                    "late"};
  uint32_t Lag = MaxLag, Stalled = StalledFrames, OutLate = LateLines;
  uint32_t OutFirst = LateFirst, OutLast = LateLast, OutMaxUs = IrqMaxUs;
  LateLines = 0;
  IrqMaxUs = 0;
  MaxLag = 0;
  Lag /= Setup.Samples >= 4 ? Setup.Samples / 4 : 1;
  StalledFrames = 0;
  if (Failed)
    Line(13, "Frames: %lu ok %lu bad (%s %lu) lag %lu", Ok, Failed,
         FailNames[(uint32_t)LastFail], LastFailLine, Lag);
  else
    Line(13, "Frames: %lu ok, lag %lu, stall %lu, out %lu", Ok, Lag,
         Stalled, OutLate);
  if (WatchdogReset && (ResetStage[0] || ResetStage[1]))
    Line(14, "Hang reset: core0 %lu core1 %lu", ResetStage[0], ResetStage[1]);
  else
    Line(14, "Out: late %lu y %lu-%lu, irq %lu us", OutLate, OutFirst, OutLast,
         OutMaxUs);
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

// The 1200 baud BOOTSEL reset, as in the SDK, but with the watchdog, core 1
// and the DMA stopped first: with them running, the Pico often vanished from
// USB instead of coming up as a drive.
extern "C" void tud_cdc_line_coding_cb(uint8_t, cdc_line_coding_t const *Coding) {
  if (Coding->bit_rate != 1200)
    return;
  hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
  multicore_reset_core1();
  for (uint32_t Chan = 0; Chan != NUM_DMA_CHANNELS; ++Chan)
    dma_channel_abort(Chan);
  rom_reset_usb_boot_extra(-1, 0, false);
}

int main() {
  // Only a timeout of this firmware's watchdog, not a reboot into BOOTSEL
  // for flashing, leaves meaningful stages behind.
  if (watchdog_enable_caused_reboot()) {
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
