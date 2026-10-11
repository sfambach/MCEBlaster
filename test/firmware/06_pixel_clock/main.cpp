//-*- C++ -*-
//
// Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
// License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
// Free for noncommercial use; commercial use only with the author's consent.
//
// Step 6: pixel clock measurement for the MCE Blaster. The TTL output of a
// video card has no clock line, so the pixel clock is found from the colour
// changes: every run of one colour lasts a whole number of pixels. A PIO
// samples the colour pins at 126 MHz, the run lengths give the pixel time,
// and rounding pixels per line (pixel clock / line frequency) to a whole
// number makes the result exact. Shown on a 640x480 VGA screen and over USB.
//

#include "Font8x8.h"
#include "FastSample.pio.h"
#include "VGAOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cmath>
#include <initializer_list>
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
static constexpr uint32_t BlockChars = 34;

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
  // A block of lines, left aligned, centered as a whole.
  uint32_t X0 = Center ? (HVisible - Width) / 2
                       : (HVisible - BlockChars * FontWidth * Zoom) / 2;
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
// Times of the first and last edge: counting edges in 1 s alone gives only
// whole hertz, so 70 Hz flips between 70 and 71.
static volatile uint64_t FirstVSyncUs = 0, LastVSyncUs = 0;

static void vsyncIRQ(uint GPIO, uint32_t Events) {
  if (GPIO == TTL_VSYNC_GPIO && (Events & GPIO_IRQ_EDGE_RISE))
    LastVSyncUs = time_us_64();
    if (VSyncEdges++ == 0)
      FirstVSyncUs = LastVSyncUs;
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
// From the MCE Blaster's TimingsTTL.def. CGA and EGA 200 lines share one
// timing; the data pins that toggle tell them apart.
static constexpr TTLMode TTLModes[] = {
    {"MDA 720x350 50 Hz", 18430, 50},
    {"CGA/EGA 640x200 60 Hz", 15700, 60},
    {"EGA 640x350 60 Hz", 21850, 60},
    {"VGA 720x400 70 Hz", 31469, 70},
    {"VGA 640x480 60 Hz", 31469, 60},
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

// ---------------------------------------------------------------------------
// Pixel clock from the run lengths of the colour pins

static constexpr uint32_t SampleClkDiv = 2;
static constexpr float SampleHz = SysClkKHz * 1000.0f / SampleClkDiv;
static constexpr uint32_t BurstWords = 4096; // 16384 samples, 130 us
static constexpr uint32_t NumBursts = 24;
static constexpr uint32_t MaxRun = 256; // longer runs are blanking
static uint32_t Burst[BurstWords];
static uint32_t RunCount[MaxRun];
// Sample positions of the rising edges of each pin in one burst.
static constexpr uint32_t MaxEdges = 4096;
static uint16_t Edges[8][MaxEdges];
static uint32_t NumEdges[8];

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
    for (uint32_t B = 0; Rising; ++B, Rising >>= 1)
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
  for (uint32_t B = 0; B != 8; ++B) {
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

static void show(const Measurement &M) {
  char Txt[NumTxtLines][MaxChars + 1] = {};
  const char *Mode = guessMode(M);
  auto Line = [&Txt](uint32_t L, const char *Fmt, auto... Args) {
    std::snprintf(Txt[L], sizeof(Txt[L]), Fmt, Args...);
  };
  Line(0, "MCE BLASTER - PIXEL CLOCK");
  if (M.H.Hz >= 1000 && M.V.Hz >= 10)
    Line(2, "HSync %.3f kHz VSync %.2f Hz", M.H.Hz / 1000.0f, M.V.Hz);
  else
    Line(2, "HSync/VSync: no signal");
  Line(3, "Mode: %s", Mode ? Mode : "NO SIGNAL");

  uint32_t FirstGPIO = CGA_RGB_GPIO;
  uint8_t Mask = 0x3f;
  const char *Pins = "CGA GPIO 8-13";
  auto Toggles = [&M](uint32_t First, uint32_t Num) {
    uint32_t Bits = ((1u << Num) - 1) << First;
    return (M.SeenHigh & M.SeenLow & Bits) != 0;
  };
  if (!Toggles(CGA_RGB_GPIO, 6) && Toggles(EGA_RGB_GPIO, 6)) {
    FirstGPIO = EGA_RGB_GPIO;
    Pins = "EGA GPIO 0-5";
  } else if (!Toggles(CGA_RGB_GPIO, 6) && Toggles(MDA_VI_GPIO, 2)) {
    FirstGPIO = MDA_VI_GPIO;
    Mask = 0x3;
    Pins = "MDA GPIO 26-27";
  }
  PixelClock P;
  if (Mode)
    P = measurePixelClock(FirstGPIO, Mask, M.H.Hz);
  Line(4, "Sampled: %s", Pins);

  if (P.PixelSamples > 0) {
    Line(6, "Pixel time:  %7.2f ns",
         P.PixelSamples / SampleHz * 1e9f);
    Line(7, "Pixel clock: %7.3f MHz", P.Hz / 1e6f);
    if (P.PixelsPerLine)
      Line(8, "Pixels per line: %lu", P.PixelsPerLine);
    for (const KnownClock &K : KnownClocks)
      if (std::fabs(P.Hz - K.Hz) < K.Hz * 0.002f)
        Line(9, "Known clock: %.3f MHz %s", K.Hz / 1e6f, K.Name);
    Line(10, "Runs: %lu  fit: %.2f px", P.Runs, P.FitPx);
  } else if (Mode) {
    Line(6, "Pixel clock: not enough detail");
    Line(7, "Runs: %lu", P.Runs);
  }
  Line(12, "Show fine detail, e.g. the");
  Line(13, "shaded blocks, chars 176/177");
  Line(15, "Greetings SteFam");

  for (uint32_t L = 0; L != NumTxtLines; ++L) {
    bool Title = L == 0 || L == 7;
    drawTxtLine(L, Txt[L], Title ? Yellow : White, L == 15);
  }
  for (uint32_t L = 2; L != 11; ++L)
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
  initFastSample();

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
