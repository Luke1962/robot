/*
 XMEM LCD Library for the Arduino

 Copyright 2012,2013 Andrew Brown

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 This notice may not be removed or altered from any source distribution.
 */

/**
 * @file Generic16BitILI9325.h
 * @brief Typedefs for using an ILI9325 in 16-bit mode
 * @ingroup ILI9325
 */

#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "PanelConfiguration.h"
#include "gl/Point.h"
#include "gl/Rectangle.h"
#include "gl/Size.h"
#include "gl/Bitmap.h"
#include "gl/DoublePrecision.h"
#include "gl/ColourNames.h"
#include "drv/accessModes/Xmem16AccessMode.h"
#include "drv/accessModes/Gpio16LatchAccessMode.h"
#include "drv/accessModes/Gpio16AccessMode.h"
#include "drv/ILI9325/ILI9325.h"
#include "GetFarAddress.h"
#include "Font.h"
#include "decoders/PicoJpeg.h"
#include "decoders/LzgFlashDecoder.h"
#include "gl/GraphicsLibrary.h"
#include "Backlight.h"
#include "terminal/TerminalPortraitImpl.h"
#include "terminal/TerminalLandscapeImpl.h"

namespace lcd {

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

  /*
   * Generic ILI9325 XMEM16 interface: 64K colours, portrait and landscape
   */

  typedef GraphicsLibrary<ILI9325<PORTRAIT,COLOURS_16BIT,Xmem16AccessMode>,Xmem16AccessMode> ILI9325_Portrait_64K_Xmem16;
  typedef GraphicsLibrary<ILI9325<LANDSCAPE,COLOURS_16BIT,Xmem16AccessMode>,Xmem16AccessMode> ILI9325_Landscape_64K_Xmem16;

  typedef TerminalPortraitImpl<ILI9325_Portrait_64K_Xmem16> ILI9325_Terminal_Portrait_64K_Xmem16;
  typedef TerminalLandscapeImpl<ILI9325_Landscape_64K_Xmem16> ILI9325_Terminal_Landscape_64K_Xmem16;

  /*
   * Generic ILI9325 GPIO16 latched interface: 64K colours, portrait and landscape
   */

  typedef GraphicsLibrary<ILI9325<PORTRAIT,COLOURS_16BIT,DefaultMegaGpio16LatchAccessMode>,DefaultMegaGpio16LatchAccessMode> ILI9325_Portrait_64K_Gpio16Latch;
  typedef GraphicsLibrary<ILI9325<LANDSCAPE,COLOURS_16BIT,DefaultMegaGpio16LatchAccessMode>,DefaultMegaGpio16LatchAccessMode> ILI9325_Landscape_64K_Gpio16Latch;

  typedef TerminalPortraitImpl<ILI9325_Portrait_64K_Gpio16Latch> ILI9325_Terminal_Portrait_64K_Gpio16Latch;
  typedef TerminalLandscapeImpl<ILI9325_Landscape_64K_Gpio16Latch> ILI9325_Terminal_Landscape_64K_Gpio16Latch;

  /*
   * Generic ILI9325 GPIO16 interface: 64K colours, portrait and landscape
   */

  typedef GraphicsLibrary<ILI9325<PORTRAIT,COLOURS_16BIT,DefaultMegaGpio16AccessMode>,DefaultMegaGpio16AccessMode> ILI9325_Portrait_64K_Gpio16;
  typedef GraphicsLibrary<ILI9325<LANDSCAPE,COLOURS_16BIT,DefaultMegaGpio16AccessMode>,DefaultMegaGpio16AccessMode> ILI9325_Landscape_64K_Gpio16;

  typedef TerminalPortraitImpl<ILI9325_Portrait_64K_Gpio16> ILI9325_Terminal_Portrait_64K_Gpio16;
  typedef TerminalLandscapeImpl<ILI9325_Landscape_64K_Gpio16> ILI9325_Terminal_Landscape_64K_Gpio16;

  /*
   * The default for most people is a PWM output on pin #2
   */

  typedef Backlight<2> DefaultBacklight;

#endif
}
