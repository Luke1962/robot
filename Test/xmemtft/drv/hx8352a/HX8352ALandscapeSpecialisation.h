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
 * @file HX8352ALandscapeSpecialisation.h
 * @brief Specialisation of HX8352AOrientation for landscape mode.
 * @ingroup HX8352A
 */

#pragma once


namespace lcd {


	/**
	 * Specialisation of HX8352AOrientation for the panel in LANDSCAPE mode.
	 * @tparam TAccessMode the access mode implementation, e.g. Gpio16LatchAccessMode
	 * @ingroup HX8352A
	 */

	template<class TAccessMode,class TPanelTraits>
	class HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits> {

		protected:
			uint8_t getMemoryAccessControl() const;

		public:
			int16_t getWidth() const;
			int16_t getHeight() const;

			void moveTo(const Rectangle& rc) const;
			void moveTo(int16_t xstart,int16_t ystart,int16_t xend,int16_t yend) const;
			void moveX(int16_t xstart,int16_t xend) const;
			void moveY(int16_t ystart,int16_t yend) const;

			void setScrollPosition(int16_t scrollPosition) const;
	};


	/**
	 * Get the register setting for memory access control
	 * @return The entry mode register setting for portrait
	 */

	template<class TAccessMode,class TPanelTraits>
	inline uint8_t HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::getMemoryAccessControl() const {
		return 0xAA;		// MY | MV | BGR | SCROLL
	}


	/**
	 * Get the width in pixels from the panel traits.
	 * @return The panel width (e.g. 240)
	 */

	template<class TAccessMode,class TPanelTraits>
	inline int16_t HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::getWidth() const {
		return TPanelTraits::LONG_SIDE;
	}


	/**
	 * Get the height in pixels from the panel traits.
	 * @return The panel height (e.g. 400)
	 */

	template<class TAccessMode,class TPanelTraits>
	inline int16_t HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::getHeight() const {
		return TPanelTraits::SHORT_SIDE;
	}


	/**
	 * Move the display output rectangle
	 * @param rc The display output rectangle
	 */

	template<class TAccessMode,class TPanelTraits>
	inline void HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::moveTo(const Rectangle& rc) const {
		moveTo(rc.X,rc.Y,rc.X+rc.Width-1,rc.Y+rc.Height-1);
	}


	/**
	 * Move the display output rectangle
	 * @param xstart left-most x co-ordinate
	 * @param ystart top-most y co-ordinate
	 * @param xend right-most x co-ordinate
	 * @param yend bottom-most y co-ordinate
	 */

	template<class TAccessMode,class TPanelTraits>
	inline void HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::moveTo(int16_t xstart,int16_t ystart,int16_t xend,int16_t yend) const {

		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_START_H,xstart >> 8);
		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_START_L,xstart & 0xff);
		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_END_H,xend >> 8);
		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_END_L,xend & 0xff);

		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_START_H,ystart >> 8);
		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_START_L,ystart & 0xff);
		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_END_H,yend >> 8);
		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_END_L,yend & 0xff);
	}


	/**
	 * Move the X position
	 * @param xstart The new X start position
	 * @param xend The new X end position
	 */

	template<class TAccessMode,class TPanelTraits>
	inline void HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::moveX(int16_t xstart,int16_t xend) const {

		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_START_H,xstart >> 8);
		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_START_L,xstart & 0xff);
		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_END_H,xend >> 8);
		TAccessMode::writeCommandData(hx8352a::COLUMN_ADDRESS_END_L,xend & 0xff);
	}


	/**
	 * Move the Y position
	 * @param ystart The new Y position
	 */

	template<class TAccessMode,class TPanelTraits>
	inline void HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::moveY(int16_t ystart,int16_t yend) const {

		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_START_H,ystart >> 8);
		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_START_L,ystart & 0xff);
		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_END_H,yend >> 8);
		TAccessMode::writeCommandData(hx8352a::ROW_ADDRESS_END_L,yend & 0xff);
	}


	/**
	 * Set a vertical scroll position
	 * @param scrollPosition The new scroll position
	 */

	template<class TAccessMode,class TPanelTraits>
	inline void HX8352AOrientation<LANDSCAPE,TAccessMode,TPanelTraits>::setScrollPosition(int16_t scrollPosition) const {

		if(scrollPosition<0)
      scrollPosition+=TPanelTraits::LONG_SIDE;
    else if(scrollPosition>TPanelTraits::LONG_SIDE-1)
      scrollPosition-=TPanelTraits::LONG_SIDE;

    // write to the register

		TAccessMode::writeCommandData(hx8352a::VERTICAL_SCROLL_START_ADDRESS_H,(scrollPosition >> 8));
		TAccessMode::writeCommandData(hx8352a::VERTICAL_SCROLL_START_ADDRESS_L,scrollPosition & 0xff);
	}
}

