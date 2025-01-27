import * as React from 'react'
import { useDispatch, useSelector } from 'react-redux'
import { useTranslation } from 'react-i18next'
import styled from 'styled-components'
import clamp from 'lodash/clamp'

import {
  Flex,
  Btn,
  DIRECTION_COLUMN,
  JUSTIFY_FLEX_START,
  ALIGN_CENTER,
  Icon,
  DIRECTION_ROW,
  Box,
  JUSTIFY_CENTER,
  BORDERS,
} from '@opentrons/components'

import { StyledText } from '../../../atoms/text'
import {
  getOnDeviceDisplaySettings,
  updateConfigValue,
} from '../../../redux/config'

import type { Dispatch } from '../../../redux/types'
import type { SettingOption } from '../../../pages/OnDeviceDisplay/RobotSettingsDashboard'

interface RectProps {
  isActive: boolean
}

const BrightnessTile = styled(Box)`
  width: 5.875rem;
  height: 8.75rem;
  border-radius: ${BORDERS.size_two};
  background: ${(props: RectProps) => (props.isActive ? '#9c3ba4' : '#E7C3E9')};
`

// Note The actual brightness is Bright 1 <---> 6 Dark which is opposite to the UI
// For UI Bright 6 <--> 1 Dark
// If the brightness 7 or more | 0, the display will be blackout
const LOWEST_BRIGHTNESS = 6
const HIGHEST_BRIGHTNESS = 1

interface DisplayBrightnessProps {
  setCurrentOption: (currentOption: SettingOption | null) => void
}

export function DisplayBrightness({
  setCurrentOption,
}: DisplayBrightnessProps): JSX.Element {
  const { t } = useTranslation(['device_settings'])
  const dispatch = useDispatch<Dispatch>()
  const initialBrightness = useSelector(getOnDeviceDisplaySettings).brightness
  const [brightness, setBrightness] = React.useState<number>(initialBrightness)

  const handleClick = (changeType: 'up' | 'down'): void => {
    const step = changeType === 'up' ? -1 : 1
    const nextBrightness = clamp(
      brightness + step,
      HIGHEST_BRIGHTNESS,
      LOWEST_BRIGHTNESS
    )
    dispatch(
      updateConfigValue('onDeviceDisplaySettings.brightness', nextBrightness)
    )
    setBrightness(nextBrightness)
  }

  return (
    <Flex flexDirection={DIRECTION_COLUMN}>
      <Flex justifyContent={JUSTIFY_FLEX_START} alignItems={ALIGN_CENTER}>
        <Btn
          onClick={() => setCurrentOption(null)}
          data-testid="DisplayBrightness_back_button"
        >
          <Icon name="chevron-left" size="2.5rem" />
        </Btn>
        <StyledText fontSize="2rem" textAlign="center">
          {t('display_brightness')}
        </StyledText>
      </Flex>
      <Flex
        flexDirection={DIRECTION_ROW}
        width="56.5rem"
        height="8.75rem"
        marginTop="7.625rem"
        alignItems={ALIGN_CENTER}
        justifyContent={JUSTIFY_CENTER}
      >
        <Btn
          disabled={brightness === LOWEST_BRIGHTNESS}
          onClick={() => handleClick('down')}
          data-testid="DisplayBrightness_decrease"
        >
          <Icon size="5rem" name="minus" />
        </Btn>
        <Flex flexDirection={DIRECTION_ROW} gridGap="0.4375rem">
          <BrightnessTile isActive={brightness <= 6} />
          <BrightnessTile isActive={brightness <= 5} />
          <BrightnessTile isActive={brightness <= 4} />
          <BrightnessTile isActive={brightness <= 3} />
          <BrightnessTile isActive={brightness <= 2} />
          <BrightnessTile isActive={brightness <= 1} />
        </Flex>

        <Btn
          disabled={brightness === HIGHEST_BRIGHTNESS}
          onClick={() => handleClick('up')}
          data-testid="DisplayBrightness_increase"
        >
          <Icon size="5rem" name="plus" />
        </Btn>
      </Flex>
    </Flex>
  )
}
