import styled from 'styled-components'
import {
  SPACING,
  COLORS,
  BORDERS,
  TYPOGRAPHY,
  styleProps,
  isntStyleProp,
} from '@opentrons/components'

import type { StyleProps } from '@opentrons/components'

interface SecondaryButtonProps extends StyleProps {
  /** button action is dangerous and may have non-reversible side-effects for user */
  isDangerous?: boolean
}
export const SecondaryButton = styled.button.withConfig<SecondaryButtonProps>({
  shouldForwardProp: p => isntStyleProp(p) && p !== 'isDangerous',
})<SecondaryButtonProps>`
  color: ${props =>
    props.isDangerous ? COLORS.errorText : COLORS.blueEnabled};
  border: ${BORDERS.lineBorder};
  border-color: ${props =>
    props.isDangerous ? COLORS.errorEnabled : 'initial'};
  border-radius: ${BORDERS.radiusSoftCorners};
  padding: ${SPACING.spacing3} ${SPACING.spacing4};
  text-transform: ${TYPOGRAPHY.textTransformNone};
  background-color: ${COLORS.transparent};
  ${TYPOGRAPHY.pSemiBold}

  &:hover,
  &:focus {
    box-shadow: 0px 3px 6px 0px rgba(0, 0, 0, 0.23);
  }

  &:hover {
    opacity: 70%;
    box-shadow: 0 0 0;
  }

  &:focus-visible {
    box-shadow: 0 0 0 3px ${COLORS.fundamentalsFocus};
  }

  &:active {
    box-shadow: none;
  }

  &:disabled,
  &.disabled {
    box-shadow: none;
    opacity: 50%;
  }

  ${styleProps}
`
