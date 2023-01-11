import * as React from 'react'
import { saveAs } from 'file-saver'
import { useTranslation } from 'react-i18next'

import {
  Box,
  Flex,
  Icon,
  Link,
  ALIGN_CENTER,
  JUSTIFY_SPACE_BETWEEN,
  SPACING,
  TYPOGRAPHY,
} from '@opentrons/components'

import { TertiaryButton } from '../../atoms/buttons'
import { StyledText } from '../../atoms/text'
import {
  useDeckCalibrationData,
  useIsOT3,
  usePipetteOffsetCalibrations,
  useRobot,
  useTipLengthCalibrations,
} from '../../organisms/Devices/hooks'
import { useTrackEvent } from '../../redux/analytics'
import { EVENT_CALIBRATION_DOWNLOADED } from '../../redux/calibration'
import * as Config from '../../redux/config'

interface CalibrationDataDownloadProps {
  robotName: string
  setShowHowCalibrationWorksModal: (
    showHowCalibrationWorksModal: boolean
  ) => void
}

export function CalibrationDataDownload({
  robotName,
  setShowHowCalibrationWorksModal,
}: CalibrationDataDownloadProps): JSX.Element {
  const { t } = useTranslation([
    'device_settings',
    'robot_calibration',
    'shared',
  ])
  const doTrackEvent = useTrackEvent()

  const robot = useRobot(robotName)
  const isOT3 = useIsOT3(robotName)
  // wait for robot request to resolve instead of using name directly from params
  const deckCalibrationData = useDeckCalibrationData(robot?.name)
  const pipetteOffsetCalibrations = usePipetteOffsetCalibrations(robot?.name)
  const tipLengthCalibrations = useTipLengthCalibrations(robot?.name)

  const downloadIsPossible =
    deckCalibrationData.isDeckCalibrated &&
    pipetteOffsetCalibrations != null &&
    pipetteOffsetCalibrations.length > 0 &&
    tipLengthCalibrations != null &&
    tipLengthCalibrations.length > 0

  const ot3DownloadIsPossible =
    isOT3 &&
    pipetteOffsetCalibrations != null &&
    pipetteOffsetCalibrations.length > 0

  const onClickSaveAs: React.MouseEventHandler = e => {
    e.preventDefault()
    doTrackEvent({
      name: EVENT_CALIBRATION_DOWNLOADED,
      properties: {},
    })
    saveAs(
      new Blob([
        isOT3
          ? JSON.stringify({
              pipetteOffset: pipetteOffsetCalibrations,
            })
          : JSON.stringify({
              deck: deckCalibrationData,
              pipetteOffset: pipetteOffsetCalibrations,
              tipLength: tipLengthCalibrations,
            }),
      ]),
      `opentrons-${robotName}-calibration.json`
    )
  }

  return (
    <>
      {!isOT3 ? (
        <Box paddingTop={SPACING.spacing5} paddingBottom={SPACING.spacing2}>
          <Flex
            alignItems={ALIGN_CENTER}
            justifyContent={JUSTIFY_SPACE_BETWEEN}
          >
            <Box marginRight={SPACING.spacing6}>
              <Box css={TYPOGRAPHY.h3SemiBold} marginBottom={SPACING.spacing3}>
                {t('robot_calibration:download_calibration_title')}
              </Box>
              <StyledText as="p" marginBottom={SPACING.spacing3}>
                {t(
                  downloadIsPossible
                    ? 'robot_calibration:download_calibration_data_available'
                    : 'robot_calibration:download_calibration_data_unavailable'
                )}
              </StyledText>
            </Box>
            <TertiaryButton
              onClick={onClickSaveAs}
              disabled={!downloadIsPossible}
            >
              <Flex alignItems={ALIGN_CENTER}>
                <Icon
                  name="download"
                  size="0.75rem"
                  marginRight={SPACING.spacing3}
                />
                {t('download_calibration_data')}
              </Flex>
            </TertiaryButton>
          </Flex>
        </Box>
      ) : (
        <Box paddingBottom={SPACING.spacing5}>
          <Flex
            alignItems={ALIGN_CENTER}
            justifyContent={JUSTIFY_SPACE_BETWEEN}
          >
            <Box marginRight={SPACING.spacing6}>
              <Box css={TYPOGRAPHY.h3SemiBold} marginBottom={SPACING.spacing3}>
                {t('about_calibration_title')}
              </Box>
              <StyledText
                as="p"
                marginBottom={SPACING.spacing3}
                whiteSpace="pre-line"
              >
                {isOT3
                  ? t('about_calibration_description_ot3')
                  : t('about_calibration_description')}
              </StyledText>
              <Link
                role="button"
                css={TYPOGRAPHY.linkPSemiBold}
                onClick={() => setShowHowCalibrationWorksModal(true)}
              >
                {t('robot_calibration:see_how_robot_calibration_works')}
              </Link>
            </Box>
            <TertiaryButton
              onClick={onClickSaveAs}
              disabled={isOT3 && !ot3DownloadIsPossible}
            >
              <Flex alignItems={ALIGN_CENTER}>
                <Icon
                  name="download"
                  size="0.75rem"
                  marginRight={SPACING.spacing3}
                />
                {t('download_calibration_data')}
              </Flex>
            </TertiaryButton>
          </Flex>
        </Box>
      )}
    </>
  )
}
