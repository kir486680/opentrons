import * as React from 'react'
import { useTranslation } from 'react-i18next'
import styled from 'styled-components'
import map from 'lodash/map'
import {
  ALIGN_CENTER,
  ALIGN_FLEX_START,
  ALIGN_STRETCH,
  COLORS,
  DIRECTION_COLUMN,
  Flex,
  Icon,
  JUSTIFY_SPACE_BETWEEN,
  LabwareRender,
  Module,
  RobotWorkSpace,
  SPACING,
  TYPOGRAPHY,
} from '@opentrons/components'
import {
  CompletedProtocolAnalysis,
  getDeckDefFromRobotType,
  getLabwareDisplayName,
  HeaterShakerCloseLatchCreateCommand,
  HeaterShakerOpenLatchCreateCommand,
  HEATERSHAKER_MODULE_TYPE,
  inferModuleOrientationFromXCoordinate,
  LabwareDefinition2,
  THERMOCYCLER_MODULE_V1,
} from '@opentrons/shared-data'
import {
  useCreateLiveCommandMutation,
  useModulesQuery,
} from '@opentrons/react-api-client'

import { StyledText } from '../../atoms/text'
import { BackButton } from '../../atoms/buttons'
import { ContinueButton, DeckMapButton } from '../ProtocolSetupModules' // Probably should move this to atoms/buttons as well
import { Portal } from '../../App/portal'
import { Modal } from '../../molecules/Modal'

import { useMostRecentCompletedAnalysis } from '../LabwarePositionCheck/useMostRecentCompletedAnalysis'
import { getLabwareDisplayLocation } from '../CommandText/utils'
import { getLabwareSetupItemGroups } from '../Devices/ProtocolRun/SetupLabware/utils'
import { getProtocolModulesInfo } from '../Devices/ProtocolRun/utils/getProtocolModulesInfo'
import { getAttachedProtocolModuleMatches } from '../ProtocolSetupModules/utils'
import { getLabwareRenderInfo } from '../Devices/ProtocolRun/utils/getLabwareRenderInfo'
import { ROBOT_MODEL_OT3 } from '../../redux/discovery'

import type { LabwareSetupItem } from '../Devices/ProtocolRun/SetupLabware/types'
import type { SetupScreens } from '../../pages/OnDeviceDisplay/ProtocolSetup'
import type { AttachedProtocolModuleMatch } from '../ProtocolSetupModules/utils'
import type { LatchStatus } from '../../redux/modules/api-types'

const OT3_STANDARD_DECK_VIEW_LAYER_BLOCK_LIST: string[] = [
  'DECK_BASE',
  'BARCODE_COVERS',
  'SLOT_SCREWS',
  'SLOT_10_EXPANSION',
  'CALIBRATION_CUTOUTS',
]
const EQUIPMENT_POLL_MS = 2000

const LabwareThumbnail = styled.svg`
  transform: scale(1, -1);
  width: 12rem;
  flex-shrink: 0;
`

export interface ProtocolSetupLabwareProps {
  runId: string
  setSetupScreen: React.Dispatch<React.SetStateAction<SetupScreens>>
}

export function ProtocolSetupLabware({
  runId,
  setSetupScreen,
}: ProtocolSetupLabwareProps): JSX.Element {
  const { t } = useTranslation('protocol_setup')
  // const { t: commandTextTranslator } = useTranslation('protocol_command_text')
  const [showDeckMapModal, setShowDeckMapModal] = React.useState<boolean>(false)
  const [
    showLabwareDetailsModal,
    setShowLabwareDetailsModal,
  ] = React.useState<boolean>(false)
  const selectedLabwareRef = React.useRef<LabwareDefinition2 | null>(null)

  const mostRecentAnalysis = useMostRecentCompletedAnalysis(runId)
  const deckDef = getDeckDefFromRobotType(ROBOT_MODEL_OT3)
  const { offDeckItems, onDeckItems } = getLabwareSetupItemGroups(
    mostRecentAnalysis?.commands ?? []
  )
  const labwareRenderInfo =
    mostRecentAnalysis != null
      ? getLabwareRenderInfo(mostRecentAnalysis, deckDef)
      : {}
  const moduleQuery = useModulesQuery({ refetchInterval: EQUIPMENT_POLL_MS })
  const attachedModules = moduleQuery?.data?.data ?? []
  const protocolModulesInfo =
    mostRecentAnalysis != null
      ? getProtocolModulesInfo(mostRecentAnalysis, deckDef)
      : []
  const attachedProtocolModuleMatches = getAttachedProtocolModuleMatches(
    attachedModules,
    protocolModulesInfo
  )

  const handleLabwareClick = (labware: LabwareDefinition2): void => {
    setShowLabwareDetailsModal(true)
    selectedLabwareRef.current = labware
  }
  return (
    <>
      <Portal level="top">
        {showDeckMapModal ? (
          <Modal
            title={t('map_view')}
            onClose={() => setShowDeckMapModal(false)}
            fullPage
          >
            <RobotWorkSpace
              deckDef={deckDef}
              deckLayerBlocklist={OT3_STANDARD_DECK_VIEW_LAYER_BLOCK_LIST}
              id="LabwareSetup_deckMap"
            >
              {() => (
                <>
                  {map(
                    attachedProtocolModuleMatches,
                    ({
                      x,
                      y,
                      moduleDef,
                      nestedLabwareDef,
                      nestedLabwareId,
                    }) => (
                      <Module
                        key={`LabwareSetup_Module_${String(
                          moduleDef.model
                        )}_${x}${y}`}
                        x={x}
                        y={y}
                        orientation={inferModuleOrientationFromXCoordinate(x)}
                        def={moduleDef}
                        innerProps={
                          moduleDef.model === THERMOCYCLER_MODULE_V1
                            ? { lidMotorState: 'open' }
                            : {}
                        }
                      >
                        {nestedLabwareDef != null && nestedLabwareId != null ? (
                          <React.Fragment
                            key={`LabwareSetup_Labware_${String(
                              nestedLabwareDef.metadata.displayName
                            )}_${x}${y}`}
                          >
                            <LabwareRender
                              definition={nestedLabwareDef}
                              onLabwareClick={handleLabwareClick}
                            />
                          </React.Fragment>
                        ) : null}
                      </Module>
                    )
                  )}
                  {map(labwareRenderInfo, ({ x, y, labwareDef }) => {
                    return (
                      <React.Fragment
                        key={`LabwareSetup_Labware_${String(
                          labwareDef.metadata.displayName
                        )}_${x}${y}`}
                      >
                        <g transform={`translate(${x},${y})`}>
                          <LabwareRender
                            definition={labwareDef}
                            onLabwareClick={handleLabwareClick}
                          />
                        </g>
                      </React.Fragment>
                    )
                  })}
                </>
              )}
            </RobotWorkSpace>
          </Modal>
        ) : null}
        {showLabwareDetailsModal && selectedLabwareRef.current != null ? (
          <Modal
            onClose={() => {
              setShowLabwareDetailsModal(false)
              selectedLabwareRef.current = null
            }}
            minHeight="14.375rem"
            minWidth="43.1875rem"
          >
            <Flex alignItems={ALIGN_STRETCH} gridGap="3rem">
              <LabwareThumbnail
                viewBox={` 0 0 ${String(
                  selectedLabwareRef.current.dimensions.xDimension
                )} ${String(selectedLabwareRef.current.dimensions.yDimension)}`}
              >
                <LabwareRender definition={selectedLabwareRef.current} />
              </LabwareThumbnail>
              <Flex
                flexDirection={DIRECTION_COLUMN}
                alignItems={ALIGN_FLEX_START}
                gridGap="1rem"
              >
                <StyledText>
                  {/* {mostRecentAnalysis != null
                    ? getLabwareDisplayLocation(
                        mostRecentAnalysis,
                        selectedLabwareRef.current.initialLocation <- no location on labware def
                        commandTextTranslator
                      )
                    : null} */}
                  PLACEHOLDER SLOT
                </StyledText>
                <StyledText
                  fontWeight={TYPOGRAPHY.fontWeightSemiBold}
                  fontSize="1.5rem"
                >
                  {getLabwareDisplayName(selectedLabwareRef.current)}
                </StyledText>
              </Flex>
            </Flex>
          </Modal>
        ) : null}
      </Portal>
      <Flex justifyContent={JUSTIFY_SPACE_BETWEEN}>
        <BackButton onClick={() => setSetupScreen('prepare to run')}>
          {t('labware')}
        </BackButton>
        <Flex gridGap="2.5rem">
          <ContinueButton onClick={() => setSetupScreen('lpc')} />
        </Flex>
      </Flex>
      <Flex
        flexDirection={DIRECTION_COLUMN}
        gridGap={SPACING.spacing3}
        marginTop={SPACING.spacing6}
      >
        <Flex justifyContent={JUSTIFY_SPACE_BETWEEN}>
          <Flex paddingLeft={SPACING.spacing5} width="16%">
            <StyledText>{'Location'}</StyledText>
          </Flex>
          <Flex width="84%">
            <StyledText>{'Labware Name'}</StyledText>
          </Flex>
        </Flex>
        {[...onDeckItems, ...offDeckItems].map((labware, i) => {
          return mostRecentAnalysis != null ? (
            <RowLabware
              key={i}
              labware={labware}
              robotSideAnalysis={mostRecentAnalysis}
              attachedProtocolModules={attachedProtocolModuleMatches}
              refetchModules={moduleQuery.refetch}
            />
          ) : null
        })}
      </Flex>
      <DeckMapButton onClick={() => setShowDeckMapModal(true)} />
    </>
  )
}

interface LabwareLatchProps {
  toggleLatch: () => void
  latchStatus: LatchStatus
}

function LabwareLatch({
  toggleLatch,
  latchStatus,
}: LabwareLatchProps): JSX.Element {
  const { t } = useTranslation(['protocol_setup', 'heater_shaker'])
  let buttonState: JSX.Element | null = null
  switch (latchStatus) {
    case 'idle_open':
      buttonState = (
        <Flex
          width="100%"
          justifyContent={JUSTIFY_SPACE_BETWEEN}
          alignItems={ALIGN_CENTER}
        >
          <StyledText
            fontSize="1.375rem"
            fontWeight={TYPOGRAPHY.fontWeightRegular}
          >
            {t('heater_shaker:open')}
          </StyledText>
          <Icon name="latch-open" size="2.5rem" color="#006CFA" />
        </Flex>
      )
      break
    case 'idle_closed':
      buttonState = (
        <Flex
          width="100%"
          justifyContent={JUSTIFY_SPACE_BETWEEN}
          alignItems={ALIGN_CENTER}
        >
          <StyledText
            fontSize="1.375rem"
            fontWeight={TYPOGRAPHY.fontWeightRegular}
          >
            {t('heater_shaker:closed')}
          </StyledText>
          <Icon name="latch-closed" size="2.5rem" />
        </Flex>
      )
      break
    case 'opening':
      buttonState = (
        <Flex
          width="100%"
          justifyContent={JUSTIFY_SPACE_BETWEEN}
          alignItems={ALIGN_CENTER}
        >
          <StyledText
            fontSize="1.375rem"
            fontWeight={TYPOGRAPHY.fontWeightRegular}
          >
            {t('heater_shaker:opening')}
          </StyledText>
          <Icon name="latch-closed" size="2.5rem" />
        </Flex>
      )
      break
    case 'closing':
      buttonState = (
        <Flex
          width="100%"
          justifyContent={JUSTIFY_SPACE_BETWEEN}
          alignItems={ALIGN_CENTER}
        >
          <StyledText
            fontSize="1.375rem"
            fontWeight={TYPOGRAPHY.fontWeightRegular}
          >
            {t('heater_shaker:closing')}
          </StyledText>
          <Icon name="latch-open" size="2.5rem" />
        </Flex>
      )
      break
    default:
      buttonState = null
  }
  return (
    <Flex
      minWidth="11.4375rem"
      height="7rem"
      flexDirection={DIRECTION_COLUMN}
      justifyContent={JUSTIFY_SPACE_BETWEEN}
      alignItems={ALIGN_FLEX_START}
      padding={SPACING.spacing4}
      gridGap="0.75rem"
      backgroundColor="#B4D4FF"
      color={
        latchStatus === 'opening' || latchStatus === 'closing'
          ? `${COLORS.darkBlack_hundred}${COLORS.opacity60HexCode}`
          : COLORS.darkBlackEnabled
      }
      borderRadius="0.75rem"
      onClick={toggleLatch}
    >
      <StyledText
        fontWeight={TYPOGRAPHY.fontWeightSemiBold}
        fontSize="1.375rem"
      >
        {t('labware_latch')}
      </StyledText>
      {buttonState}
    </Flex>
  )
}

interface RowLabwareProps {
  labware: LabwareSetupItem
  robotSideAnalysis: CompletedProtocolAnalysis
  attachedProtocolModules: AttachedProtocolModuleMatch[]
  refetchModules: () => void
}

function RowLabware({
  labware,
  robotSideAnalysis,
  attachedProtocolModules,
  refetchModules,
}: RowLabwareProps): JSX.Element | null {
  const { definition, initialLocation, nickName } = labware
  const { createLiveCommand } = useCreateLiveCommandMutation()
  const { t: commandTextTranslator } = useTranslation('protocol_command_text')
  const { t: setupTextTranslator } = useTranslation('protocol_setup')
  const matchedModule =
    initialLocation !== 'offDeck' &&
    'moduleId' in initialLocation &&
    attachedProtocolModules.length > 0
      ? attachedProtocolModules.find(
          mod => mod.moduleId === initialLocation.moduleId
        )
      : null
  const moduleInstructions = (
    <StyledText>{setupTextTranslator('labware_latch_instructions')}</StyledText>
  )

  let latchCommand:
    | HeaterShakerCloseLatchCreateCommand
    | HeaterShakerOpenLatchCreateCommand
  let latchStatus: LatchStatus = 'unknown'
  if (
    matchedModule != null &&
    matchedModule.attachedModuleMatch != null &&
    matchedModule.attachedModuleMatch.moduleType === HEATERSHAKER_MODULE_TYPE
  ) {
    latchStatus = matchedModule.attachedModuleMatch.data.labwareLatchStatus
    latchCommand = {
      commandType:
        latchStatus === 'idle_closed' || latchStatus === 'closing'
          ? 'heaterShaker/openLabwareLatch'
          : 'heaterShaker/closeLabwareLatch',
      params: { moduleId: matchedModule.attachedModuleMatch.id },
    }
  }

  const toggleLatch = (): void => {
    createLiveCommand({
      command: latchCommand,
    })
      .then(() => {
        refetchModules()
      })
      .catch((e: Error) => {
        console.error(
          `error setting module status with command type ${latchCommand.commandType}: ${e.message}`
        )
      })
  }

  const isOnHeaterShaker =
    matchedModule != null &&
    matchedModule.attachedModuleMatch?.moduleType === HEATERSHAKER_MODULE_TYPE

  return (
    <Flex
      alignItems={ALIGN_CENTER}
      backgroundColor={COLORS.light_one}
      borderRadius="1rem"
      justifyContent={JUSTIFY_SPACE_BETWEEN}
      padding={`${SPACING.spacing4} ${SPACING.spacing5}`}
      gridGap={SPACING.spacing6}
    >
      <Flex minWidth="10%">
        <StyledText>
          {getLabwareDisplayLocation(
            robotSideAnalysis,
            initialLocation,
            commandTextTranslator
          )}
        </StyledText>
      </Flex>
      <Flex
        flexDirection={DIRECTION_COLUMN}
        width="86%"
        gridGap={SPACING.spacing2}
        alignSelf={ALIGN_FLEX_START}
      >
        <StyledText fontSize="1.5rem" fontWeight="500">
          {getLabwareDisplayName(definition)}
        </StyledText>
        <StyledText>{nickName}</StyledText>
        {isOnHeaterShaker ? moduleInstructions : null}
      </Flex>
      {isOnHeaterShaker ? (
        <LabwareLatch toggleLatch={toggleLatch} latchStatus={latchStatus} />
      ) : null}
    </Flex>
  )
}
