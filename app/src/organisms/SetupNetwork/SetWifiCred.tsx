import * as React from 'react'
import { useTranslation } from 'react-i18next'
import { useSelector, useDispatch } from 'react-redux'
import { useHistory } from 'react-router-dom'
import last from 'lodash/last'
import { css } from 'styled-components'

import {
  Box,
  Flex,
  DIRECTION_COLUMN,
  DIRECTION_ROW,
  ALIGN_CENTER,
  SPACING,
  Icon,
  Btn,
  JUSTIFY_SPACE_BETWEEN,
  POSITION_FIXED,
  useInterval,
} from '@opentrons/components'

import { StyledText } from '../../atoms/text'
import { InputField } from '../../atoms/InputField'
import { NormalKeyboard } from '../../atoms/SoftwareKeyboard'
import { TertiaryButton } from '../../atoms/buttons'
import * as RobotApi from '../../redux/robot-api'
import * as Networking from '../../redux/networking'
import { getLocalRobot } from '../../redux/discovery'
import { ConnectingNetwork } from './ConnectingNetwork'
import { FailedToConnect } from './FailedToConnect'
import { ConnectedNetworkInfo } from '../../pages/OnDeviceDisplay/ConnectedNetworkInfo'
import {
  /* 
  Note: kj 12/27/2022 These will be used in RobotSettings networking function
  CONNECT,
  DISCONNECT,
  */
  JOIN_OTHER,
} from '../Devices/RobotSettings/ConnectNetwork/constants'

import type { State, Dispatch } from '../../redux/types'
import type {
  WifiConfigureRequest,
  NetworkChangeState,
} from '../Devices/RobotSettings/ConnectNetwork/types'

const STATUS_REFRESH_MS = 5000
const LIST_REFRESH_MS = 10000

const SSID_INPUT_FIELD_STYLE = css`
  padding-top: ${SPACING.spacing5};
  padding-bottom: ${SPACING.spacing5};
  font-size: 2.5rem;
  line-height: 3.25rem;
  text-align: center;
`

interface SetWifiCredProps {
  ssid: string
  authType?: 'wpa' | 'none'
  setIsShowSetWifiCred: (isShow: boolean) => void
  // setRequiredRerender: (requiredRerender: boolean) => void
}

export function SetWifiCred({
  ssid,
  authType = 'wpa',
  setIsShowSetWifiCred,
}: // setRequiredRerender,
SetWifiCredProps): JSX.Element {
  const { t } = useTranslation(['device_settings', 'shared'])
  const localRobot = useSelector(getLocalRobot)
  const robotName = localRobot?.name != null ? localRobot.name : 'no name'
  const keyboardRef = React.useRef(null)
  const [password, setPassword] = React.useState<string>('')
  const [showPassword, setShowPassword] = React.useState<boolean>(false)
  const [changeState, setChangeState] = React.useState<NetworkChangeState>({
    type: null,
  })
  const dispatch = useDispatch<Dispatch>()
  const [dispatchApiRequest, requestIds] = RobotApi.useDispatchApiRequest()
  const requestState = useSelector((state: State) => {
    const lastId = last(requestIds)
    return lastId != null ? RobotApi.getRequestById(state, lastId) : null
  })
  const history = useHistory()
  const list = useSelector((state: State) =>
    Networking.getWifiList(state, robotName)
  )
  const selectedNetwork = list.find(nw => nw.ssid === ssid)

  const formatNetworkSettings = (): WifiConfigureRequest => {
    const securityType = selectedNetwork?.securityType
    const hidden = false // ToDo update for manual connect when the design is ready for dev
    const psk = password

    return {
      ssid,
      securityType,
      hidden,
      psk,
      // eapConfig,
    }
  }

  const handleConnect = (): void => {
    const options = formatNetworkSettings()
    dispatchApiRequest(Networking.postWifiConfigure(robotName, options))
    // ToDo There will be needed codes for manual connect for wpa-2
    if (changeState.type === JOIN_OTHER) {
      setChangeState({ ...changeState, ssid: options.ssid })
    }
  }

  useInterval(
    () => dispatch(Networking.fetchWifiList(robotName)),
    LIST_REFRESH_MS,
    true
  )

  useInterval(
    () => dispatch(Networking.fetchStatus(robotName)),
    STATUS_REFRESH_MS,
    true
  )

  return (
    <>
      {requestState == null ? (
        <Flex flexDirection={DIRECTION_COLUMN} margin={SPACING.spacingXXL}>
          <Flex
            flexDirection={DIRECTION_ROW}
            justifyContent={JUSTIFY_SPACE_BETWEEN}
            alignItems={ALIGN_CENTER}
            marginBottom="3.0625rem"
          >
            <Btn onClick={() => setIsShowSetWifiCred(false)}>
              <Flex flexDirection={DIRECTION_ROW}>
                <Icon
                  name="arrow-back"
                  marginRight={SPACING.spacing2}
                  size="1.875rem"
                />
                <StyledText
                  fontSize="1.625rem"
                  lineHeight="2.1875rem"
                  fontWeight="700"
                >
                  {t('shared:back')}
                </StyledText>
              </Flex>
            </Btn>
            <StyledText fontSize="2rem" lineHeight="2.75rem" fontWeight="700">
              {`${ssid}`}
            </StyledText>
            <TertiaryButton
              width="8.9375rem"
              height="3.75rem"
              fontSize="1.5rem"
              fontWeight="500"
              lineHeight="2.0425rem"
              onClick={handleConnect}
            >
              {t('connect')}
            </TertiaryButton>
          </Flex>

          <Flex
            width="100%"
            flexDirection={DIRECTION_COLUMN}
            paddingLeft="6.25rem"
          >
            <StyledText
              marginBottom="0.75rem"
              fontSize="1.375rem"
              lineHeight="1.875rem"
              fontWeight="500"
            >
              {'Enter password'}
            </StyledText>
            <Flex flexDirection={DIRECTION_ROW}>
              <Box width="36.375rem">
                <InputField
                  aria-label="wifi_password"
                  value={password}
                  onChange={e => setPassword(e.target.value)}
                  type={showPassword ? 'text' : 'password'}
                  css={SSID_INPUT_FIELD_STYLE}
                />
              </Box>
              <Btn
                marginLeft="1.5rem"
                onClick={() => setShowPassword(currentState => !currentState)}
              >
                <Flex flexDirection={DIRECTION_ROW} alignItems={ALIGN_CENTER}>
                  <Icon
                    name={showPassword ? 'eye-slash' : 'eye'}
                    width="2.75rem"
                    height="1.875rem"
                  />
                  <StyledText marginLeft={SPACING.spacing4}>
                    {showPassword ? t('hide') : t('show')}
                  </StyledText>
                </Flex>
              </Btn>
            </Flex>
          </Flex>

          <Flex width="100%" position={POSITION_FIXED} left="0" bottom="0">
            <NormalKeyboard
              onChange={e => e != null && setPassword(String(e))}
              keyboardRef={keyboardRef}
            />
          </Flex>
        </Flex>
      ) : (
        <>
          {requestState?.status === RobotApi.PENDING ? (
            <ConnectingNetwork />
          ) : requestState?.status === RobotApi.SUCCESS ? (
            <ConnectedNetworkInfo ssid={ssid} />
          ) : (
            <FailedToConnect
              ssid={ssid}
              requestState={requestState}
              type={changeState.type}
              onConnect={handleConnect}
              setIsShowSetWifiCred={setIsShowSetWifiCred}
              setRequiredRerender={setRequiredRerender}
            />
          )}
        </>
      )}
    </>
  )
}
