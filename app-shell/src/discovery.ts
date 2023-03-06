// app shell discovery module
import { app, MessageChannelMain } from 'electron'
import Store from 'electron-store'
import groupBy from 'lodash/groupBy'
import throttle from 'lodash/throttle'

import {
  createDiscoveryClient,
  DEFAULT_PORT,
} from '@opentrons/discovery-client'

import { UI_INITIALIZED } from '@opentrons/app/src/redux/shell/actions'
import {
  DISCOVERY_START,
  DISCOVERY_FINISH,
  DISCOVERY_REMOVE,
  CLEAR_CACHE,
} from '@opentrons/app/src/redux/discovery/actions'

import { getFullConfig, handleConfigChange } from './config'
import { createLogger } from './log'

import type { BrowserWindow } from 'electron'

import type {
  Address,
  DiscoveryClientRobot,
  LegacyService,
  DiscoveryClient,
} from '@opentrons/discovery-client'

import type { Action, Dispatch } from './types'
import type { Config } from './config'

const log = createLogger('discovery')

// TODO(mc, 2018-08-09): values picked arbitrarily and should be researched
const FAST_POLL_INTERVAL_MS = 3000
const SLOW_POLL_INTERVAL_MS = 15000
const UPDATE_THROTTLE_MS = 500

interface DiscoveryStore {
  robots: DiscoveryClientRobot[]
  services?: LegacyService[]
}

let config: Config['discovery']
let store: Store<DiscoveryStore>
let client: DiscoveryClient

const makeManualAddresses = (addrs: string | string[]): Address[] => {
  return ['fd00:0:cafe:fefe::1']
    .concat(addrs)
    .map(ip => ({ ip, port: DEFAULT_PORT }))
}

const migrateLegacyServices = (
  legacyServices: LegacyService[]
): DiscoveryClientRobot[] => {
  const servicesByName = groupBy<LegacyService>(legacyServices, 'name')

  return Object.keys(servicesByName).map((name: string) => {
    const services = servicesByName[name]
    const addresses = services.flatMap((service: LegacyService) => {
      const { ip, port } = service
      return ip != null
        ? [
            {
              ip,
              port,
              seen: false,
              healthStatus: null,
              serverHealthStatus: null,
              healthError: null,
              serverHealthError: null,
              advertisedModel: null,
            },
          ]
        : []
    })

    return { name, health: null, serverHealth: null, addresses }
  })
}

export function registerDiscovery(
  dispatch: Dispatch,
  mainWindow: BrowserWindow
): (action: Action) => unknown {
  const handleRobotListChange = throttle(handleRobots, UPDATE_THROTTLE_MS)

  config = getFullConfig().discovery
  store = new Store({
    name: 'discovery',
    defaults: { robots: [] as DiscoveryClientRobot[] },
  })

  let disableCache = config.disableCache
  let initialRobots: DiscoveryClientRobot[] = []

  // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
  if (!disableCache) {
    const legacyCachedServices: LegacyService[] | undefined = store.get(
      'services',
      // @ts-expect-error(mc, 2021-02-16): tweak these type definitions
      null
    )

    // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
    if (legacyCachedServices) {
      initialRobots = migrateLegacyServices(legacyCachedServices)
      store.delete('services')
    } else {
      initialRobots = store.get('robots', [])
    }
  }

  client = createDiscoveryClient({
    onListChange: handleRobotListChange,
    logger: log,
  })

  // // create message ports
  const { port1, port2 } = new MessageChannelMain()

  port2.postMessage({ test: 21 })
  port2.on('message', event => {
    console.log('from renderer main world:', event.data)
  })
  port2.postMessage({ httpAgent: client.httpAgent })
  port2.start()

  mainWindow.webContents.postMessage('main-world-port', null, [port1])

  client.start({
    initialRobots,
    healthPollInterval: SLOW_POLL_INTERVAL_MS,
    manualAddresses: makeManualAddresses(config.candidates),
  })

  handleConfigChange('discovery.candidates', (value: string | string[]) => {
    client.start({ manualAddresses: makeManualAddresses(value) })
  })

  handleConfigChange('discovery.disableCache', (value: boolean) => {
    // eslint-disable-next-line @typescript-eslint/no-unnecessary-boolean-literal-compare
    if (value === true) {
      disableCache = value
      store.set('robots', [])
      clearCache()
    }
  })

  app.once('will-quit', () => {
    client.stop()
  })

  return function handleIncomingAction(action: Action) {
    log.debug('handling action in discovery', { action })

    switch (action.type) {
      case UI_INITIALIZED:
      case DISCOVERY_START:
        handleRobots()
        return client.start({ healthPollInterval: FAST_POLL_INTERVAL_MS })

      case DISCOVERY_FINISH:
        return client.start({ healthPollInterval: SLOW_POLL_INTERVAL_MS })

      case DISCOVERY_REMOVE:
        return client.removeRobot(
          (action.payload as { robotName: string }).robotName
        )

      case CLEAR_CACHE:
        return clearCache()
    }
  }

  function handleRobots(): void {
    const robots = client.getRobots()

    if (!disableCache) store.set('robots', robots)

    dispatch({
      type: 'discovery:UPDATE_LIST',
      payload: { robots },
    })
  }

  function clearCache(): void {
    client.start({ initialRobots: [] })
  }
}
