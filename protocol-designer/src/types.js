// @flow
import type {RootState as StepList} from './steplist/reducers'
import type {RootState as Navigation} from './navigation'
import type {RootState as LabwareIngred} from './labware-ingred/reducers'

export type BaseState = {
  steplist: StepList,
  navigation: Navigation,
  labwareIngred: LabwareIngred
}

export type Dispatch<A> = (action: A | ThunkAction<A>) => any
export type GetState = () => BaseState
export type ThunkAction<A> = (dispatch: Dispatch<A>, getState: GetState) => any

export type WellVolumes = {[wellName: string]: number}
// TODO LATER Ian 2018-02-19 type for containers.json
export type JsonWellData = {
  'total-liquid-volume': number
  // missing rest of fields, todo later
}
export type VolumeJson = {
  locations: {
    [wellName: string]: JsonWellData
  }
}
