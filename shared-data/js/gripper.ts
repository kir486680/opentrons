import gripperV1 from '../gripper/definitions/1/v1.json'

import {
  GRIPPER_V1,
} from './constants'

import type {
  GripperModel,
  GripperDefinition,
} from './types'

export const getGripperDef = (gripperModel: GripperModel): GripperDefinition => {
  switch (gripperModel) {
    case GRIPPER_V1:
      return gripperV1 

    default:
      throw new Error(`Invalid gripper model ${gripperModel as string}`)
  }
}

export function getGripperDisplayName(gripperModel: GripperModel): string {
  return getGripperDef(gripperModel).displayName
}
