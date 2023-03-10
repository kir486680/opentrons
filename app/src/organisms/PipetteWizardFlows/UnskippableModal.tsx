import * as React from 'react'
import { useTranslation } from 'react-i18next'
import { COLORS, TEXT_TRANSFORM_CAPITALIZE } from '@opentrons/components'
import { AlertPrimaryButton } from '../../atoms/buttons'
import { SmallButton } from '../../atoms/buttons/ODD/SmallButton'
import { SimpleWizardBody } from '../../molecules/SimpleWizardBody'

interface UnskippableModalProps {
  goBack: () => void
  isOnDevice: boolean | null
}

export function UnskippableModal(props: UnskippableModalProps): JSX.Element {
  const { goBack, isOnDevice } = props
  const { t } = useTranslation(['pipette_wizard_flows', 'shared'])
  return (
    <SimpleWizardBody
      iconColor={COLORS.warningEnabled}
      header={t('critical_unskippable_step')}
      subHeader={t('must_detach_mounting_plate')}
      isSuccess={false}
    >
      {isOnDevice ? (
        <SmallButton
          onClick={goBack}
          aria-label="isOnDevice_button"
          buttonText={t('shared:return')}
          buttonType="alert"
        />
      ) : (
        <AlertPrimaryButton
          onClick={goBack}
          textTransform={TEXT_TRANSFORM_CAPITALIZE}
        >
          {t('shared:return')}
        </AlertPrimaryButton>
      )}
    </SimpleWizardBody>
  )
}
