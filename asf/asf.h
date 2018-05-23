/*
 * asf.h
 *
 *  Created on: 13 Mar 2016
 *      Authors: David and Christian
 */

#ifndef ASF_H_
#define ASF_H_

#include "sam/utils/compiler.h"

#ifndef NO_STATUS_CODES
#if SAM4E || SAME70
#include "sam/drivers/afec/afec.h"
#elif SAMG55
#include "sam/drivers/adc2/adc2.h"
#else
#include "sam/drivers/adc/adc.h"
#endif

#if SAM4E || SAM3XA
#include "sam/drivers/dmac/dmac.h"
#endif
#if SAME70
#include "sam/drivers/xdmac/xdmac.h"
#endif
#endif

#if SAMG55
#else
#include "sam/drivers/dacc/dacc.h"
#include "sam/drivers/pwm/pwm.h"
#endif //SAMG55

#include "sam/drivers/pio/pio.h"
#include "sam/drivers/pmc/pmc.h"
#include "sam/drivers/spi/spi.h"
#include "sam/drivers/wdt/wdt.h"

#if SAM4E || SAME70
#include "sam/drivers/rswdt/rswdt.h"
#endif

#if SAM4E || SAM4S || SAME70
#include "sam/services/flash_efc/flash_efc.h"
#endif

#endif /* ASF_H_ */
