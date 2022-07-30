#include "cortex_m_hal/systick.h"
#include "postform/config.h"
#include "systick_config.h"

namespace Postform {
uint64_t getGlobalTimestamp() {
  SysTick& systick = SysTick::getInstance();
  return systick.getTickCount();
}
}  // namespace Postform

DECLARE_POSTFORM_CONFIG(.timestamp_frequency = App::SYSTICK_CLK_HZ);
