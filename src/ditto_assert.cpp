#include "cortex_m_hal/core.h"
#include "ditto/assert.h"
#include "postform/rtt/transport.h"
#include "postform/serial_logger.h"

extern Postform::SerialLogger<Postform::Rtt::Transport> logger;

namespace Ditto {
void assert_failed(const char* condition, int line, const char* file) {
  LOG_ERROR(&logger, "Assert `%s` failed at line %d, file %s", condition, line,
            file);
  CortexMHal::halt();
}

void unimplemented(const char* function, int line, const char* file) {
  LOG_ERROR(&logger, "Assert `%s` failed at line %d, file %s", function, line,
            file);
  CortexMHal::halt();
}
}  // namespace Ditto
