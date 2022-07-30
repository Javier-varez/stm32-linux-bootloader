#include <array>

#include "cortex_m_hal/systick.h"
#include "postform/logger.h"
#include "postform/rtt/rtt.h"
#include "postform/rtt/transport.h"
#include "postform/serial_logger.h"
#include "systick_config.h"

std::array<std::uint8_t, 1024u> postform_channel_buffer;

extern "C" Postform::Rtt::ControlBlock<1u, 0u> _SEGGER_RTT{
    std::array{
        Postform::Rtt::ChannelDescriptor{
            .name = "postform",
            .buffer = postform_channel_buffer,
        },
    },
    {}};

Postform::Rtt::Transport transport{&_SEGGER_RTT.up_channels[0]};
Postform::SerialLogger<Postform::Rtt::Transport> logger{&transport};

int main() {
  SysTick& systick = SysTick::getInstance();
  systick.init(App::SYSTICK_CLK_HZ);

  LOG_DEBUG(&logger, "Hello world!");
  LOG_INFO(&logger, "Hello world! (Again!)");
  LOG_WARNING(&logger, "Hello world! (And again!)");

  while (true) {
    systick.delay(1'000);
    LOG_ERROR(&logger, "Hello world! (Yet again!)");
  }
}
