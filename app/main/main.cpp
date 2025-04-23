#include "main.h"
#include "bmm150.hpp"
#include "usart.h"

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_USART2_UART_Init();

    using namespace bmm150;

    auto bmm150 = BMM150{};

    bmm150.initialize();

    while (1) {
        auto scaled = bmm150.get_mag_xyz_scaled();
    }

    bmm150.deinitialize();
}
