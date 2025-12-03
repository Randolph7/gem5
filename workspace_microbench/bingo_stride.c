// Simple stride-based microbenchmark to exercise cache prefetchers.
#include <stdint.h>
#include <stdio.h>

#define N_BYTES (32 * 1024 * 1024ULL) // 32 MiB to exceed L2
#define LINE_SIZE 64
static const size_t stride_list[] = {64, 96, 128, 192}; // bytes

static uint64_t data[N_BYTES / sizeof(uint64_t)] __attribute__((aligned(LINE_SIZE)));

int main(void)
{
    const size_t n = N_BYTES / sizeof(uint64_t);

    for (size_t i = 0; i < n; i++)
        data[i] = i;

    uint64_t sum = 0;
    // Walk the array with several strides; each stride visits twice.
    for (size_t s = 0; s < sizeof(stride_list) / sizeof(stride_list[0]); s++) {
        size_t step = stride_list[s] / sizeof(uint64_t);
        // Offset start to vary region alignment across strides.
        size_t start = (s * 17) % step;
        for (size_t i = start; i < n; i += step)
            sum += data[i];
        for (size_t i = start; i < n; i += step)
            sum += data[i];
    }

    printf("sum=%lu\n", sum);
    return 0;
}
