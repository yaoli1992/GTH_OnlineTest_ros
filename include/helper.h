#ifndef HELPER_H
#define HELPER_H

#include <array>
#include <memory>
#include <cstdint>
#include <string.h>

const int HEIGHT = 240;
const int WIDTH = 304;
static const uint32_t OUTPUT_BUFFER_SIZE = HEIGHT * WIDTH * sizeof(int16_t);
// Helper struct for grouping outgoing data together
struct PhaseOutput
{
    std::shared_ptr<int16_t> phase1;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase2;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase3;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase4;  // has to be pre-allocated!

    /**
     * Constructor.
     * Pre-allocates the depth and confidence buffers.
     */
    PhaseOutput();
};

struct Phase8Output
{
    std::shared_ptr<int16_t> phase1;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase2;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase3;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase4;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase5;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase6;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase7;  // has to be pre-allocated!
    std::shared_ptr<int16_t> phase8;  // has to be pre-allocated!
    /**
     * Constructor.
     * Pre-allocates the depth and confidence buffers.
     */
    Phase8Output();
};

// Helper struct for grouping outgoing data together
struct AlgorithmOutput
{
    std::shared_ptr<float> depth;       // has to be pre-allocated!
    std::shared_ptr<float> confidence;  // has to be pre-allocated!

    /**
     * Constructor.
     * Pre-allocates the depth and confidence buffers.
     */
    AlgorithmOutput();
};

/**
 * Helper template for deleting an array managed by a shared_ptr.
 */
template <typename T>
struct array_deleter
{
    void operator()(const T* ptr)
    {
        delete [] ptr;
    }
};

AlgorithmOutput::AlgorithmOutput()
    : depth(new float[HEIGHT * WIDTH], array_deleter<float>())
    , confidence(new float[HEIGHT * WIDTH], array_deleter<float>())
{
    memset(depth.get(), 0, HEIGHT * WIDTH * sizeof(float));
    memset(confidence.get(), 0, HEIGHT * WIDTH * sizeof(float));
}


PhaseOutput::PhaseOutput()
    : phase1(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase2(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase3(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase4(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
{
    memset(phase1.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase2.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase3.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase4.get(), 0, OUTPUT_BUFFER_SIZE);
}

Phase8Output::Phase8Output()
    : phase1(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase2(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase3(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase4(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase5(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase6(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase7(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
    , phase8(new int16_t[HEIGHT * WIDTH], array_deleter<int16_t>())
{
    memset(phase1.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase2.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase3.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase4.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase5.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase6.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase7.get(), 0, OUTPUT_BUFFER_SIZE);
    memset(phase8.get(), 0, OUTPUT_BUFFER_SIZE);
}
#endif // HELPER_H
